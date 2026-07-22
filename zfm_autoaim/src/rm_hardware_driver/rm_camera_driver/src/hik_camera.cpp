#include "rm_camera_driver/hik_camera_node.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace hik_camera
{

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

  // 选择相机：pDeviceInfo[device_index]
  device_index_ = this->declare_parameter("device_index", 0);

  // 初始化相机设备
  if (!initializeCamera()) {
    rclcpp::shutdown();
    return;
  }

  // 创建图像发布器
  bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
  auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
  camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

  // 声明相机参数
  declareParameters();

  // 开始图像采集
  if (!startGrabbing()) {
    rclcpp::shutdown();
    return;
  }

  // 加载相机标定信息
  loadCameraInfo();

  // 初始化视频录制
  record_enable_ = this->declare_parameter("record_enable", false);
  if (record_enable_) {
    std::string record_path = this->declare_parameter(
        "record_path", "/tmp/RMUL_record.avi");
    int record_fps = this->declare_parameter("record_fps", 30);
    std::string record_codec = this->declare_parameter("record_codec", "ffv1");
    int fourcc;
    if (record_codec == "raw") {
      fourcc = 0;
    } else if (record_codec == "ffv1") {
      fourcc = cv::VideoWriter::fourcc('F', 'F', 'V', '1');
    } else if (record_codec == "mjpg") {
      fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    } else {
      fourcc = cv::VideoWriter::fourcc('F', 'F', 'V', '1');
    }
    // 文件名插入时间戳，避免每次启动覆盖
    auto ext_pos = record_path.rfind('.');
    std::string base = (ext_pos != std::string::npos)
        ? record_path.substr(0, ext_pos)
        : record_path;
    std::string ext = (ext_pos != std::string::npos)
        ? record_path.substr(ext_pos)
        : ".avi";
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << base << "_" << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << ext;
    record_path = ss.str();
    cv::Size record_size(image_msg_.width, image_msg_.height);
    recorder_ = std::make_unique<zfm::camera_driver::Recorder>(
        record_path, record_fps, record_size, fourcc);
    if (recorder_->start()) {
      RCLCPP_INFO(this->get_logger(), "Recording started: %s (codec=%s, %dx%d@%dfps)",
                  record_path.c_str(), record_codec.c_str(),
                  image_msg_.width, image_msg_.height, record_fps);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Recording FAILED to open: %s (codec=%s, %dx%d@%dfps)",
                   record_path.c_str(), record_codec.c_str(),
                   image_msg_.width, image_msg_.height, record_fps);
      recorder_.reset();
    }
  }

  // 设置动态参数回调
  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

  // 启动图像采集线程
  startCaptureThread();
}

bool HikCameraNode::initializeCamera()
{
  MV_CC_DEVICE_INFO_LIST device_list;
  memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  
  // 枚举USB相机设备
  nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (nRet != MV_OK) {
    RCLCPP_FATAL(this->get_logger(), "Enum devices failed! Error code: 0x%x", nRet);
    return false;
  }
  
  RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

  // 等待相机连接
  while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
    RCLCPP_ERROR(this->get_logger(), "No camera found! Retrying...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  }

  // 创建相机句柄
  int idx = (device_index_ < static_cast<int>(device_list.nDeviceNum)) ? device_index_ : 0;
  nRet = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[idx]);
  if (nRet != MV_OK) {
    RCLCPP_FATAL(this->get_logger(), "Create handle failed! Error code: 0x%x", nRet);
    return false;
  }
  
  // 打开相机设备
  nRet = MV_CC_OpenDevice(camera_handle_);
  nRet = MV_CC_SetIntValue(camera_handle_,"Width",1440);
  nRet = MV_CC_SetIntValue(camera_handle_,"Height",1080);
  image_msg_.width = 1440;
  image_msg_.height = 1080;
  if (nRet != MV_OK) {
    RCLCPP_FATAL(this->get_logger(), "Open device failed! Error code: 0x%x", nRet);
    MV_CC_DestroyHandle(&camera_handle_);
    return false;
  }

  // 获取相机基本信息
  memset(&img_info_, 0, sizeof(MV_IMAGE_BASIC_INFO));
  nRet = MV_CC_GetImageInfo(camera_handle_, &img_info_);
  if (nRet != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "Get image info failed! Error code: 0x%x", nRet);
  }
  
  // 预分配RGB8图像缓冲区（最大分辨率）
  size_t max_rgb_size = img_info_.nHeightMax * img_info_.nWidthMax * 3;
  image_msg_.data.resize(max_rgb_size);
  RCLCPP_INFO(this->get_logger(), "Pre-allocated RGB8 buffer: %zu bytes for %dx%d", 
              max_rgb_size, img_info_.nWidthMax, img_info_.nHeightMax);

  // 初始化像素转换参数（保留以备将来可能回退到SDK转换）
  // memset(&convert_param_, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
  // convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

  return true;
}

bool HikCameraNode::startGrabbing()
{
  // 开始图像采集流
  nRet = MV_CC_StartGrabbing(camera_handle_);
  if (nRet != MV_OK) {
    RCLCPP_FATAL(this->get_logger(), "Start grabbing failed! Error code: 0x%x", nRet);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
    return false;
  }
  return true;
}

void HikCameraNode::loadCameraInfo()
{
  // 加载相机标定参数
  camera_name_ = this->declare_parameter("camera_name", "camera");
  camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
  
  auto camera_info_url = this->declare_parameter("camera_info_url",
                                                "package://rm_bringup/config/camera_info.yaml");
  
  if (camera_info_manager_->validateURL(camera_info_url)) {
    if (camera_info_manager_->loadCameraInfo(camera_info_url)) {
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
      RCLCPP_INFO(this->get_logger(), "Loaded camera info from: %s", camera_info_url.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to load camera info from: %s", camera_info_url.c_str());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
  }
}

void HikCameraNode::startCaptureThread()
{
  // 启动图像采集线程
  capture_thread_ = std::thread{[this]() -> void {
    MV_FRAME_OUT out_frame;
    memset(&out_frame, 0, sizeof(MV_FRAME_OUT));

    RCLCPP_INFO(this->get_logger(), "Image capture thread started!");

    // 设置ROS图像消息的固定参数
    image_msg_.header.frame_id = "camera_optical_frame";
    image_msg_.encoding = "bgr8";

    // 图像采集主循环
    while (rclcpp::ok()) {
      processSingleFrame(out_frame);
    }
  }};
}

void HikCameraNode::processSingleFrame(MV_FRAME_OUT& out_frame)
{
  // 步骤1: 从相机获取原始图像帧
  nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
  
  if (MV_OK != nRet) {
    handleGrabError(nRet);
    return;
  }

  // 步骤2: 执行Bayer到RGB8的像素格式转换（OpenCV SIMD加速）
  if (!convertBayerToRGB8(out_frame)) {
    MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
    return;
  }

  // 步骤2.5: 录制视频
  if (record_enable_ && recorder_) {
    recorder_->addFrame(image_msg_.data);
  }

  // 步骤3: 发布转换后的图像
  publishImage();

  // 步骤4: 释放相机缓冲区
  MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
  fail_conut_ = 0;  // 重置错误计数
}

bool HikCameraNode::convertBayerToRGB8(const MV_FRAME_OUT& out_frame)
{
  // 设置当前帧的图像尺寸
  image_msg_.height = out_frame.stFrameInfo.nHeight;
  image_msg_.width = out_frame.stFrameInfo.nWidth;
  
  // 显示原始Bayer格式信息
  std::string bayer_type = getBayerTypeString(out_frame.stFrameInfo.enPixelType);
  RCLCPP_DEBUG(this->get_logger(), 
               "Bayer frame: %dx%d, type: %s (0x%lx), raw size: %u bytes", 
               out_frame.stFrameInfo.nWidth, 
               out_frame.stFrameInfo.nHeight,
               bayer_type.c_str(),
               static_cast<long unsigned int>(out_frame.stFrameInfo.enPixelType),
               out_frame.stFrameInfo.nFrameLen);

  // 计算RGB8格式所需缓冲区大小
  size_t required_rgb_size = out_frame.stFrameInfo.nWidth * out_frame.stFrameInfo.nHeight * 3;
  
  // 确保缓冲区足够大
  if (image_msg_.data.size() < required_rgb_size) {
    RCLCPP_WARN(this->get_logger(), "Resizing RGB buffer: %zu -> %zu bytes", 
                image_msg_.data.size(), required_rgb_size);
    image_msg_.data.resize(required_rgb_size);
  }

  // ================== 使用OpenCV进行Bayer到RGB8转换（SIMD加速，比SDK快） ==================
  cv::Mat bayer_mat(
    out_frame.stFrameInfo.nHeight,
    out_frame.stFrameInfo.nWidth,
    CV_8UC1,
    out_frame.pBufAddr
  );
  
  cv::Mat rgb_mat(
    out_frame.stFrameInfo.nHeight,
    out_frame.stFrameInfo.nWidth,
    CV_8UC3,
    image_msg_.data.data()
  );
  
  // 根据Bayer格式选择转换类型（输出 BGR 格式，与纯 C++ 版一致）
  int cvt_code = cv::COLOR_BayerRG2BGR;
  switch (out_frame.stFrameInfo.enPixelType) {
    case PixelType_Gvsp_BayerBG8: cvt_code = cv::COLOR_BayerBG2BGR; break;
    case PixelType_Gvsp_BayerGB8: cvt_code = cv::COLOR_BayerGB2BGR; break;
    case PixelType_Gvsp_BayerGR8: cvt_code = cv::COLOR_BayerGR2BGR; break;
    default: /* BayerRG8 */ break;
  }
  cv::cvtColor(bayer_mat, rgb_mat, cvt_code);
  
  if (rgb_mat.empty()) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV cvtColor failed!");
    fail_conut_++;
    return false;
  }

  // ================== 设置RGB8图像参数 ==================
  image_msg_.step = out_frame.stFrameInfo.nWidth * 3;  // RGB8紧凑格式：width * 3 bytes
  
  // 精确调整数据大小（去除预分配的多余空间）
  if (image_msg_.data.size() > required_rgb_size) {
    image_msg_.data.resize(required_rgb_size);
  }
  
  RCLCPP_DEBUG(this->get_logger(), "OpenCV cvtColor successful: %dx%d, RGB data: %zu bytes", 
               image_msg_.width, image_msg_.height, image_msg_.data.size());
  
  return true;
}

std::string HikCameraNode::getBayerTypeString(MvGvspPixelType pixel_type)
{
  switch(pixel_type) {
    case PixelType_Gvsp_BayerRG8: return "BayerRG8";
    case PixelType_Gvsp_BayerGB8: return "BayerGB8";
    case PixelType_Gvsp_BayerGR8: return "BayerGR8";
    case PixelType_Gvsp_BayerBG8: return "BayerBG8";
    case PixelType_Gvsp_Mono8: return "Mono8";
    case PixelType_Gvsp_Mono10: return "Mono10";
    case PixelType_Gvsp_Mono12: return "Mono12";
    case PixelType_Gvsp_RGB8_Packed: return "RGB8";
    case PixelType_Gvsp_BGR8_Packed: return "BGR8";
    case PixelType_Gvsp_YUV422_Packed: return "YUV422";
    default: return "Unknown(" + std::to_string(pixel_type) + ")";
  }
}

void HikCameraNode::publishImage()
{
  // 设置时间戳
  image_msg_.header.stamp = this->now();
  camera_info_msg_.header = image_msg_.header;
  
  // 发布RGB8图像和相机信息
  camera_pub_.publish(image_msg_, camera_info_msg_);
  
  RCLCPP_DEBUG(this->get_logger(), "RGB8 image published: %dx%d", 
               image_msg_.width, image_msg_.height);
}

void HikCameraNode::handleGrabError(int error_code)
{
  RCLCPP_WARN(this->get_logger(), "Get image buffer failed! Error code: 0x%x", error_code);
  fail_conut_++;

  if (fail_conut_ > 5) {
    RCLCPP_FATAL(this->get_logger(), "Camera failed after 5 consecutive attempts!");
    rclcpp::shutdown();
  }
}

HikCameraNode::~HikCameraNode()
{
  if (record_enable_ && recorder_) {
    recorder_->stop();
  }
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (camera_handle_) {
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
  }
  RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
}

void HikCameraNode::declareParameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  MVCC_FLOATVALUE f_value;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  
  // 曝光时间参数
  param_desc.description = "Exposure time in microseconds";
  nRet = MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
  if (nRet == MV_OK) {
    param_desc.integer_range[0].from_value = static_cast<int64_t>(f_value.fMin);
    param_desc.integer_range[0].to_value = static_cast<int64_t>(f_value.fMax);
    double exposure_time = this->declare_parameter("exposure_time", 5000.0, param_desc);
    nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Set exposure time: %f us", exposure_time);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Set exposure time failed! Error code: 0x%x", nRet);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Get exposure time range failed! Error code: 0x%x", nRet);
  }

  // 增益参数
  param_desc.description = "Gain";
  nRet = MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
  if (nRet == MV_OK) {
    param_desc.integer_range[0].from_value = static_cast<int64_t>(f_value.fMin);
    param_desc.integer_range[0].to_value = static_cast<int64_t>(f_value.fMax);
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
    nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Set gain: %f", gain);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Set gain failed! Error code: 0x%x", nRet);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Get gain range failed! Error code: 0x%x", nRet);
  }

  // 帧率参数
  param_desc.description = "Acquisition frame rate (fps), -1 = use max";
  nRet = MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &f_value);
  if (nRet == MV_OK) {
    param_desc.integer_range[0].from_value = -1;
    param_desc.integer_range[0].to_value = static_cast<int64_t>(f_value.fMax);
    double frame_rate = this->declare_parameter("frame_rate", -1.0, param_desc);
    if (frame_rate <= 0) {
      frame_rate = f_value.fMax;
    }
    nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Set frame rate: %f fps", frame_rate);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Set frame rate failed! Error code: 0x%x", nRet);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Get frame rate range failed! Error code: 0x%x", nRet);
  }
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto & param : parameters) {
    try {
      if (param.get_name() == "exposure_time") {
        nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_double());
        if (nRet != MV_OK) {
          result.successful = false;
          result.reason = "Failed to set exposure time, error code: 0x" + std::to_string(nRet);
          RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "Updated exposure time: %f us", param.as_double());
        }
      } else if (param.get_name() == "gain") {
        nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (nRet != MV_OK) {
          result.successful = false;
          result.reason = "Failed to set gain, error code: 0x" + std::to_string(nRet);
          RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "Updated gain: %f", param.as_double());
        }
      } else if (param.get_name() == "frame_rate") {
        double fps = param.as_double();
        if (fps <= 0) {
          MVCC_FLOATVALUE fv;
          if (MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &fv) == MV_OK) {
            fps = fv.fMax;
          }
        }
        nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", fps);
        if (nRet != MV_OK) {
          result.successful = false;
          result.reason = "Failed to set frame rate, error code: 0x" + std::to_string(nRet);
          RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "Updated frame rate: %f fps", fps);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Ignoring unknown parameter: %s", param.get_name().c_str());
      }
    } catch (const rclcpp::ParameterTypeException & ex) {
      result.successful = false;
      result.reason = "Parameter type error: " + std::string(ex.what());
      RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
    }
  }
  return result;
}

}  // namespace hik_camera

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
