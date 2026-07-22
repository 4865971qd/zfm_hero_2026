#ifndef HIK_CAMERA_NODE_HPP
#define HIK_CAMERA_NODE_HPP

#include "MvCameraControl.h"
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include <memory>
#include <string>
#include <filesystem>

#include "rm_camera_driver/recorder.hpp"

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options);
  ~HikCameraNode() override;

private:
  // 初始化相关
  bool initializeCamera();
  bool startGrabbing();
  void loadCameraInfo();
  void startCaptureThread();
  
  // 图像处理相关
  void processSingleFrame(MV_FRAME_OUT& out_frame);
  bool convertBayerToRGB8(const MV_FRAME_OUT& out_frame);
  std::string getBayerTypeString(MvGvspPixelType pixel_type);
  void publishImage();
  void handleGrabError(int error_code);
  
  // 参数管理
  void declareParameters();
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // 成员变量
  sensor_msgs::msg::Image image_msg_;
  image_transport::CameraPublisher camera_pub_;
  
  int nRet = MV_OK;
  void* camera_handle_ = nullptr;
  MV_IMAGE_BASIC_INFO img_info_;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;
  
  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  
  int fail_conut_ = 0;
  int device_index_ = 0;
  std::thread capture_thread_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  // Video recording
  bool record_enable_ = false;
  std::unique_ptr<zfm::camera_driver::Recorder> recorder_;
};

}  // namespace hik_camera

#endif  // HIK_CAMERA_NODE_HPP
