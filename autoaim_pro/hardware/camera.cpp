#include "camera.hpp"

#if HAS_MVS
#include "utils/logger.hpp"
#include <cstring>
#include <immintrin.h>

namespace autoaim::hardware {

HikCamera::HikCamera(int device_index) : device_index_(device_index) {
    if (!initDevice()) {
        return;
    }
    setExposureTime(2500.0);
    setGain(12.0);
    setFrameRate(-1.0);
    if (!startGrabbing()) {
        return;
    }

    // 预分配双缓冲
    size_t buf_size = width_ * height_ * 3;
    buf_[0].resize(buf_size);
    buf_[1].resize(buf_size);

    // 启动取图线程
    running_ = true;
    capture_thread_ = std::thread(&HikCamera::captureLoop, this);

    getLogger()->info("HikCamera initialized: {}x{} @ device_index={}", width_, height_, device_index_);
}

HikCamera::~HikCamera() {
    running_ = false;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    if (handle_) {
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(&handle_);
    }
}

void HikCamera::captureLoop() {
    while (running_) {

        MV_FRAME_OUT out_frame;
        memset(&out_frame, 0, sizeof(out_frame));

        int ret = MV_CC_GetImageBuffer(handle_, &out_frame, 1000);
        if (ret != MV_OK) {
            continue;
        }

        int w = out_frame.stFrameInfo.nWidth;
        int h = out_frame.stFrameInfo.nHeight;

        // OpenCV Bayer→BGR 转换
        cv::Mat bayer_mat(h, w, CV_8UC1, out_frame.pBufAddr);
        cv::Mat rgb_mat(h, w, CV_8UC3, buf_[front_].data());

        int cvt_code = cv::COLOR_BayerRG2BGR;
        switch (out_frame.stFrameInfo.enPixelType) {
            case PixelType_Gvsp_BayerBG8: cvt_code = cv::COLOR_BayerBG2BGR; break;
            case PixelType_Gvsp_BayerGB8: cvt_code = cv::COLOR_BayerGB2BGR; break;
            case PixelType_Gvsp_BayerGR8: cvt_code = cv::COLOR_BayerGR2BGR; break;
            default: break;
        }
        cv::cvtColor(bayer_mat, rgb_mat, cvt_code);

        last_ts_ = std::chrono::steady_clock::now();

        // 交换缓冲
        back_ = front_;
        front_ = (front_ + 1) % 2;
        frame_seq_.fetch_add(1);
        cv_.notify_one();

        MV_CC_FreeImageBuffer(handle_, &out_frame);
    }
}

bool HikCamera::read(cv::Mat &bgr_img, std::chrono::steady_clock::time_point &timestamp) {
    // 等待新帧（自旋等待，微秒级响应）
    static uint64_t last_seq = 0;
    for (int i = 0; i < 10000000; i++) {
        if (!running_) return false;
        uint64_t seq = frame_seq_.load(std::memory_order_acquire);
        if (seq != last_seq) {
            last_seq = seq;
            int idx = back_.load(std::memory_order_acquire);
            if (idx >= 0) {
                bgr_img = cv::Mat(height_, width_, CV_8UC3, const_cast<uint8_t*>(buf_[idx].data()));
                timestamp = last_ts_;
                return true;
            }
        }
        _mm_pause();
    }
    return false;
}

bool HikCamera::initDevice() {
    MV_CC_DEVICE_INFO_LIST dev_list;
    memset(&dev_list, 0, sizeof(dev_list));

    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &dev_list);
    if (ret != MV_OK || dev_list.nDeviceNum == 0) {
        getLogger()->error("No camera found!");
        return false;
    }

    int idx = (device_index_ < static_cast<int>(dev_list.nDeviceNum)) ? device_index_ : 0;
    ret = MV_CC_CreateHandle(&handle_, dev_list.pDeviceInfo[idx]);
    if (ret != MV_OK) return false;

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) { MV_CC_DestroyHandle(&handle_); return false; }

    MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
    MV_CC_SetEnumValue(handle_, "AcquisitionMode", 1);

    MV_CC_SetIntValue(handle_, "Width", 1440);
    MV_CC_SetIntValue(handle_, "Height", 1080);

    return true;
}

bool HikCamera::startGrabbing() {
    int ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) return false;
    grabbing_ = true;
    return true;
}

void HikCamera::setExposureTime(double us) {
    if (handle_) MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(us));
}
void HikCamera::setGain(double gain) {
    if (handle_) MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain));
}
void HikCamera::setFrameRate(double fps) {
    if (!handle_) return;
    if (fps <= 0) {
        MVCC_FLOATVALUE fv;
        if (MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &fv) == MV_OK)
            fps = fv.fMax;
    }
    MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", static_cast<float>(fps));
}

}  // namespace autoaim::hardware
#endif  // HAS_MVS
