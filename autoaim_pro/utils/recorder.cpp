#include "recorder.hpp"
#include "math.hpp"
#include <fmt/chrono.h>
#include <fmt/core.h>
#include <filesystem>

namespace autoaim {

Recorder::Recorder(double fps) : fps_(fps) {
    start_t_ = std::chrono::steady_clock::now();
    last_t_ = start_t_;
    auto folder = "records";
    auto name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
    txt_path_ = fmt::format("{}/{}.txt", folder, name);
    vid_path_ = fmt::format("{}/{}.avi", folder, name);
    std::filesystem::create_directory(folder);
}

Recorder::~Recorder() {
    stop_ = true;
    if (save_thread_.joinable()) save_thread_.join();
    if (init_done_) {
        txt_out_.close();
        vid_out_.release();
    }
}

void Recorder::init(const cv::Mat &img) {
    txt_out_.open(txt_path_);
    vid_out_ = cv::VideoWriter(vid_path_,
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps_, img.size());
    save_thread_ = std::thread(&Recorder::saveLoop, this);
    init_done_ = true;
}

void Recorder::saveLoop() {
    while (!stop_) {
        Frame f;
        {
            std::lock_guard<std::mutex> lock(q_mtx_);
            if (queue_.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            f = queue_.front();
            queue_.pop();
        }
        if (f.img.empty()) continue;
        vid_out_.write(f.img);
        Eigen::Vector4d xyzw = f.q.coeffs();
        double since = math::deltaTime(f.t, start_t_);
        txt_out_ << fmt::format("{} {} {} {} {}\n", since,
                                xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
    }
}

void Recorder::record(const cv::Mat &img,
                      const Eigen::Quaterniond &q,
                      const std::chrono::steady_clock::time_point &t) {
    if (img.empty()) return;
    if (!init_done_) init(img);
    double since = math::deltaTime(t, last_t_);
    if (since < 1.0 / fps_) return;
    last_t_ = t;
    std::lock_guard<std::mutex> lock(q_mtx_);
    queue_.push({img, q, t});
}

}  // namespace autoaim
