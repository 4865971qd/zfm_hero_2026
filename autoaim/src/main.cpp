#include "armor.hpp"
#include "detector.hpp"
#include "solver.hpp"
#include "tracker.hpp"
#include "hardware/camera.hpp"
#include "hardware/video_player.hpp"
#include "hardware/serial.hpp"
#include "utils/exiter.hpp"
#include "utils/logger.hpp"
#include "utils/plotter.hpp"
#include "utils/recorder.hpp"
#include "utils/math.hpp"
#include <fmt/core.h>
#include <opencv2/highgui.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <list>
#include <memory>
#include <thread>

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fmt::print(stderr, "Usage: {} <config.yaml>\n", argv[0]);
        return 1;
    }

    auto cfg = YAML::LoadFile(argv[1]);

    // Logging.
    auto log_level = cfg["debug"]["log_level"].as<std::string>("info");
    autoaim::setLogLevel(log_level);

    // Exit signal handling.
    autoaim::Exiter exiter;

    // Camera or video playback.
    bool video_mode = cfg["video"]["enable"].as<bool>(false);
    std::unique_ptr<autoaim::hardware::VideoPlayer> video_player;
    std::unique_ptr<autoaim::hardware::HikCamera> camera;

    if (video_mode) {
        video_player = std::make_unique<autoaim::hardware::VideoPlayer>(
            cfg["video"]["path"].as<std::string>(""),
            cfg["video"]["fps"].as<double>(110.0),
            cfg["video"]["loop"].as<bool>(false));
    } else {
        camera = std::make_unique<autoaim::hardware::HikCamera>(
            cfg["camera"]["device_index"].as<int>(0));
        camera->setExposureTime(cfg["camera"]["exposure_time"].as<double>(2500.0));
        camera->setGain(cfg["camera"]["gain"].as<double>(12.0));
        camera->setFrameRate(cfg["camera"]["frame_rate"].as<double>(-1.0));
    }

    // Serial port.
    autoaim::hardware::Serial serial(
        cfg["serial"]["port_name"].as<std::string>("/dev/ttyACM_mcu"),
        cfg["serial"]["baudrate"].as<int>(115200));

    // Core modules.
    autoaim::Detector detector(cfg);
    autoaim::Solver solver(cfg);
    autoaim::Tracker tracker(cfg);

    // Debug outputs.
    autoaim::Plotter plotter;
    autoaim::Recorder recorder(cfg["debug"]["record_fps"].as<double>(110.0));
    bool show_img = cfg["debug"]["show_image"].as<bool>(true);
    bool show_det = cfg["debug"]["show_detection"].as<bool>(true);
    bool show_obs = cfg["debug"]["show_observation"].as<bool>(true);
    bool show_sel = cfg["debug"]["show_selected"].as<bool>(true);
    bool do_plot = cfg["debug"]["enable_plotter"].as<bool>(true);
    bool do_record = cfg["debug"]["enable_record"].as<bool>(false);

    autoaim::getLogger()->info("AutoAim initialized. Waiting for data...");

    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    autoaim::hardware::SerialRx rx;

    while (!exiter.exit()) {
        // 1. Capture image.
        bool got_img = false;
        if (video_mode && video_player) {
            got_img = video_player->read(img, t);
        } else if (camera) {
            got_img = camera->read(img, t);
        }
        if (!got_img) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 2. Read serial data without blocking the vision loop.
        static autoaim::hardware::SerialRx last_rx;
        bool got_rx = serial.isOpen() ? serial.receive(rx) : false;
        if (!got_rx && serial.isOpen()) {
            rx = last_rx;
        } else if (got_rx) {
            last_rx = rx;
        }

        // 3. Update IMU pose. The MCU protocol uses degrees; Eigen uses radians.
        const double imu_roll = rx.roll * M_PI / 180.0;
        const double imu_pitch = -rx.pitch * M_PI / 180.0;
        const double imu_yaw = rx.yaw * M_PI / 180.0;
        solver.setImu(imu_roll, imu_pitch, imu_yaw);

        static unsigned long long imu_log_sequence = 0;
        if (++imu_log_sequence % 200 == 1) {
            autoaim::getLogger()->debug(
                "[Outpost][IMU] raw_deg=({:.2f},{:.2f},{:.2f}) "
                "solver_rad=({:.4f},{:.4f},{:.4f})",
                rx.roll, rx.pitch, rx.yaw, imu_roll, imu_pitch, imu_yaw);
        }

        // 4. Detect armors.
        auto armors = detector.detect(img);
        std::list<autoaim::Armor> armor_list(armors.begin(), armors.end());

        // 5. Estimate armor pose.
        for (auto it = armor_list.begin(); it != armor_list.end();) {
            if (!solver.solveArmor(*it)) {
                it = armor_list.erase(it);
            } else {
                ++it;
            }
        }

        // 6. Track target.
        tracker.track(armor_list, t);

        // 7. Solve aim command.
        autoaim::GimbalCommand cmd;
        if (tracker.hasTarget()) {
            auto target = tracker.getTarget();
            target.current_time = std::chrono::duration<double>(t.time_since_epoch()).count();
            cmd = solver.solve(target, target.current_time);
        } else {
            solver.resetCommandDeadZone();
        }

        // 8. Send command.
        autoaim::hardware::SerialTx tx;
        tx.yaw = static_cast<float>(cmd.yaw);
        tx.pitch = static_cast<float>(cmd.pitch);
        tx.distance = static_cast<float>(cmd.distance);
        tx.fire = cmd.fire_advice ? 0x01 : 0x00;
        serial.send(tx);

        // Runtime FPS statistics.
        static int fps_cnt = 0;
        static auto fps_t0 = std::chrono::steady_clock::now();
        static double fps_val = 0.0;
        fps_cnt++;
        auto fps_now = std::chrono::steady_clock::now();
        double fps_elapsed = std::chrono::duration<double>(fps_now - fps_t0).count();
        if (fps_cnt % 10000 == 0) {
            fps_val = fps_cnt / fps_elapsed;
            autoaim::getLogger()->info("Run FPS: {:.1f}", fps_val);
            fps_cnt = 0;
            fps_t0 = fps_now;
        }

        // Display FPS, refreshed at a lower rate to keep the image stable.
        static double display_fps = 0.0;
        static int disp_cnt = 0;
        static auto disp_t0 = std::chrono::steady_clock::now();
        disp_cnt++;
        double disp_elapsed = std::chrono::duration<double>(fps_now - disp_t0).count();
        if (disp_elapsed >= 0.5) {
            display_fps = disp_cnt / disp_elapsed;
            disp_cnt = 0;
            disp_t0 = fps_now;
        }

        // 9. Debug image, rate-limited to reduce GUI overhead.
        static auto show_t0 = std::chrono::steady_clock::now();
        if (show_img) {
            auto show_now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(show_now - show_t0).count();
            if (dt >= 0.033) {
                show_t0 = show_now;

                if (show_obs) {
                    for (auto &armor : armor_list) {
                        if (armor.number != "outpost") continue;
                        auto pts = armor.landmarks();
                        if (pts.size() != 4) continue;
                        for (int k = 0; k < 4; k++) {
                            cv::line(img, pts[k], pts[(k + 1) % 4],
                                     cv::Scalar(255, 255, 0), 2);
                        }
                        cv::putText(
                            img, "OBS",
                            pts[0] + cv::Point2f(0, -10),
                            cv::FONT_HERSHEY_DUPLEX, 0.7,
                            cv::Scalar(255, 255, 0), 2);
                    }
                }

                if (show_det) {
                    for (auto &armor : armor_list) {
                        cv::Scalar color = (armor.type == autoaim::ArmorType::LARGE) ?
                            cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
                        auto pts = armor.landmarks();
                        for (size_t i = 0; i < pts.size(); i++) {
                            cv::line(img, pts[i], pts[(i + 1) % pts.size()],
                                     color, 2);
                        }

                        std::string type_str =
                            (armor.type == autoaim::ArmorType::LARGE) ? "BIG" : "SM";
                        cv::putText(
                            img,
                            fmt::format("{} {} {:.1f}m", type_str, armor.number,
                                        armor.xyz_in_world.norm()),
                            armor.center + cv::Point2f(-50, -18),
                            cv::FONT_HERSHEY_DUPLEX, 0.7, color, 2);
                    }
                }

                if (show_sel && tracker.hasTarget() && solver.hasSelectedArmor()) {
                    auto target = tracker.getTarget();
                    auto pts = solver.reproject(solver.selectedArmorPosition(),
                                                solver.selectedArmorYaw(),
                                                target.armor_type,
                                                target.armor_num == 3);
                    if (pts.size() == 4) {
                        const std::string selected_label = target.armor_num == 3 ?
                            fmt::format("SEL P{}", solver.selectedArmorIndex()) : "SEL";
                        const bool outpost_preselect =
                            target.armor_num == 3 &&
                            solver.selectedArmorIndex() != target.outpost_observed_plate;
                        if (outpost_preselect) {
                            cv::Point2f center(0.0f, 0.0f);
                            for (const auto &pt : pts) {
                                center += pt;
                            }
                            center *= 0.25f;
                            constexpr float cross_len = 8.0f;
                            cv::line(img,
                                     center + cv::Point2f(-cross_len, 0.0f),
                                     center + cv::Point2f(cross_len, 0.0f),
                                     cv::Scalar(255, 0, 255), 2);
                            cv::line(img,
                                     center + cv::Point2f(0.0f, -cross_len),
                                     center + cv::Point2f(0.0f, cross_len),
                                     cv::Scalar(255, 0, 255), 2);
                            cv::putText(img, selected_label,
                                        center + cv::Point2f(10, -10),
                                        cv::FONT_HERSHEY_DUPLEX, 0.6,
                                        cv::Scalar(255, 0, 255), 2);
                        } else {
                            for (int k = 0; k < 4; k++) {
                                cv::line(img, pts[k], pts[(k + 1) % 4],
                                         cv::Scalar(255, 0, 255), 3);
                            }
                            cv::putText(img, selected_label,
                                        pts[0] + cv::Point2f(0, -10),
                                        cv::FONT_HERSHEY_DUPLEX, 0.7,
                                        cv::Scalar(255, 0, 255), 2);
                        }
                    }
                }

                std::string state = tracker.hasTarget() ?
                    fmt::format("TRACK {}", tracker.trackedId()) : "LOST";
                cv::putText(img, state, {10, 30}, cv::FONT_HERSHEY_DUPLEX, 0.8,
                            tracker.hasTarget() ? cv::Scalar{0, 255, 0} :
                                                  cv::Scalar{0, 0, 255},
                            2);

                cv::putText(img, fmt::format("FPS: {:.1f}", display_fps),
                            cv::Point(img.cols - 130, 30),
                            cv::FONT_HERSHEY_DUPLEX, 0.8,
                            cv::Scalar{0, 255, 0}, 2);

                cv::imshow("autoaim", img);
                cv::waitKey(1);
            }
        }

        if (do_plot && tracker.hasTarget()) {
            auto target = tracker.getTarget();
            const double dist = std::sqrt(target.state[0] * target.state[0] +
                                          target.state[2] * target.state[2] +
                                          target.state[4] * target.state[4]);
            const double yaw_deg = target.state[6] * 180.0 / 3.14159265358979323846;
            plotter.plot(fmt::format(
                R"({{"dist":{:.6f},"yaw":{:.6f},"v_yaw":{:.6f},"cmd_yaw":{:.6f},"cmd_pitch":{:.6f},"fire":{}}})",
                dist, yaw_deg, target.state[7], cmd.yaw, cmd.pitch,
                cmd.fire_advice ? 1 : 0));
        }

        if (do_record) {
            Eigen::Quaterniond q =
                autoaim::math::eulerToQuat(imu_roll, imu_pitch, imu_yaw);
            recorder.record(img, q, t);
        }
    }

    autoaim::getLogger()->info("AutoAim exited.");
    return 0;
}
