#include "src/armor.hpp"
#include "src/detector.hpp"
#include "src/solver.hpp"
#include "src/tracker.hpp"
#include "hardware/video_player.hpp"
#include "utils/logger.hpp"
#include "utils/math.hpp"
#include <fmt/core.h>
#include <opencv2/highgui.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <list>

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fmt::print(stderr, "Usage: {} <config.yaml> [--show]\n", argv[0]);
        fmt::print(stderr, "  --show  显示画面调试\n");
        fmt::print(stderr, "  q       退出\n");
        fmt::print(stderr, "  空格    暂停/继续\n");
        return 1;
    }

    using namespace autoaim;

    bool show = (argc >= 3 && std::string(argv[2]) == "--show");

    auto cfg = YAML::LoadFile(argv[1]);

    // 日志级别
    autoaim::setLogLevel(cfg["debug"]["log_level"].as<std::string>("info"));

    bool show_det = cfg["debug"]["show_detection"].as<bool>(true);
    bool show_obs = cfg["debug"]["show_observation"].as<bool>(true);
    bool show_sel = cfg["debug"]["show_selected"].as<bool>(true);

    autoaim::hardware::VideoPlayer player(
        cfg["video"]["path"].as<std::string>(""),
        cfg["video"]["fps"].as<double>(110.0),
        false);

    if (!player.isOpened()) {
        fmt::print(stderr, "Cannot open video\n");
        return 1;
    }

    autoaim::Detector detector(cfg);
    autoaim::Solver solver(cfg);
    autoaim::Tracker tracker(cfg);

    cv::Mat img, display;
    std::chrono::steady_clock::time_point t;
    solver.setImu(0, 0, 0);

    int frame_cnt = 0, detect_cnt = 0, track_cnt = 0;
    bool paused = false;
    std::chrono::steady_clock::time_point last_show_t = std::chrono::steady_clock::now();

    while (player.read(img, t)) {
        if (show) {
            auto key = cv::waitKey(paused ? 0 : 1);
            if (key == 'q' || key == 'Q') break;
            if (key == ' ') paused = !paused;
            if (paused) continue;
        }

        frame_cnt++;
        auto armors = detector.detect(img);
        std::list<autoaim::Armor> armor_list(armors.begin(), armors.end());

        for (auto it = armor_list.begin(); it != armor_list.end();) {
            if (!solver.solveArmor(*it)) {
                it = armor_list.erase(it);
            } else {
                ++it;
            }
        }
        tracker.track(armor_list, t);

        // ====== 预测框(整车观测) vs 检测框 对比日志 (调试用) ======
        if (tracker.hasTarget() && !armor_list.empty() && frame_cnt % 20 == 0) {
            auto target = tracker.getTarget();
            auto preds = target.armorXYZA();
            for (size_t i = 0; i < preds.size(); ++i) {
                const Eigen::Vector3d pp = preds[i].head<3>();
                const autoaim::Armor *best_a = nullptr;
                double best_d3 = 1e9;
                for (const auto &a : armor_list) {
                    double d3 = (a.xyz_in_world - pp).norm();
                    if (d3 < best_d3) { best_d3 = d3; best_a = &a; }
                }
                if (best_a) {
                    auto pts = solver.reproject(pp, preds[i][3],
                        target.armor_type, target.armor_num == 3);
                    cv::Point2f pc;
                    if (pts.size() >= 4)
                        pc = (pts[0] + pts[1] + pts[2] + pts[3]) * 0.25f;
                    fmt::print("[MATCH] pred{} world=({:.3f},{:.3f},{:.3f}) "
                        "det world=({:.3f},{:.3f},{:.3f}) d3d={:.3f}m dpx={:.1f}\n",
                        i, pp.x(), pp.y(), pp.z(),
                        best_a->xyz_in_world.x(), best_a->xyz_in_world.y(),
                        best_a->xyz_in_world.z(),
                        best_d3, cv::norm(best_a->center - pc));
                }
            }
        }

        if (!armor_list.empty()) detect_cnt++;
        if (tracker.hasTarget()) track_cnt++;

        // ====== 可视化叠加 ======
        if (show) {
            display = img.clone();

            // 整车观测可视化: 画推测装甲板轮廓框 (红色)
            if (show_obs && tracker.hasTarget()) {
                auto target = tracker.getTarget();
                for (auto &xyza : target.armorXYZA()) {
                    auto pts = solver.reproject(xyza.head<3>(), xyza[3],
                        target.armor_type, target.armor_num == 3);
                    if (pts.size() == 4) {
                        for (int k = 0; k < 4; k++)
                            cv::line(display, pts[k], pts[(k+1)%4], {0, 0, 255}, 2);
                    }
                }
            }

            // 识别可视化 (装甲板框+标签)
            if (show_det) {
                for (auto &a : armor_list) {
                    cv::Scalar box_color = (a.type == ArmorType::LARGE) ?
                        cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
                    auto pts = a.landmarks();
                    for (size_t i = 0; i < pts.size(); i++)
                        cv::line(display, pts[i], pts[(i+1)%pts.size()], box_color, 2);
                    std::string type_str = (a.type == ArmorType::LARGE) ? "BIG" : "SM";
                    cv::putText(display,
                        fmt::format("{} {} {:.1f}m", type_str, a.number, a.xyz_in_world.norm()),
                        a.center + cv::Point2f(-50, -18),
                        cv::FONT_HERSHEY_DUPLEX, 0.7, box_color, 2);
                }
            }

            // 选中装甲板可视化 (洋红色)
            if (show_sel && tracker.hasTarget() && solver.hasSelectedArmor()) {
                auto target = tracker.getTarget();
                auto pts = solver.reproject(solver.selectedArmorPosition(),
                                            solver.selectedArmorYaw(),
                                            target.armor_type,
                                            target.armor_num == 3);
                if (pts.size() == 4) {
                    for (int k = 0; k < 4; k++)
                        cv::line(display, pts[k], pts[(k+1)%4],
                                 cv::Scalar(255, 0, 255), 3);
                    cv::putText(display, "SEL",
                                pts[0] + cv::Point2f(0, -10),
                                cv::FONT_HERSHEY_DUPLEX, 0.7,
                                cv::Scalar(255, 0, 255), 2);
                }
            }

            // status
            std::string state_str = tracker.hasTarget() ?
                fmt::format("TRACK {}", tracker.trackedId()) : "LOST";
            cv::putText(display, state_str, {10, 30},
                cv::FONT_HERSHEY_DUPLEX, 0.8,
                tracker.hasTarget() ? cv::Scalar{0, 255, 0} : cv::Scalar{0, 0, 255}, 2);

            // FPS
            double fps = 1.0 / std::max(0.001, math::deltaTime(t, last_show_t));
            cv::putText(display, fmt::format("FPS: {:.0f}", fps),
                {10, display.rows - 10},
                cv::FONT_HERSHEY_DUPLEX, 0.6, {255, 255, 255}, 1);

            cv::resize(display, display, {}, 0.6, 0.6);
            cv::imshow("offline_test [q=quit SPACE=pause]", display);
        }
        last_show_t = t;

        if (frame_cnt % 100 == 0) {
            fmt::print("Frame {}: {} det, tracking={}\n",
                frame_cnt, armor_list.size(), tracker.hasTarget());
        }
    }

    fmt::print("\n===== Summary =====\n");
    fmt::print("Total frames: {}\n", frame_cnt);
    fmt::print("Frames with detection: {} ({:.1f}%)\n",
        detect_cnt, 100.0 * detect_cnt / frame_cnt);
    fmt::print("Frames tracking target: {} ({:.1f}%)\n",
        track_cnt, 100.0 * track_cnt / frame_cnt);

    return 0;
}
