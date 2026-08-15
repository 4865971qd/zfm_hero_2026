#ifndef AUTO_AIM_DETECTOR_HPP_
#define AUTO_AIM_DETECTOR_HPP_

#include "armor.hpp"
#include "classifier.hpp"
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace autoaim {

// 传统 CV 检测器 (zfm_ws1 armor_detector 移植)
class TraditionalDetector {
public:
    TraditionalDetector(const YAML::Node &cfg);

    // 运行时切换敌方颜色 (0=红, 1=蓝)
    void setEnemyColor(int c) { enemy_color_ = c; }

    std::vector<Armor> detect(const cv::Mat &bgr_img);

    int binary_thres_ = 90;
    double light_min_ratio_ = 0.0001, light_max_ratio_ = 1.0;
    double light_max_angle_ = 40.0;
    int color_diff_thresh_ = 20;
    double armor_min_light_ratio_ = 0.8;
    double armor_min_small_dist_ = 0.8, armor_max_small_dist_ = 3.5;
    double armor_min_large_dist_ = 3.5, armor_max_large_dist_ = 8.0;
    double armor_max_angle_ = 35.0;

    int enemy_color_ = 1;

    std::unique_ptr<NumberClassifier> classifier_;
};

// 统一检测接口
class Detector {
public:
    Detector(const YAML::Node &cfg);

    // 运行时切换敌方颜色 (0=红, 1=蓝)
    void setEnemyColor(int color);

    std::vector<Armor> detect(const cv::Mat &img);

private:
    std::unique_ptr<TraditionalDetector> trad_;
};

}  // namespace autoaim

#endif  // AUTO_AIM_DETECTOR_HPP_
