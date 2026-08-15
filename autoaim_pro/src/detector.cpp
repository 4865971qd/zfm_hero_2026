#include "detector.hpp"
#include "utils/logger.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace autoaim {

// ============ TraditionalDetector (逐行对照 zfm_ws1 armor_detector.cpp) ============

TraditionalDetector::TraditionalDetector(const YAML::Node &cfg) {
    auto &d = cfg["detector"];
    binary_thres_       = d["binary_thres"].as<int>(90);
    enemy_color_        = d["detect_color"].as<int>(1);
    light_max_angle_    = d["light"]["max_angle"].as<double>(40.0);
    light_min_ratio_    = d["light"]["min_ratio"].as<double>(0.0001);
    light_max_ratio_    = d["light"]["max_ratio"].as<double>(1.0);
    color_diff_thresh_  = d["light"]["color_diff_thresh"].as<int>(20);

    armor_min_light_ratio_  = d["armor"]["min_light_ratio"].as<double>(0.8);
    armor_min_small_dist_   = d["armor"]["min_small_center_distance"].as<double>(0.8);
    armor_max_small_dist_   = d["armor"]["max_small_center_distance"].as<double>(3.5);
    armor_min_large_dist_   = d["armor"]["min_large_center_distance"].as<double>(3.5);
    armor_max_large_dist_   = d["armor"]["max_large_center_distance"].as<double>(8.0);
    armor_max_angle_        = d["armor"]["max_angle"].as<double>(35.0);

    // 分类器 (可选)
    std::string model_path = "../models/lenet.onnx";
    std::string label_path = "../models/label.txt";
    if (d["classifier_model"]) model_path = d["classifier_model"].as<std::string>();
    if (d["classifier_label"]) label_path = d["classifier_label"].as<std::string>();
    double thres = d["classifier_threshold"].as<double>(0.7);
    std::vector<std::string> ignores;
    if (d["ignore_classes"]) ignores = d["ignore_classes"].as<std::vector<std::string>>();
    classifier_ = std::make_unique<NumberClassifier>(model_path, label_path, thres, ignores);
}

// ====== 1. 预处理: 灰度 → 二值化 (对照 zfm_ws1 preprocessImage) ======
static cv::Mat preprocessImage(const cv::Mat &bgr_img, int thres) {
    cv::Mat gray;
    // 注意: zfm_ws1 输入是 RGB，但 OpenCV VideoCapture 给的是 BGR
    // 灰度转换对 BGR/RGB 是一样的
    cv::cvtColor(bgr_img, gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    cv::threshold(gray, binary, thres, 255, cv::THRESH_BINARY);
    return binary;
}

// ====== 2. 灯条过滤 (对照 zfm_ws1 isLight) ======
static bool isLight(const Light &light, double min_ratio, double max_ratio, double max_angle) {
    float ratio = light.width / light.length;
    return (min_ratio < ratio && ratio < max_ratio) && (light.tilt_angle < max_angle);
}

// ====== 3. 找灯条 (对照 zfm_ws1 findLights) ======
static std::vector<Light> findLights(const cv::Mat &bgr_img, const cv::Mat &binary_img,
                                      double min_ratio, double max_ratio, double max_angle,
                                      int color_diff_thresh) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<Light> lights;

    for (const auto &contour : contours) {
        if (contour.size() < 6) continue;

        Light light(contour);

        if (isLight(light, min_ratio, max_ratio, max_angle)) {
            // === 颜色分类: 遍历轮廓内所有像素 (对照 zfm_ws1 原版) ===
            // OpenCV 存储 BGR: [0]=B [1]=G [2]=R
            int sum_r = 0, sum_b = 0;
            for (const auto &pt : contour) {
                auto &px = bgr_img.at<cv::Vec3b>(pt.y, pt.x);
                sum_r += px[2];  // R 通道
                sum_b += px[0];  // B 通道
            }
            int n_pixels = static_cast<int>(contour.size());
            if (std::abs(sum_r - sum_b) * 1.0 / n_pixels > color_diff_thresh) {
                light.color = (sum_r > sum_b) ? EnemyColor::RED : EnemyColor::BLUE;
            }
            // 颜色不符的灯条也保留，light.color 保持 WHITE
            lights.emplace_back(light);
        }
    }

    // 按 x 坐标排序 (左→右)
    std::sort(lights.begin(), lights.end(), [](const Light &a, const Light &b) {
        return a.center.x < b.center.x;
    });
    return lights;
}

// ====== 4. 灯条之间是否有其他灯条 (对照 zfm_ws1 containLight) ======
static bool containLight(int i, int j, const std::vector<Light> &lights) {
    const auto &l1 = lights[i], &l2 = lights[j];
    std::vector<cv::Point2f> pts{l1.top, l1.bottom, l2.top, l2.bottom};
    auto brect = cv::boundingRect(pts);
    double avg_len = (l1.length + l2.length) / 2.0;
    double avg_wid = (l1.width + l2.width) / 2.0;
    for (int k = i + 1; k < j; k++) {
        const auto &tl = lights[k];
        if (tl.width > 2 * avg_wid) continue;
        if (tl.length < 0.5 * avg_len) continue;
        if (brect.contains(tl.top) || brect.contains(tl.bottom) || brect.contains(tl.center))
            return true;
    }
    return false;
}

// ====== 5. 装甲板配对判断 (对照 zfm_ws1 isArmor) ======
static ArmorType isArmor(const Light &l1, const Light &l2,
                          double min_light_ratio, double min_small_dist, double max_small_dist,
                          double min_large_dist, double max_large_dist, double max_angle) {
    float len_ratio = std::min(l1.length, l2.length) / std::max(l1.length, l2.length);
    if (len_ratio < min_light_ratio) return ArmorType::INVALID;

    float avg_len = (l1.length + l2.length) / 2;
    float center_dist = cv::norm(l1.center - l2.center) / avg_len;
    bool dist_ok = (min_small_dist <= center_dist && center_dist < max_small_dist) ||
                   (min_large_dist <= center_dist && center_dist < max_large_dist);
    if (!dist_ok) return ArmorType::INVALID;

    cv::Point2f diff = l1.center - l2.center;
    float angle = std::abs(std::atan2(diff.y, diff.x)) / CV_PI * 180;
    if (angle > 90) angle = 180 - angle;
    if (angle >= max_angle) return ArmorType::INVALID;

    return center_dist > min_large_dist ? ArmorType::LARGE : ArmorType::SMALL;
}

// ====== 6. 配对灯条为装甲板 (对照 zfm_ws1 matchLights) ======
static std::vector<Armor> matchLights(const std::vector<Light> &lights, EnemyColor detect_color,
                                       double min_light_ratio, double min_small_dist, double max_small_dist,
                                       double min_large_dist, double max_large_dist, double max_angle,
                                       double max_large_center_dist) {
    std::vector<Armor> armors;

    for (size_t i = 0; i < lights.size(); i++) {
        if (lights[i].color != detect_color) continue;
        double max_w = lights[i].length * max_large_center_dist;

        for (size_t j = i + 1; j < lights.size(); j++) {
            if (lights[j].color != detect_color) continue;
            if (containLight(i, j, lights)) continue;
            if (lights[j].center.x - lights[i].center.x > max_w) break;

            auto type = isArmor(lights[i], lights[j], min_light_ratio,
                                min_small_dist, max_small_dist,
                                min_large_dist, max_large_dist, max_angle);
            if (type != ArmorType::INVALID) {
                Armor armor(lights[i], lights[j]);
                armor.type = type;
                armors.push_back(armor);
            }
        }
    }
    return armors;
}

// ====== 主检测入口 (对照 zfm_ws1 detect) ======
std::vector<Armor> TraditionalDetector::detect(const cv::Mat &bgr_img) {
    EnemyColor target = (enemy_color_ == 0) ? EnemyColor::RED : EnemyColor::BLUE;

    // 1. 预处理
    cv::Mat binary = preprocessImage(bgr_img, binary_thres_);

    // 2. 找灯条
    auto lights = findLights(bgr_img, binary,
                             light_min_ratio_, light_max_ratio_, light_max_angle_,
                             color_diff_thresh_);

    // 3. 配对装甲板
    auto armors = matchLights(lights, target,
                              armor_min_light_ratio_,
                              armor_min_small_dist_, armor_max_small_dist_,
                              armor_min_large_dist_, armor_max_large_dist_,
                              armor_max_angle_,
                              armor_max_large_dist_);

    // 4. 数字分类
    if (!armors.empty() && classifier_ && classifier_->enabled()) {
        for (auto &armor : armors) {
            armor.number_img = classifier_->extractNumber(bgr_img, armor);
            classifier_->classify(bgr_img, armor);
        }
        classifier_->eraseIgnoreClasses(armors);
    }

    // 5. 优先级 (无分类器时用默认值)
    for (auto &a : armors) {
        if (a.number.empty()) a.number = "?";  // 无分类器回退
        if (a.number == "hero") a.priority = ArmorPriority::first;
        else if (a.number == "3") a.priority = ArmorPriority::second;
        else if (a.number == "4") a.priority = ArmorPriority::third;
        else if (a.number == "5") a.priority = ArmorPriority::fourth;
        else if (a.number == "outpost") a.priority = ArmorPriority::third;
        else if (a.number == "base") a.priority = ArmorPriority::fifth;
        else a.priority = ArmorPriority::fifth;
    }

    return armors;
}

// ============ Detector (统一接口) ============

Detector::Detector(const YAML::Node &cfg) {
    trad_ = std::make_unique<TraditionalDetector>(cfg);
}

void Detector::setEnemyColor(int color) {
    if (trad_) trad_->setEnemyColor(color);
}

std::vector<Armor> Detector::detect(const cv::Mat &img) {
    return trad_ ? trad_->detect(img) : std::vector<Armor>{};
}

}  // namespace autoaim
