#include "classifier.hpp"
#include "utils/logger.hpp"
#include <fmt/format.h>
#include <fstream>
#include <opencv2/imgproc.hpp>

namespace autoaim {

NumberClassifier::NumberClassifier(const std::string &model_path,
                                   const std::string &label_path,
                                   double thre,
                                   const std::vector<std::string> &ignore_classes)
    : threshold(thre), ignore_classes_(ignore_classes) {
    // 检查模型文件是否存在
    std::ifstream test(model_path);
    if (!test.good()) {
        getLogger()->warn("Classifier model not found: {} — number classification disabled", model_path);
        enabled_ = false;
        return;
    }
    test.close();

    try {
        net_ = cv::dnn::readNetFromONNX(model_path);
    } catch (const cv::Exception &e) {
        getLogger()->warn("Failed to load ONNX model: {} — number classification disabled", e.what());
        enabled_ = false;
        return;
    }

    std::ifstream f(label_path);
    if (!f.good()) {
        getLogger()->warn("Label file not found: {} — number classification disabled", label_path);
        enabled_ = false;
        return;
    }
    std::string line;
    while (std::getline(f, line)) class_names_.push_back(line);
    enabled_ = true;
    std::string class_list;
    for (auto &c : class_names_) class_list += c + " ";
    getLogger()->info("Classifier loaded: {} ({} classes: {})", model_path, class_names_.size(), class_list);
}

cv::Mat NumberClassifier::extractNumber(const cv::Mat &src, const Armor &armor) const noexcept {
    constexpr int light_len = 12;
    constexpr int warp_h = 28;
    constexpr int small_w = 32, large_w = 54;
    cv::Size roi_size(20, 28);
    cv::Size input_size(28, 28);

    cv::Point2f src_pts[4] = {armor.left_light.bottom, armor.left_light.top,
                               armor.right_light.top, armor.right_light.bottom};
    int top_y = (warp_h - light_len) / 2 - 1;
    int bot_y = top_y + light_len;
    int warp_w = (armor.type == ArmorType::SMALL) ? small_w : large_w;
    cv::Point2f dst_pts[4] = {{0.f, static_cast<float>(bot_y)}, {0.f, static_cast<float>(top_y)},
                               {static_cast<float>(warp_w - 1), static_cast<float>(top_y)},
                               {static_cast<float>(warp_w - 1), static_cast<float>(bot_y)}};

    cv::Mat num_img;
    auto M = cv::getPerspectiveTransform(src_pts, dst_pts);
    cv::warpPerspective(src, num_img, M, {warp_w, warp_h});
    num_img = num_img(cv::Rect((warp_w - roi_size.width) / 2, 0, roi_size.width, roi_size.height));
    cv::cvtColor(num_img, num_img, cv::COLOR_BGR2GRAY);
    cv::threshold(num_img, num_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::resize(num_img, num_img, input_size);
    return num_img;
}

void NumberClassifier::classify(const cv::Mat &, Armor &armor) noexcept {
    if (!enabled_) return;
    cv::Mat input = armor.number_img / 255.0;
    cv::Mat blob;
    cv::dnn::blobFromImage(input, blob);

    std::lock_guard<std::mutex> lock(mtx_);
    net_.setInput(blob);
    cv::Mat outputs = net_.forward().clone();

    double conf;
    cv::Point max_pt;
    cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &conf, nullptr, &max_pt);
    armor.confidence = static_cast<float>(conf);
    armor.number = class_names_[max_pt.x];
    armor.classification_result = fmt::format("{}:{:.1f}%", armor.number, conf * 100.0);

    /*static int dbg = 0;
    if (++dbg <= 5 || dbg % 50 == 0) {
        getLogger()->debug("Classify: type={} → {} conf={:.2f} (thres={})",
            (armor.type == ArmorType::LARGE ? "BIG" : "SM"),
            armor.classification_result, conf, threshold);
    }*/
}

void NumberClassifier::eraseIgnoreClasses(std::vector<Armor> &armors) noexcept {
    armors.erase(std::remove_if(armors.begin(), armors.end(), [this](const Armor &a) {
        if (a.confidence < threshold) return true;
        for (auto &ic : ignore_classes_)
            if (a.number == ic) return true;
        // 类型/数字不匹配检查 (sentry 不做限制, 其灯条间距可能被判为LARGE)
        if (a.type == ArmorType::LARGE && (a.number == "outpost" || a.number == "2"))
            return true;
        if (a.type == ArmorType::SMALL && a.number == "1") return true;
        return false;
    }), armors.end());
}

}  // namespace autoaim
