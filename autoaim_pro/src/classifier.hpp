#ifndef AUTO_AIM_CLASSIFIER_HPP_
#define AUTO_AIM_CLASSIFIER_HPP_

#include "armor.hpp"
#include <mutex>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>

namespace autoaim {

// ONNX 数字分类器 (zfm_ws1 NumberClassifier 移植)
class NumberClassifier {
public:
    NumberClassifier(const std::string &model_path, const std::string &label_path,
                     double threshold = 0.7,
                     const std::vector<std::string> &ignore_classes = {});

    bool enabled() const noexcept { return enabled_; }

    /// 从装甲板区域提取数字 ROI
    cv::Mat extractNumber(const cv::Mat &src, const Armor &armor) const noexcept;

    /// 分类装甲板数字
    void classify(const cv::Mat &src, Armor &armor) noexcept;

    /// 过滤低置信度 / 忽略类别 / 类型不匹配的装甲板
    void eraseIgnoreClasses(std::vector<Armor> &armors) noexcept;

    double threshold;

private:
    bool enabled_ = false;
    std::mutex mtx_;
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    std::vector<std::string> ignore_classes_;
};

}  // namespace autoaim

#endif  // AUTO_AIM_CLASSIFIER_HPP_
