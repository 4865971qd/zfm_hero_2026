#ifndef ARMOR_DETECTOR_NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR_NUMBER_CLASSIFIER_HPP_

// std
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>
// third party
#include <opencv2/opencv.hpp>
// project
#include "armor_detector/types.hpp"

namespace zfm::auto_aim {
// Class used to classify the number of the armor, based on the MLP model
class NumberClassifier {
public:
  NumberClassifier(const std::string &model_path,
                   const std::string &label_path,
                   const double threshold,
                   const std::vector<std::string> &ignore_classes = {});

  // Extract the roi image of number from the src
  cv::Mat extractNumber(const cv::Mat &src, const Armor &armor) const noexcept;

  // Classify the number of the armor
  void classify(const cv::Mat &src, Armor &armor) noexcept;

  // Erase the ignore classes
  void eraseIgnoreClasses(std::vector<Armor> &armors) noexcept;

  double threshold;

private:
  std::mutex mutex_;
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};
}  // namespace zfm::auto_aim
#endif  // ARMOR_DETECTOR_NUMBER_CLASSIFIER_HPP_
