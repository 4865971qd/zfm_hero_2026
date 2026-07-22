#include "rm_utils/math/manual_compensator.hpp"

namespace zfm {
bool ManualCompensator::updateMap(const LineRegion& d_region,
                                  const LineRegion& h_region,
                                  const double pitch_offset,
                                  const double yaw_offset) {
    auto target_dist_node = 
      std::find_if(angle_offset_map_.begin(), 
                   angle_offset_map_.end(),
                   [&](const DistMapNode& dist_node) {
     return dist_node.dist_region.checkIntersection(d_region); 
    });

    if (target_dist_node == angle_offset_map_.end()) {
      HeightMapNode height_node(h_region, pitch_offset, yaw_offset);
      std::vector<HeightMapNode> h_nodes{height_node};
      DistMapNode dist_node(d_region, h_nodes);
      angle_offset_map_.emplace_back(dist_node);
    } else {
      auto target_height_node = 
        std::find_if(target_dist_node->height_map.begin(),
                     target_dist_node->height_map.end(),
                     [&](const HeightMapNode& height_node) {
      return height_node.height_region.checkIntersection(h_region);
      });

      if (target_height_node == target_dist_node->height_map.end()) {
        HeightMapNode height_node(h_region, pitch_offset, yaw_offset);
        target_dist_node->height_map.emplace_back(height_node);
      } else {
        return false;
      }
    }
    return true;
  }

std::vector<double> ManualCompensator::angleHardCorrect(const double dist, 
                                           const double height) {
  auto target_dist_node = 
    std::find_if(angle_offset_map_.begin(), 
                  angle_offset_map_.end(),
                  [&](const DistMapNode& dist_node) {
    return dist_node.dist_region.checkPoint(dist); 
  });

  if (target_dist_node != angle_offset_map_.end()) {
    auto target_height_node = 
      std::find_if(target_dist_node->height_map.begin(),
                    target_dist_node->height_map.end(),
                    [&](const HeightMapNode& height_node) {
      return height_node.height_region.checkPoint(height);
    });

    if (target_height_node != target_dist_node->height_map.end()) {
      return {target_height_node->pitch_offset, target_height_node->yaw_offset};    
    }
  }
  return {0.0, 0.0};
} 
  
bool ManualCompensator::parseStr(const std::string& str, 
                                 std::vector<double>& nums) {
  std::stringstream ss(str);
  double num;
  while (!ss.eof()) {
    ss >> num;
    nums.emplace_back(num);
  }
  
  if (nums.size() == 5) {
    nums.push_back(0.0);  // old format compat: yaw_offset defaults to 0
  }
  if (nums.size() != NORMAL_STR_NUM) {
    return false;
  }
  return true;
}

bool ManualCompensator::updateMapByStr(const std::string &str) {
  std::vector<double> nums;

  if (!parseStr(str, nums)) {
    return false;
  }

  LineRegion d_region(nums[0], nums[1]);
  LineRegion h_region(nums[2], nums[3]);
  if (!updateMap(d_region, h_region, nums[4], nums[5])) {
    return false;
  }
  return true;
}
}  // namespace zfm