#ifndef RM_UTILS_MANUAL_COMPENSATOR_HPP_
#define RM_UTILS_MANUAL_COMPENSATOR_HPP_

#include <vector>
#include <algorithm>
#include <string>
#include <sstream>

namespace zfm {
  constexpr size_t NORMAL_STR_NUM = 6;

  class LineRegion{
    public:
      LineRegion(const double l, const double u): 
      lowwer_(l), upper_(u) {}

      bool checkPoint(const double p) const {
        if (p > lowwer_ && p < upper_ ) { 
          return true; 
        }
        return false;
      }

      bool checkIntersection(const LineRegion& l) const {
        if (checkPoint(l.lowwer_) || 
            checkPoint(l.upper_)) {
          return true;
        }
        return false;
      }

    private:
      double lowwer_;
      double upper_;
  };

  class ManualCompensator {
    public:
      struct HeightMapNode {
        HeightMapNode(const LineRegion &region,  const double pitch, const double yaw): 
        height_region(region), pitch_offset(pitch), yaw_offset(yaw) {}
        LineRegion height_region;
        double pitch_offset;
        double yaw_offset;
      };

      struct DistMapNode {
        DistMapNode(const LineRegion &region, const std::vector<HeightMapNode> h_nodes):
        dist_region(region), height_map(h_nodes) {}
        LineRegion dist_region;
        std::vector<HeightMapNode> height_map;
      };

      ManualCompensator() = default;

      std::vector<double> angleHardCorrect(const double dist, const double height);

      bool updateMap(const LineRegion& d_region,
                     const LineRegion& h_region,
                     const double pitch_offset,
                     const double yaw_offset);

      bool updateMapByStr(const std::string& str);

      bool updateMapFlow(const std::vector<std::string> strs) {
        for (const auto& str: strs) {
          if (!updateMapByStr(str)) {
            return false;
          }
        }
        return true;
      }

    private:
      bool parseStr(const std::string& str, std::vector<double>& nums); 

      std::vector<DistMapNode> angle_offset_map_;
  };
}  // namespace zfm
#endif