#ifndef AUTO_AIM_MANUAL_COMPENSATOR_HPP_
#define AUTO_AIM_MANUAL_COMPENSATOR_HPP_

#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

namespace autoaim {

class ManualCompensator {
public:
    struct LineRegion {
        double lo, hi;
        LineRegion(double l, double h) : lo(l), hi(h) {}
        bool contains(double v) const { return v > lo && v < hi; }
        bool overlaps(const LineRegion &o) const { return contains(o.lo) || contains(o.hi); }
    };

    struct HeightNode {
        LineRegion height_region;
        double pitch_offset, yaw_offset;
        HeightNode(const LineRegion &r, double p, double y)
            : height_region(r), pitch_offset(p), yaw_offset(y) {}
    };

    struct DistNode {
        LineRegion dist_region;
        std::vector<HeightNode> height_map;
        DistNode(const LineRegion &r, const std::vector<HeightNode> &h)
            : dist_region(r), height_map(h) {}
    };

    ManualCompensator() = default;

    // 查询 (distance, height) 对应的补偿值 → {pitch_offset, yaw_offset}
    std::vector<double> correct(double dist, double height) const;

    // 逐条加载字符串: "dist_lo dist_hi height_lo height_hi pitch yaw"
    bool loadFromStrings(const std::vector<std::string> &lines);

private:
    bool parseLine(const std::string &line, std::vector<double> &nums);
    bool addEntry(const LineRegion &dr, const LineRegion &hr,
                  double pitch, double yaw);

    std::vector<DistNode> map_;
};

}  // namespace autoaim

#endif  // AUTO_AIM_MANUAL_COMPENSATOR_HPP_
