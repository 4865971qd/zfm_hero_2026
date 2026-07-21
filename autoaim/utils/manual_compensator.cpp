#include "manual_compensator.hpp"

namespace autoaim {

bool ManualCompensator::parseLine(const std::string &line, std::vector<double> &nums) {
    std::stringstream ss(line);
    double v;
    while (ss >> v) nums.push_back(v);
    if (nums.size() == 5) nums.push_back(0.0);  // old format: yaw defaults to 0
    return nums.size() == 6;
}

bool ManualCompensator::addEntry(const LineRegion &dr, const LineRegion &hr,
                                  double pitch, double yaw) {
    auto dit = std::find_if(map_.begin(), map_.end(), [&](const DistNode &d) {
        return d.dist_region.overlaps(dr);
    });
    if (dit == map_.end()) {
        map_.emplace_back(dr, std::vector<HeightNode>{HeightNode(hr, pitch, yaw)});
    } else {
        auto hit = std::find_if(dit->height_map.begin(), dit->height_map.end(),
                                [&](const HeightNode &h) { return h.height_region.overlaps(hr); });
        if (hit == dit->height_map.end()) {
            dit->height_map.emplace_back(hr, pitch, yaw);
        } else {
            return false;  // 重复区间
        }
    }
    return true;
}

bool ManualCompensator::loadFromStrings(const std::vector<std::string> &lines) {
    for (const auto &line : lines) {
        std::vector<double> nums;
        if (!parseLine(line, nums)) return false;
        if (!addEntry(LineRegion(nums[0], nums[1]),
                      LineRegion(nums[2], nums[3]),
                      nums[4], nums[5]))
            return false;
    }
    return true;
}

std::vector<double> ManualCompensator::correct(double dist, double height) const {
    auto dit = std::find_if(map_.begin(), map_.end(), [&](const DistNode &d) {
        return d.dist_region.contains(dist);
    });
    if (dit != map_.end()) {
        auto hit = std::find_if(dit->height_map.begin(), dit->height_map.end(),
                                [&](const HeightNode &h) { return h.height_region.contains(height); });
        if (hit != dit->height_map.end()) {
            return {hit->pitch_offset, hit->yaw_offset};
        }
    }
    return {0.0, 0.0};
}

}  // namespace autoaim
