#include "armor.hpp"

#include "utils/math.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace autoaim {

Light::Light(const std::vector<cv::Point> &contour)
    : cv::RotatedRect(cv::minAreaRect(contour)), color(EnemyColor::WHITE) {
    float n = static_cast<float>(contour.size());
    center = std::accumulate(contour.begin(), contour.end(), cv::Point2f(0, 0),
                             [n](const cv::Point2f &a, const cv::Point &b) {
                                 return a + cv::Point2f(b) / n;
                             });

    cv::Point2f p[4];
    points(p);
    std::sort(p, p + 4, [](const auto &a, const auto &b) { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;
    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);
    axis = (top - bottom) / length;
    tilt_angle = std::atan2(std::abs(top.x - bottom.x),
                            std::abs(top.y - bottom.y)) *
                 180.0f / CV_PI;
}

Armor::Armor(const Light &l1, const Light &l2) {
    if (l1.center.x < l2.center.x) {
        left_light = l1;
        right_light = l2;
    } else {
        left_light = l2;
        right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
    color = left_light.color;
}

std::vector<cv::Point2f> Armor::landmarks() const {
    return {left_light.bottom, left_light.top, right_light.top, right_light.bottom};
}

std::vector<cv::Point3f> Armor::buildObjectPoints(double w, double h, int n_pts) {
    float hw = static_cast<float>(w / 2.0);
    float hh = static_cast<float>(h / 2.0);
    if (n_pts == 6) {
        return {{0.f, hw, -hh}, {0.f, hw, 0.f},  {0.f, hw, hh},
                {0.f, -hw, hh}, {0.f, -hw, 0.f}, {0.f, -hw, -hh}};
    }
    return {{0.f, hw, -hh}, {0.f, hw, hh}, {0.f, -hw, hh}, {0.f, -hw, -hh}};
}

std::vector<Eigen::Vector4d> Target::armorXYZA() const {
    std::vector<Eigen::Vector4d> list;
    const int display_armor_num =
        armor_num == 3 && !outpost_z_calibrated ? 1 : armor_num;
    for (int i = 0; i < display_armor_num; i++) {
        double angle = math::limitRad(state[6] + (armor_num == 3 ?
            outpost_angle_offsets[i] : i * 2 * M_PI / armor_num));
        bool alt = (armor_num == 4) && (i % 2 == 1);
        double r = alt ? another_r : state[8];
        double dz = alt ? (state[9] + d_za) : state[9];
        if (armor_num == 3) {
            double z_offsets[3] = {0.0, state[9], d_za};
            dz = z_offsets[i];
        }
        double x = state[0] - r * std::cos(angle);
        double y = state[2] - r * std::sin(angle);
        double z = state[4] + dz;

        if (armor_num == 3) {
            x = state[0] + r * std::cos(angle);
            y = state[2] + r * std::sin(angle);
        }

        list.push_back({x, y, z, angle});
    }
    return list;
}

}  // namespace autoaim
