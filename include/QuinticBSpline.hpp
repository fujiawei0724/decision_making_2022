/*
 * @Author: fujiawei0724
 * @Date: 2021-11-11 20:38:19
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-11-12 11:37:24
 * @Descripttion: Quintic B-spline in 3D dimension.
 */

#pragma once

#include "Point.hpp"
#include "Path.hpp"
#include "Tools.hpp"
#include "Const.hpp"
#include <vector>
#include <set>
#include <stdlib.h>
#include <unistd.h>
#include <algorithm>
#include <cassert>

namespace Common {

class Point3f {
 public:
    Point3f() = default;
    Point3f(double x, double y, double z) {
        x_ = x;
        y_ = y;
        z_ = z;
    }
    ~Point3f() = default;

    double x_{0.0};
    double y_{0.0};
    double z_{0.0};
};

class QuinticBSplineTrajectory {
 public:
    QuinticBSplineTrajectory(const std::vector<double>& s, const std::vector<double>& d, const std::vector<double>& t) {
        assert(s.size() == t.size() && d.size() == t.size());
        
        // Add additional points information
        int all_points_num = static_cast<int>(t.size()) + 4;
        assert(all_points_num > 5);
        std::vector<double> s_all(all_points_num, 0.0);
        std::vector<double> d_all(all_points_num, 0.0);
        std::vector<double> t_all(all_points_num, 0.0);
        for (int i = 0; i < static_cast<int>(t.size()); i++) {
            s_all[i + 2] = s[i];
            d_all[i + 2] = d[i];
            t_all[i + 2] = t[i];
        }
        s_all[0] = 2.0 * s_all[2] - s_all[4];
        d_all[0] = 2.0 * d_all[2] - d_all[4];
        t_all[0] = 2.0 * t_all[2] - t_all[4];
        s_all[1] = 2.0 * s_all[2] - s_all[3];
        d_all[1] = 2.0 * d_all[2] - d_all[3];
        t_all[1] = 2.0 * t_all[2] - t_all[3];
        s_all[all_points_num - 1] = 2.0 * s_all[all_points_num - 3] - s_all[all_points_num - 5];
        d_all[all_points_num - 1] = 2.0 * d_all[all_points_num - 3] - d_all[all_points_num - 5];
        t_all[all_points_num - 1] = 2.0 * t_all[all_points_num - 3] - t_all[all_points_num - 5];
        s_all[all_points_num - 2] = 2.0 * s_all[all_points_num - 3] - s_all[all_points_num - 4];
        d_all[all_points_num - 2] = 2.0 * d_all[all_points_num - 3] - d_all[all_points_num - 4];
        t_all[all_points_num - 2] = 2.0 * t_all[all_points_num - 3] - t_all[all_points_num - 4];

        // Determine segment number
        segment_num_ = all_points_num - 5;
        s_coefficients_.resize(segment_num_);
        d_coefficients_.resize(segment_num_);
        t_coefficients_.resize(segment_num_);

        // Calculate coefficients of segment 
        for (int i = 0; i < all_points_num - 5; i++) {
            // Calculate s coefficients
            s_coefficients_[i].resize(6);
            s_coefficients_[i][0] = (1.0 / 120.0) * s_all[i] + (26.0 / 120.0) * s_all[i + 1] + (33.0 / 60.0) * s_all[i + 2] + (13.0 / 60.0) * s_all[i + 3] + (1.0 / 120.0) * s_all[i + 4];
            s_coefficients_[i][1] = (-5.0 / 120.0) * s_all[i] + (-50.0 / 120.0) * s_all[i + 1] + (25.0 / 60.0) * s_all[i + 3] + (5.0 / 120.0) * s_all[i + 4];
            s_coefficients_[i][2] = (10.0 / 120.0) * s_all[i] + (20.0 / 120.0) * s_all[i + 1] + (-30.0 / 60.0) * s_all[i + 2] + (10.0 / 60.0) * s_all[i + 3] + (10.0 / 120.0) * s_all[i + 4];
            s_coefficients_[i][3] = (-10.0 / 120.0) * s_all[i] + (20.0 / 120.0) * s_all[i + 1] + (-10.0 / 60.0) * s_all[i + 3] + (10.0 / 120.0) * s_all[i + 4];
            s_coefficients_[i][4] = (5.0 / 120.0) * s_all[i] + (-20.0 / 120.0) * s_all[i + 1] + (15.0 / 60.0) * s_all[i + 2] + (-10.0 / 60.0) * s_all[i + 3] + (5.0 / 120.0) * s_all[i + 4];
            s_coefficients_[i][5] = (-1.0 / 120.0) * s_all[i] + (5.0 / 120.0) * s_all[i + 1] + (-5.0 / 60.0) * s_all[i + 2] + (5.0 / 60.0) * s_all[i + 3] + (-5.0 / 120.0) * s_all[i + 4] + (1.0 / 120.0) * s_all[i + 5];

            // Calculate d coefficients
            d_coefficients_[i].resize(6);
            d_coefficients_[i][0] = (1.0 / 120.0) * d_all[i] + (26.0 / 120.0) * d_all[i + 1] + (33.0 / 60.0) * d_all[i + 2] + (13.0 / 60.0) * d_all[i + 3] + (1.0 / 120.0) * d_all[i + 4];
            d_coefficients_[i][1] = (-5.0 / 120.0) * d_all[i] + (-50.0 / 120.0) * d_all[i + 1] + (25.0 / 60.0) * d_all[i + 3] + (5.0 / 120.0) * d_all[i + 4];
            d_coefficients_[i][2] = (10.0 / 120.0) * d_all[i] + (20.0 / 120.0) * d_all[i + 1] + (-30.0 / 60.0) * d_all[i + 2] + (10.0 / 60.0) * d_all[i + 3] + (10.0 / 120.0) * d_all[i + 4];
            d_coefficients_[i][3] = (-10.0 / 120.0) * d_all[i] + (20.0 / 120.0) * d_all[i + 1] + (-10.0 / 60.0) * d_all[i + 3] + (10.0 / 120.0) * d_all[i + 4];
            d_coefficients_[i][4] = (5.0 / 120.0) * d_all[i] + (-20.0 / 120.0) * d_all[i + 1] + (15.0 / 60.0) * d_all[i + 2] + (-10.0 / 60.0) * d_all[i + 3] + (5.0 / 120.0) * d_all[i + 4];
            d_coefficients_[i][5] = (-1.0 / 120.0) * d_all[i] + (5.0 / 120.0) * d_all[i + 1] + (-5.0 / 60.0) * d_all[i + 2] + (5.0 / 60.0) * d_all[i + 3] + (-5.0 / 120.0) * d_all[i + 4] + (1.0 / 120.0) * d_all[i + 5];

            // Calculte t coefficients
            t_coefficients_[i].resize(6);
            t_coefficients_[i][0] = (1.0 / 120.0) * t_all[i] + (26.0 / 120.0) * t_all[i + 1] + (33.0 / 60.0) * t_all[i + 2] + (13.0 / 60.0) * t_all[i + 3] + (1.0 / 120.0) * t_all[i + 4];
            t_coefficients_[i][1] = (-5.0 / 120.0) * t_all[i] + (-50.0 / 120.0) * t_all[i + 1] + (25.0 / 60.0) * t_all[i + 3] + (5.0 / 120.0) * t_all[i + 4];
            t_coefficients_[i][2] = (10.0 / 120.0) * t_all[i] + (20.0 / 120.0) * t_all[i + 1] + (-30.0 / 60.0) * t_all[i + 2] + (10.0 / 60.0) * t_all[i + 3] + (10.0 / 120.0) * t_all[i + 4];
            t_coefficients_[i][3] = (-10.0 / 120.0) * t_all[i] + (20.0 / 120.0) * t_all[i + 1] + (-10.0 / 60.0) * t_all[i + 3] + (10.0 / 120.0) * t_all[i + 4];
            t_coefficients_[i][4] = (5.0 / 120.0) * t_all[i] + (-20.0 / 120.0) * t_all[i + 1] + (15.0 / 60.0) * t_all[i + 2] + (-10.0 / 60.0) * t_all[i + 3] + (5.0 / 120.0) * t_all[i + 4];
            t_coefficients_[i][5] = (-1.0 / 120.0) * t_all[i] + (5.0 / 120.0) * t_all[i + 1] + (-5.0 / 60.0) * t_all[i + 2] + (5.0 / 60.0) * t_all[i + 3] + (-5.0 / 120.0) * t_all[i + 4] + (1.0 / 120.0) * t_all[i + 5];
        }

    }
    ~QuinticBSplineTrajectory() = default;

    // Generate scatter point
    Point3f generateTrajPoint(double u) const {
        double s = sValue(u);
        double d = dValue(u);
        double t = tValue(u);
        Point3f cur_traj_point = Point3f(s, d, t);
    }

    /**
     * @brief Generate the whole trajectory
     * @param sample_gap the gap in coefficient sampling, not the gap in world frame or frenet frame
     * @return the final trajectory
     */    
    std::vector<Point3f> generateTraj(double sample_gap) {
        int sample_num = static_cast<int>(static_cast<double>(segment_num_) / sample_gap);
        std::vector<double> samples = Tools::linspace(0.0, static_cast<double>(segment_num_), sample_num);
        std::vector<Point3f> trajectory;

        for (const auto sample : samples) {
            trajectory.emplace_back(generateTrajPoint(sample));
        }

        return trajectory;
    }



 private:
    // Verify the input
    double inputVerify(double u) const  {
        if (Tools::isSmall(u, 0.0)) {
            return 0.0;
        } else if (Tools::isLarge(u, static_cast<double>(segment_num_))){
            return static_cast<double>(segment_num_);
        } else {
            return u;
        }
    }

    // Generate segment information, i.e., the index of segment, the parameter in this specified segment
    std::pair<int, double> getSegmentInfo(double u) const  {
        u = this->inputVerify(u);
        for (int i = 0; i < this->segment_num_; i++) {
            if (Tools::isSmall(u, static_cast<double>(i + 1))) {
                double remain = u - i;
                std::pair<int, double> result{i, remain};
                return result;
            }
        }
        std::pair<int, double> result{segment_num_ - 1, 1.0};
        return result;
    }

    // Calculate s
    double sValue(double u) const  {
        u = inputVerify(u);
        // 得到分段和参数
        auto segment_info = getSegmentInfo(u);
        u = segment_info.second;
        int index = segment_info.first;
        return s_coefficients_[index][0] + s_coefficients_[index][1] * u + s_coefficients_[index][2] * u * u + s_coefficients_[index][3] * u * u * u + s_coefficients_[index][4] * u * u * u * u + s_coefficients_[index][5] * u * u * u * u * u;
    }
    
    // Calculate d
    double dValue(double u) const  {
        u = inputVerify(u);
        // 得到分段和参数
        auto segment_info = getSegmentInfo(u);
        u = segment_info.second;
        int index = segment_info.first;
        return d_coefficients_[index][0] + d_coefficients_[index][1] * u + d_coefficients_[index][2] * u * u + d_coefficients_[index][3] * u * u * u + d_coefficients_[index][4] * u * u * u * u + d_coefficients_[index][5] * u * u * u * u * u;
    }

    // Calculate t
    double tValue(double u) const  {
        u = inputVerify(u);
        // 得到分段和参数
        auto segment_info = getSegmentInfo(u);
        u = segment_info.second;
        int index = segment_info.first;
        return t_coefficients_[index][0] + t_coefficients_[index][1] * u + t_coefficients_[index][2] * u * u + t_coefficients_[index][3] * u * u * u + t_coefficients_[index][4] * u * u * u * u + t_coefficients_[index][5] * u * u * u * u * u;
    }


    int segment_num_;
    std::vector<std::vector<double>> s_coefficients_;
    std::vector<std::vector<double>> d_coefficients_;
    std::vector<std::vector<double>> t_coefficients_;

};

} // End of namespace Common






