/*
 * @Author: fujiawei0724
 * @Date: 2021-12-01 21:10:42
 * @LastEditTime: 2021-12-01 21:24:08
 * @LastEditors: Please set LastEditors
 * @Description: Components for behavior planning.
 */


#include "Common.hpp"

// TODO: split class definition here
namespace BehaviorPlanner {

    constexpr double BehaviorPlannerConfig::look_ahead_min_distance;
    constexpr double BehaviorPlannerConfig::look_ahead_max_distance;
    constexpr double BehaviorPlannerConfig::steer_control_gain;
    constexpr double BehaviorPlannerConfig::wheelbase_length;

    constexpr double BehaviorPlannerConfig::max_lon_acc_jerk;
    constexpr double BehaviorPlannerConfig::max_lon_brake_jerk;
    constexpr double BehaviorPlannerConfig::max_lat_acceleration_abs;
    constexpr double BehaviorPlannerConfig::max_lat_jerk_abs;
    constexpr double BehaviorPlannerConfig::max_steer_angle_abs;
    constexpr double BehaviorPlannerConfig::max_steer_rate;
    constexpr double BehaviorPlannerConfig::max_curvature_abs;

    constexpr double IDM::vehicle_length_;
    constexpr double IDM::minimum_spacing_;
    constexpr double IDM::desired_headaway_time_;
    constexpr double IDM::acceleration_;
    constexpr double IDM::comfortable_braking_deceleration_;
    constexpr double IDM::hard_braking_deceleration_;
    constexpr double IDM::exponent_;

} // End of namespace BehaviorPlanner
