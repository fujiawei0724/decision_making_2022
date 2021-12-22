/*
 * @Author: fujiawei0724
 * @Date: 2021-12-14 17:44:31
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-12-22 14:39:06
 * @Description: Utils include trigger, checker
 */

#pragma once

#include "VehicleState.hpp"

namespace Utils {

using namespace Common;

// Trigger for replanning
class Trigger {
 public:
    Trigger();
    ~Trigger();

    /**
     * @brief update data
     * @param {*}
     */
    void load(const std::vector<Point3f>& executed_trajectory, const std::vector<double>& thetas, const std::vector<double>& curvatures, const std::vector<double>& velocities, const std::vector<double>& accelerations, const PathPlanningUtilities::VehicleState& current_vehicle_state, const PathPlanningUtilities::VehicleMovementState& current_vehicle_movement_state);

    /**
     * @brief judge whether replanning from corresponding state in previous trajectory
     * @param {*}
     * @return {*}
     */    
    bool runOnce(PathPlanningUtilities::VehicleState* corres_ego_veh_state, PathPlanningUtilities::VehicleMovementState* corres_ego_veh_movement_state);

    /**
     * @brief find the nearest index from the current position to the executed trajectory
     * @return {*}
     */
    int findCorrespondingTrajIndex(); 


    PathPlanningUtilities::VehicleState current_vehicle_state_;
    PathPlanningUtilities::VehicleMovementState current_vehicle_movement_state_;
    std::vector<Point3f> traj_{};
    std::vector<double> traj_thetas_{};
    std::vector<double> traj_curvatures_{};
    std::vector<double> traj_velocities_{};
    std::vector<double> traj_accelerations_{};

};

// Publish trajectory
class TrajectoryChecker {
 public:
    /**
     * @brief calculate trajectory detailed information
     * @param {*}
     * @return {*}
     */    
    static void checkTraj(const std::vector<Point3f>& trajectory, std::vector<double>* thetas, std::vector<double>* curvatures, std::vector<double>* velocities, std::vector<double>* accelerations);
};


} //End of Utils namespace