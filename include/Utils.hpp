/*
 * @Author: fujiawei0724
 * @Date: 2021-12-14 17:44:31
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-12-21 18:28:35
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
    void load(const std::vector<Point3f>& executed_trajectory, const clock_t& last_update_time_stamp, const PathPlanningUtilities::Point2f& ego_vehicle_position);

    /**
     * @brief judge whether need replanning
     * @param {*}
     * @return {*}
     */    
    bool runOnce();

    /**
     * @brief judge whether replanning from remain time
     * @param {*}
     * @return {*}
     */    
    void checkTrajRemainTime(bool* need_replanning);

    /**
     * @brief judge whether replanning from remain distance
     * @param {*}
     * @return {*}
     */ 
    void checkTrajRemainDis(bool* need_replanning);   

    /**
     * @brief judge whether replanning from collision 
     * @param {*}
     * @return {*}
     */ 
    void checkTrajCollision(bool* need_replanning);
    
    PathPlanningUtilities::Point2f cur_pos_;
    std::vector<Point3f> traj_;
    clock_t update_time_stamp_;

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