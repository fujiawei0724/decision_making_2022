/*
 * @Author: fujiawei0724
 * @Date: 2021-12-14 17:44:31
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-12-15 10:39:12
 * @Description: Utils include trigger, checker
 */

#pragma once

#include "VehicleState.hpp"

namespace Utils {

using namespace Common;

class Trigger {
 public:
    Trigger() = default;
    ~Trigger() = default;

    /**
     * @brief update data
     * @param {*}
     */
    void load(const std::vector<Point3f>& executed_trajectory, const clock_t& last_update_time_stamp) {
        traj_ = executed_trajectory;
        update_time_stamp_ = last_update_time_stamp;
    }

    /**
     * @brief judge whether need replanning
     * @param {*}
     * @return {*}
     */    
    bool runOnce() {
        bool replanning_due_to_remain_time = false;
        checkTrajRemainTime(&replanning_due_to_remain_time);
        
        // TODO: add collision avoidance check here, need add vehicles information to do this
        bool replanning_due_to_collision = false;

        return replanning_due_to_remain_time || replanning_due_to_collision;
    }

    /**
     * @brief judge whether replanning from remain time
     * @param {*}
     * @return {*}
     */    
    void checkTrajRemainTime(bool* need_replanning) {
        clock_t current_time_stamp = clock();
        if (static_cast<double>((current_time_stamp - update_time_stamp_)) / CLOCKS_PER_SEC > 3.8) {
            *need_replanning = true;
        } else {
            *need_replanning = false;
        }
    }

    /**
     * @brief judge whether replanning from collision 
     * @param {*}
     * @return {*}
     */ 
    void checkTrajCollision(bool* need_replanning) {

    }   
    

    std::vector<Point3f> traj_;
    clock_t update_time_stamp_;

};


class TrajectoryChecker {
 public:
    /**
     * @brief calculate trajectory detailed information
     * @param {*}
     * @return {*}
     */    
    static void checkTraj(const std::vector<Point3f>& trajectory, std::vector<double>* thetas, std::vector<double>* curvatures, std::vector<double>* velocities, std::vector<double>* accelerations) {
        // Initialize
        int traj_length = static_cast<int>(trajectory.size());
        std::vector<double> tmp_thetas, tmp_curvatures, tmp_velocities, tmp_accelerations;
        tmp_thetas.resize(traj_length);
        tmp_curvatures.resize(traj_length);
        tmp_velocities.resize(traj_length);
        tmp_accelerations.resize(traj_length);

        // Traverse all points
        for (int i = 0; i < traj_length; i++) {
            // Calculate theta and velocity
            if (i != traj_length - 1) {
                tmp_thetas[i] = atan2(trajectory[i + 1].y_ - trajectory[i].y_, trajectory[i + 1].x_ - trajectory[i].x_);
                tmp_velocities[i] = sqrt((trajectory[i + 1].y_ - trajectory[i].y_) * (trajectory[i + 1].y_ - trajectory[i].y_) + (trajectory[i + 1].x_ - trajectory[i].x_) * (trajectory[i + 1].x_ - trajectory[i].x_)) / (trajectory[i + 1].z_ - trajectory[i].z_);
            } else if (i == traj_length - 1) {
                tmp_thetas[i] = tmp_thetas[i - 1];
                tmp_velocities[i] = tmp_velocities[i - 1];
            } else {
                assert(false);
            }
        }

        // Traverse all points
        for (int i = 0; i < traj_length; i++) {
            // Calculate theta and velocity
            if (i != traj_length - 1) {
                tmp_curvatures[i] = (tmp_thetas[i + 1] - tmp_thetas[i]) / sqrt((trajectory[i + 1].y_ - trajectory[i].y_) * (trajectory[i + 1].y_ - trajectory[i].y_) + (trajectory[i + 1].x_ - trajectory[i].x_) * (trajectory[i + 1].x_ - trajectory[i].x_));
                tmp_accelerations[i] = (tmp_velocities[i + 1] - tmp_velocities[i]) / (trajectory[i + 1].z_ - trajectory[i].z_);
            } else if (i == traj_length - 1) {
                tmp_curvatures[i] = tmp_curvatures[i - 1];
                tmp_accelerations[i] = tmp_accelerations[i - 1];
            } else {
                assert(false);
            }
             
        }

        // Cache
        *thetas = tmp_thetas;
        *curvatures = tmp_curvatures;
        *velocities = tmp_velocities;
        *accelerations = tmp_accelerations;

    }
};


} //End of Utils namespace