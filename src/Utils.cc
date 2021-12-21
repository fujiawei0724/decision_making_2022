/*
 * @Author: fujiawei0724
 * @Date: 2021-12-15 10:40:30
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-12-21 19:20:49
 * @Description: Utils for trajectory planning.
 */

#include "Common.hpp"

void DecisionMaking::SubVehicle::trajectoryPublish(const std::vector<double>& thetas, const std::vector<double>& curvatures, const std::vector<double>& velocities, const std::vector<double>& accelerations, const ros::Publisher &publisher) {
    // Initialize
    path_planning_msgs::MotionPlanningCurve final_curve;
    final_curve.header.frame_id = "world";
    final_curve.header.stamp = ros::Time::now();

    // Calculate speed limitiation
    double max_velocity = *std::max_element(velocities.begin(), velocities.end()) + 1.0;
    double min_velocity = std::max(*std::min_element(velocities.begin(), velocities.end()) - 1.0, 0.0);

    // Supple trajectory
    final_curve.points.resize(generated_trajectory_.size());
    for (int i = 0; i < static_cast<int>(generated_trajectory_.size()); i++) {
        path_planning_msgs::CurvePoint point;
        point.x = generated_trajectory_[i].x_;
        point.y = generated_trajectory_[i].y_;
        point.theta = thetas[i];
        point.kappa = curvatures[i];
        point.velocity = velocities[i];
        final_curve.points[i] = point;
    }
    // TODO: test this logic, this value probably no use
    final_curve.point_margin = 0.0;    
    final_curve.mode = path_planning_msgs::MotionPlanningCurve::MAINTAIN_VELOCITY;
    // TODO: test this logic, this value probably no use
    final_curve.vehicle_position_index = 0;
    final_curve.reverse_allowing = false;
    final_curve.tracking_mode = path_planning_msgs::MotionPlanningCurve::CENTER;
    // TODO: test this logic, this value probably no use
    final_curve.expected_acceleration = 0.0;
    final_curve.velocity_limitation_max = max_velocity;
    final_curve.velocity_limitation_min = min_velocity;
    
    // Publish 
    publisher.publish(final_curve);
    executed_trajectory_ = generated_trajectory_;
    trajectory_update_time_stamp_ = clock();

}

void DecisionMaking::SubVehicle::trajectoryCheck(std::vector<double>* thetas, std::vector<double>* curvatures, std::vector<double>* velocities, std::vector<double>* accelerations) const {
    
    Utils::TrajectoryChecker::checkTraj(generated_trajectory_, thetas, curvatures, velocities, accelerations);

    // TODO: add detailed check logic here
    
}

Utils::Trigger::Trigger() = default;
Utils::Trigger::~Trigger() = default;

/**
 * @brief update data
 * @param {*}
 */
void Utils::Trigger::load(const std::vector<Point3f>& executed_trajectory, const clock_t& last_update_time_stamp, const PathPlanningUtilities::Point2f& ego_vehicle_position) {
    cur_pos_ = ego_vehicle_position;
    traj_ = executed_trajectory;
    update_time_stamp_ = last_update_time_stamp;
}

/**
 * @brief judge whether need replanning
 * @param {*}
 * @return {*}
 */    
bool Utils::Trigger::runOnce() {
    bool replanning_due_to_remain_time = false;
    checkTrajRemainTime(&replanning_due_to_remain_time);

    bool replanning_due_to_remain_dis = false;
    checkTrajRemainDis(&replanning_due_to_remain_dis);
    
    // TODO: add collision avoidance check here, need add vehicles information to do this
    bool replanning_due_to_collision = false;

    return replanning_due_to_remain_time || replanning_due_to_remain_dis || replanning_due_to_collision;
}

/**
 * @brief judge whether replanning from remain time
 * @param {*}
 * @return {*}
 */    
void Utils::Trigger::checkTrajRemainTime(bool* need_replanning) {
    clock_t current_time_stamp = clock();
    if (static_cast<double>((current_time_stamp - update_time_stamp_)) / CLOCKS_PER_SEC > 3.8) {
        *need_replanning = true;
    } else {
        *need_replanning = false;
    }
}

/**
 * @brief judge whether replanning from remain distance
 * @param {*}
 * @return {*}
 */ 
void Utils::Trigger::checkTrajRemainDis(bool* need_replanning) {

    std::function <double (double, double)> dis = [&](const double x, const double y) {return sqrt((x - cur_pos_.x_) * (x - cur_pos_.x_) + (y - cur_pos_.y_) * (y - cur_pos_.y_));};
    
    // Binary search the nearest index in the trajectory from the ego vehicle position
    // TODO: check this logic
    int left = 0;
    int right = static_cast<int>(traj_.size()) - 1;
    while (left < right) {
        int mid = left + (right - left) / 2;
        if (mid == 0 || mid == static_cast<int>(traj_.size()) - 1) {
            break;
        }
        if (dis(traj_[mid].x_, traj_[mid].y_) >= dis(traj_[mid - 1].x_, traj_[mid - 1].y_)) {
            right = mid;
        } else {
            left = mid + 1;
        }
    }
    if (static_cast<double>(left) / static_cast<double>(traj_.size()) > 0.85) {
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
void Utils::Trigger::checkTrajCollision(bool* need_replanning) {

} 

/**
 * @brief calculate trajectory detailed information
 * @param {*}
 * @return {*}
 */    
void Utils::TrajectoryChecker::checkTraj(const std::vector<Point3f>& trajectory, std::vector<double>* thetas, std::vector<double>* curvatures, std::vector<double>* velocities, std::vector<double>* accelerations) {
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
