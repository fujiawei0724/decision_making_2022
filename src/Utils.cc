/*
 * @Author: fujiawei0724
 * @Date: 2021-12-15 10:40:30
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-12-15 11:41:53
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