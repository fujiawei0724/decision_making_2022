/*
 * @Author: fujiawei0724
 * @Date: 2021-11-12 20:14:57
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-11-12 20:40:45
 * @Descripttion: Trajectory planner's interface with the whole pipeline
 */

#include "Common.hpp"

void DecisionMaking::SubVehicle::trajectoryPlanning(bool* trajectory_planning_result) {
    // Transform ego vehicle information and surround vehice information
    // Update vehicle information
    PathPlanningUtilities::VehicleState start_point_in_world;
    this->current_vehicle_world_position_mutex_.lock();
    start_point_in_world = this->current_vehicle_world_position_;
    this->current_vehicle_world_position_mutex_.unlock();
    PathPlanningUtilities::VehicleMovementState start_point_movement;
    this->current_vehicle_movement_mutex_.lock();
    start_point_movement = this->current_vehicle_movement_;
    this->current_vehicle_movement_mutex_.unlock();
    this->current_vehicle_kappa_mutex_.lock();
    double start_point_kappa = this->current_vehicle_kappa_;
    start_point_in_world.kappa_ = start_point_kappa;
    this->current_vehicle_kappa_mutex_.unlock();
    current_vehicle_steer_metex_.lock();
    double current_vehicle_steer = current_vehicle_steer_;
    current_vehicle_steer_metex_.unlock();

    // Update data
    Eigen::Matrix<double, 2, 1> ego_veh_position{start_point_in_world.position_.x_, start_point_in_world.position_.y_};
    Common::Vehicle ego_vehicle = BehaviorPlanner::VehicleInterface::getEgoVehicle(ego_veh_position, start_point_in_world.theta_, start_point_kappa, start_point_movement.velocity_, start_point_movement.acceleration_, current_vehicle_steer, vehicle_length_, vehicle_width_);

    // Trajectory planning
    std::vector<Common::Point3f> trajectory;
    bool result = false;
    TrajectoryPlanner::TrajectoryPlanningCore* traj_planning_core = new TrajectoryPlanner::TrajectoryPlanningCore();
    traj_planning_core->load(ego_vehicle, reference_lane_, ego_trajectory_, surround_trajectories_, unlaned_obstacles_);
    traj_planning_core->runOnce(&result, &trajectory);
    if (!result) {
        printf("[MainPipeline] trajectory planning failed.\n");
    }

    *trajectory_planning_result = result;


}
