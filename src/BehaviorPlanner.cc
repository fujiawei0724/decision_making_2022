/*
 * @Author: fujiawei0724
 * @Date: 2021-10-27 11:30:42
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-11-03 20:24:09
 * @Descripttion: EUDM behavior planner interface with the whole pipeline
 */

#include "Common.hpp"

bool DecisionMaking::SubVehicle::behaviorPlanning() {
    // Update information for behavior planning
    updateMapInformation();
    updateObstacleInformation();

    // Contruct map interface for behavior planner
    std::map<BehaviorPlanner::LaneId, bool> lanes_exist_info{{BehaviorPlanner::LaneId::CenterLane, false}, {BehaviorPlanner::LaneId::LeftLane, false}, {BehaviorPlanner::LaneId::RightLane, false}};
    std::map<BehaviorPlanner::LaneId, Lane> lanes_info;
    if (center_lane_.getLaneExistance()) {
        lanes_exist_info[BehaviorPlanner::LaneId::CenterLane] = true;
        lanes_info[BehaviorPlanner::LaneId::CenterLane] = center_lane_;
    }
    if (left_lane_.getLaneExistance()) {
        lanes_exist_info[BehaviorPlanner::LaneId::LeftLane] = true;
        lanes_info[BehaviorPlanner::LaneId::LeftLane] = left_lane_;
    }
    if (right_lane_.getLaneExistance()) {
        lanes_exist_info[BehaviorPlanner::LaneId::RightLane] = true;
        lanes_info[BehaviorPlanner::LaneId::RightLane] = right_lane_;
    }
    BehaviorPlanner::MapInterface map_interface = BehaviorPlanner::MapInterface(lanes_exist_info, lanes_info);

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
    BehaviorPlanner::Vehicle ego_vehicle = BehaviorPlanner::VehicleInterface::getEgoVehicle(ego_veh_position, start_point_in_world.theta_, start_point_kappa, start_point_movement.velocity_, start_point_movement.acceleration_, current_vehicle_steer, vehicle_length_, vehicle_width_);
    std::unordered_map<int, BehaviorPlanner::Vehicle> surround_vehicles = BehaviorPlanner::VehicleInterface::getSurroundVehicles(&map_interface, obstacles_);

    // Construct behavior planner core and decision making
    double behavior_planner_time_span = 4.0;
    double behavior_planner_dt = 0.4;
    bool is_behavior_planning_success = false;
    BehaviorPlanner::BehaviorPlannerCore behavior_planner = BehaviorPlanner::BehaviorPlannerCore(&map_interface, behavior_planner_time_span, behavior_planner_dt);
    is_behavior_planning_success = behavior_planner.runBehaviorPlanner(ego_vehicle, surround_vehicles, ego_trajectory_, surround_trajectories_);

    // DEBUG
    // Visualization best policy states predicted by behavior planning
    

    return is_behavior_planning_success;
}
