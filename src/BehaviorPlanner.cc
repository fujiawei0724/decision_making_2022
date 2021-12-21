/*
 * @Author: fujiawei0724
 * @Date: 2021-10-27 11:30:42
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-12-21 10:42:57
 * @Descripttion: behavior planner interface with the whole pipeline.
 */

#include "Common.hpp"

void DecisionMaking::SubVehicle::behaviorPlanning(bool* result) {
    // Update information for behavior planning
    updateMapInformation();
    updateValidateTrafficRuleInformation();
    updateObstacleInformation();

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

    // Shield the lane which is occupied by static obstacles
    if (left_lane_.isLaneOccupiedByStaticObs(ego_vehicle.state_.position_, obstacles_, traffic_rule_obstacles_)) {
        left_lane_.lane_existance_ = false;
    }
    if (right_lane_.isLaneOccupiedByStaticObs(ego_vehicle.state_.position_, obstacles_, traffic_rule_obstacles_)) {
        right_lane_.lane_existance_ = false;
    }
    if (center_lane_.isLaneOccupiedByStaticObs(ego_vehicle.state_.position_, obstacles_, traffic_rule_obstacles_)) {
        if (left_lane_.lane_existance_ || right_lane_.lane_existance_) {
            center_lane_.lane_existance_ = false;
        }
    }

    // Contruct map interface for behavior planner
    std::map<Common::LaneId, bool> lanes_exist_info{{Common::LaneId::CenterLane, false}, {Common::LaneId::LeftLane, false}, {Common::LaneId::RightLane, false}};
    std::map<Common::LaneId, Lane> lanes_info;
    if (center_lane_.getLaneExistance()) {
        lanes_exist_info[Common::LaneId::CenterLane] = true;
        lanes_info[Common::LaneId::CenterLane] = center_lane_;
    }
    if (left_lane_.getLaneExistance()) {
        lanes_exist_info[Common::LaneId::LeftLane] = true;
        lanes_info[Common::LaneId::LeftLane] = left_lane_;
    }
    if (right_lane_.getLaneExistance()) {
        lanes_exist_info[Common::LaneId::RightLane] = true;
        lanes_info[Common::LaneId::RightLane] = right_lane_;
    }
    BehaviorPlanner::MapInterface map_interface = BehaviorPlanner::MapInterface(lanes_exist_info, lanes_info);

    // Clear information
    unlaned_obstacles_.clear();
    ego_trajectory_.clear();
    surround_trajectories_.clear();

    // Unlaned obstacles are considered in trajectory planner to generate occupied semantic cubes
    std::vector<DecisionMaking::Obstacle> unlaned_obstacles;
    std::unordered_map<int, Common::Vehicle> surround_vehicles = BehaviorPlanner::VehicleInterface::getSurroundVehicles(&map_interface, obstacles_, unlaned_obstacles);
    unlaned_obstacles_ = unlaned_obstacles;

    // Construct behavior planner core and decision making
    double behavior_planner_time_span = 4.0;
    double behavior_planner_dt = 0.4;
    bool is_behavior_planning_success = false;
    clock_t behavior_planning_start_time = clock();
    BehaviorPlanner::BehaviorPlannerCore* behavior_planner = new BehaviorPlanner::BehaviorPlannerCore(&map_interface, behavior_planner_time_span, behavior_planner_dt, vis_behavior_planner_ego_states_pub_);
    is_behavior_planning_success = behavior_planner->runBehaviorPlanner(ego_vehicle, surround_vehicles, &ego_trajectory_, &surround_trajectories_, &reference_lane_);
    clock_t behavior_planning_end_time = clock();
    printf("[MainPipeline] behavior planning time consumption: %lf.\n", static_cast<double>((behavior_planning_end_time - behavior_planning_start_time)) / CLOCKS_PER_SEC);

    // // Visualization best policy states predicted by behavior planning
    // if (is_behavior_planning_success) {
    //     VisualizationMethods::visualizeTrajectory(ego_trajectory_, vis_behavior_planner_ego_states_pub_);
    // } else {
    //     printf("[MainPipeline] Behavior planning failed.\n");
    // }

    *result = is_behavior_planning_success;

}

void DecisionMaking::SubVehicle::hpdmPlanning(bool* result) {
    // Update information for behavior planning
    updateMapInformation();
    updateObstacleInformation();

    // Contruct map interface for HPDM 
    std::map<Common::LaneId, bool> lanes_exist_info{{Common::LaneId::CenterLane, false}, {Common::LaneId::LeftLane, false}, {Common::LaneId::RightLane, false}};
    std::map<Common::LaneId, Lane> lanes_info;
    if (center_lane_.getLaneExistance()) {
        lanes_exist_info[Common::LaneId::CenterLane] = true;
        lanes_info[Common::LaneId::CenterLane] = center_lane_;
    }
    if (left_lane_.getLaneExistance()) {
        lanes_exist_info[Common::LaneId::LeftLane] = true;
        lanes_info[Common::LaneId::LeftLane] = left_lane_;
    }
    if (right_lane_.getLaneExistance()) {
        lanes_exist_info[Common::LaneId::RightLane] = true;
        lanes_info[Common::LaneId::RightLane] = right_lane_;
    }
    BehaviorPlanner::MapInterface map_interface = BehaviorPlanner::MapInterface(lanes_exist_info, lanes_info);
    // Get lane information
    std::vector<double> lane_info = map_interface.getLaneInfo();

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
    Lane nearest_lane = map_interface.calculateNearestLane(ego_vehicle);

    // Clear information
    unlaned_obstacles_.clear();
    ego_trajectory_.clear();
    surround_trajectories_.clear();

    // Unlaned obstacles are considered in trajectory planner to generate occupied semantic cubes
    std::vector<DecisionMaking::Obstacle> unlaned_obstacles;
    std::unordered_map<int, Common::Vehicle> surround_vehicles = BehaviorPlanner::VehicleInterface::getSurroundVehicles(&map_interface, obstacles_, unlaned_obstacles);
    unlaned_obstacles_ = unlaned_obstacles;

    // Run HPDM
    // Load information
    std::string model_path = "/home/fjw/PioneerTest/catkin_ws/src/planning/motion_planning/model/model0.pt";
    clock_t hpdm_planning_start_time = clock();
    HpdmPlanner::HpdmPlannerCore* hpdm_planner = new HpdmPlanner::HpdmPlannerCore(&map_interface, nearest_lane, model_path, vis_behavior_planner_ego_states_pub_);
    hpdm_planner->load(ego_vehicle, surround_vehicles, lane_info);
    // Get additional information
    bool is_safe = false;
    double cost = 0.0;
    hpdm_planner->runHpdmPlanner(3, &ego_trajectory_, &surround_trajectories_, &reference_lane_, &is_safe, &cost);
    clock_t hpdm_planning_end_time = clock();
    printf("[MainPipeline] hpdm planning time consumption: %lf.\n", static_cast<double>((hpdm_planning_end_time - hpdm_planning_start_time)) / CLOCKS_PER_SEC);

    *result = is_safe;
}
