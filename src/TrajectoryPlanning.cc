/*
 * @Author: fujiawei0724
 * @Date: 2021-11-12 20:14:57
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-08-05 15:06:29
 * @Descripttion: Trajectory planner's interface with the main pipeline
 */

#include "Common.hpp"

/**
 * @brief Trajectory planning with quintic B-spline
 * TODO: need test and DEBUG
 */
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

/**
 * @brief Ssc planning
 */
void DecisionMaking::SubVehicle::sscPlanning(bool* trajectory_planning_result) {
    
    PathPlanningUtilities::VehicleState start_point_in_world;
    PathPlanningUtilities::VehicleMovementState start_point_movement;
    double start_point_kappa = 0.0;
    double current_vehicle_steer = 0.0;

    // Judge replanning start point
    if (replanning_from_previous_trajectory_state_) {
        // supply data from previous trajectory
        start_point_in_world = prev_traj_corres_veh_state_;
        start_point_movement = prev_traj_corres_veh_movement_state_;
        start_point_kappa = start_point_in_world.kappa_;
        current_vehicle_steer = atan(2.8498 * start_point_kappa);

        // // DEBUG
        // printf("DEBUG replanning from previous trajectory points.\n");
        // printf("DEBUG replanning start velocity: %lf, acceleration: %lf.\n", start_point_movement.velocity_, start_point_movement.acceleration_);
        // // END DEBUG

    } else {
        // supply data from world information
        this->current_vehicle_world_position_mutex_.lock();
        start_point_in_world = this->current_vehicle_world_position_;
        this->current_vehicle_world_position_mutex_.unlock();
        this->current_vehicle_movement_mutex_.lock();
        start_point_movement = this->current_vehicle_movement_;
        this->current_vehicle_movement_mutex_.unlock();
        this->current_vehicle_kappa_mutex_.lock();
        start_point_kappa = this->current_vehicle_kappa_;
        start_point_in_world.kappa_ = start_point_kappa;
        this->current_vehicle_kappa_mutex_.unlock();
        current_vehicle_steer_metex_.lock();
        current_vehicle_steer = current_vehicle_steer_;
        current_vehicle_steer_metex_.unlock();
    }


    // start_point_kappa = 0.0;
    // current_vehicle_steer = 0.0;

    // // DEBUG
    // std::cout << start_point_kappa << std::endl;
    // std::cout << current_vehicle_steer << std::endl;
    // // END DEBUG

    


    // Update data
    Eigen::Matrix<double, 2, 1> ego_veh_position{start_point_in_world.position_.x_, start_point_in_world.position_.y_};
    Common::Vehicle ego_vehicle = BehaviorPlanner::VehicleInterface::getEgoVehicle(ego_veh_position, start_point_in_world.theta_, start_point_kappa, start_point_movement.velocity_, start_point_movement.acceleration_, current_vehicle_steer, vehicle_length_, vehicle_width_);

    // Trajectory planning
    std::vector<Common::Point3f> trajectory;
    bool result = false;
    if (!reference_lane_.is_existence_) {
        *trajectory_planning_result = false;
        printf("[SscPlanner] unknown reference lane information.\n");
        return;
    }
    SscPlanner::SscTrajectoryPlanningCore* ssc_planning_core = new SscPlanner::SscTrajectoryPlanningCore();
    ssc_planning_core->load(ego_vehicle, reference_lane_, ego_trajectory_, surround_trajectories_, unlaned_obstacles_);
    ssc_planning_core->runOnce(&result, &trajectory);

    // // Visualization
    // VisualizationMethods::visualizeTrajectory(trajectory, vis_trajectory_planner_pub_);

    // // DEBUG
    // for (int i = 0; i < static_cast<int>(trajectory.size()); i++) {
    //     printf("Index: %d, x: %lf, y: %lf, t: %lf.\n", i, trajectory[i].x_, trajectory[i].y_, trajectory[i].z_);
    // }
    // // END DEBUG
    
    delete ssc_planning_core;

    if (result) {
        // Smooth path
        PathPlanningUtilities::Path raw_path;
        PathPlanningUtilities::Path smoothed_path;
        for (int i = 0; i < trajectory.size(); i++) {
            PathPlanningUtilities::Point2f cur_point;
            cur_point.x_ = trajectory[i].x_;
            cur_point.y_ = trajectory[i].y_;
            raw_path.push_back(cur_point);
        }
        double ave_dis = sqrt(pow((trajectory.back().x_ - trajectory.front().x_), 2) + pow((trajectory.back().y_ - trajectory.front().y_), 2)) / static_cast<int>(trajectory.size());
        PathPlanningUtilities::PathUtilities::pathSmooth(raw_path, smoothed_path, ave_dis);
        
        std::vector<Common::Point3f> smoothed_traj;
        smoothed_traj.emplace_back(trajectory[0]);
        for (int i = 1; i < smoothed_path.size(); i++) {
            double distance = sqrt(pow(smoothed_path[i].x_ - smoothed_traj.back().x_, 2) + pow(smoothed_path[i].y_ - smoothed_traj.back().y_, 2));


            Common::Point3f cur_traj_point = Common::Point3f(smoothed_path[i].x_, smoothed_path[i].y_, trajectory[i].z_);
            smoothed_traj.emplace_back(cur_traj_point);

        }
        generated_trajectory_ = smoothed_traj;
        // generated_trajectory_ = trajectory;
    }
    *trajectory_planning_result = result;

}
