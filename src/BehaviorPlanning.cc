/*
 * @Author: fujiawei0724
 * @Date: 2021-12-01 21:10:42
 * @LastEditTime: 2021-12-03 17:41:13
 * @LastEditors: fujiawei0724
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

    // Add the variability of behavior sequence
    BehaviorPlannerCore::BehaviorPlannerCore(MapInterface* mtf, double predict_time_span, double dt) {
        mtf_ = mtf;
        predict_time_span_ = predict_time_span;
        dt_ = dt;
    }
    BehaviorPlannerCore::BehaviorPlannerCore(MapInterface* mtf, double predict_time_span, double dt, const ros::Publisher& vis_pub) {
        mtf_ = mtf;
        predict_time_span_ = predict_time_span;
        dt_ = dt;
        vis_pub_ = vis_pub;
    }
    BehaviorPlannerCore::~BehaviorPlannerCore() {
        
    }

    // Behavior planner runner
    bool BehaviorPlannerCore::runBehaviorPlanner(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles, Trajectory* ego_best_traj, std::unordered_map<int, Trajectory>* sur_best_trajs, Lane* target_behavior_reference_lane) {
        // Simulate all policies
        simulateAllBehaviors(ego_vehicle, surround_vehicles);

        // Select best policy
        int winner_index = -1;
        double winner_cost = MAX_VALUE;
        evaluatePolicies(winner_index, winner_cost);
        if (winner_index == -1) {
            // TODO: add logic to handle the situation where there is no safe policy
            return false;
        }
        
        // // DEBUG
        // // Print behavior cost information
        // for (int i = 0; i < static_cast<int>(behavior_sequence_cost_.size()); i++) {
        //     std::cout << "Behavior sequence: " << i << ", cost: " << behavior_sequence_cost_[i] << ", safe: " << behavior_safety_[i] << std::endl; 
        // }
        // // END DEBUG


        *ego_best_traj = ego_traj_[winner_index];
        *sur_best_trajs = sur_veh_trajs_[winner_index];
        *target_behavior_reference_lane = mtf_->lane_set_[final_reference_lane_id_[winner_index]];
        return true;

    }

    // Evaluate all policies
    void BehaviorPlannerCore::evaluatePolicies(int& winner_index, double& winner_cost) {
        int sequence_num = behavior_sequence_cost_.size();
        int win_idx = -1;
        double win_cost = MAX_VALUE;
        for (int i = 0; i < sequence_num; i++) {
            if (!behavior_safety_[i]) {
                continue;
            }
            if (behavior_sequence_cost_[i] < win_cost) {
                win_idx = i;
                win_cost = behavior_sequence_cost_[i];
            }
        }
        winner_index = win_idx;
        winner_cost = win_cost;
    }

    // Simulate all situation and store trajectories information
    void BehaviorPlannerCore::simulateAllBehaviors(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles) {
        // Generate all behavior sequences
        BehaviorGenerator* behavior_generator = new BehaviorGenerator(mtf_, static_cast<int>(predict_time_span_ / dt_));
        std::vector<BehaviorSequence> behavior_set = behavior_generator->generateBehaviorSequence();
        delete behavior_generator;
        int sequence_num = static_cast<int>(behavior_set.size());

        // // DEBUG
        // for (int i = 0; i < static_cast<int>(behavior_set.size()); i++) {
        //     std::cout << "Index: " << i << " ";
        //     std::cout << static_cast<std::underlying_type<LongitudinalBehavior>::type>(behavior_set[i][0].lon_beh_) << " ";
        //     for (int j = 0; j < static_cast<int>(behavior_set[i].size()); j++) {
        //         std::cout << static_cast<std::underlying_type<LateralBehavior>::type>(behavior_set[i][j].lat_beh_) << " ";
        //     }
        //     std::cout << std::endl;
        // }
        // // END DEBUG

        // Initialize container
        initializeContainer(sequence_num);

        // Multiple threads calculation
        // TODO: use thread pool to balance calculation consumption in difference thread
        std::vector<std::thread> thread_set(sequence_num);
        for (int i = 0; i < sequence_num; i++) {
            thread_set[i] = std::thread(&BehaviorPlannerCore::simulateSingleBehaviorSequence, this, ego_vehicle, surround_vehicles, behavior_set[i], i);
        }
        for (int i = 0; i < sequence_num; i++) {
            thread_set[i].join();
        }

        // // DEBUG
        // for (int i = 0; i < 1; i ++) {
        //     simulateSingleBehaviorSequence(ego_vehicle, surround_vehicles, behavior_set[41], 41);
        // }
        // // END DEBUG
    }

    // Simulate single behavior sequence
    void BehaviorPlannerCore::simulateSingleBehaviorSequence(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles, const BehaviorSequence& behavior_sequence, int index) {
        
        // Initialize semantic vehicles (include ego semantic vehicle and surround semantic vehicles)
        SemanticVehicle ego_semantic_vehicle = mtf_->getEgoSemanticVehicle(ego_vehicle, behavior_sequence[0]);
        std::unordered_map<int, SemanticVehicle> surround_semantic_vehicles = mtf_->getSurroundSemanticVehicles(surround_vehicles);

        // Determine desired speed based on longitudinal speed
        // TODO: add logic to calculate the desired speed
        double ego_vehicle_desired_speed = ego_vehicle.state_.velocity_;
        if (behavior_sequence[0].lon_beh_ == LongitudinalBehavior::Aggressive) {
            ego_vehicle_desired_speed += 5.0;
        } else if (behavior_sequence[0].lon_beh_ == LongitudinalBehavior::Conservative) {
            ego_vehicle_desired_speed = std::max(0.0, ego_vehicle_desired_speed - 5.0);
        } else {
            ego_vehicle_desired_speed = ego_vehicle.state_.velocity_;
        }

        // // DEBUG
        // std::cout << "Ego vehicle desired speed: " << ego_vehicle_desired_speed << std::endl;
        // // END DEBUG

        // Initialize trajectory
        Trajectory ego_trajectory;
        std::unordered_map<int, Trajectory> surround_trajectories;
        ego_trajectory.emplace_back(ego_vehicle);
        for (auto sur_veh : surround_vehicles) {
            surround_trajectories[sur_veh.first].emplace_back(sur_veh.second);
        }

        // State cache
        Vehicle current_ego_vehicle = ego_vehicle;
        std::unordered_map<int, Vehicle> current_surround_vehicles = surround_vehicles;
        // std::unordered_map<int, Vehicle> all_vehicles = surround_vehicles;
        // all_vehicles.insert({0, ego_vehicle});

        // Traverse behavior sequence to forward simulation
        for (int i = 0; i < static_cast<int>(behavior_sequence.size()); i++) {
            // Update the lateral behavior and reference lane for ego semantic vehicle
            if (i > 0 && behavior_sequence[i].lat_beh_ != behavior_sequence[i - 1].lat_beh_) {
                ego_semantic_vehicle = mtf_->getEgoSemanticVehicle(current_ego_vehicle, behavior_sequence[i]);
            }

            // Initialize desired state
            Vehicle ego_desired_state;
            std::unordered_map<int, Vehicle> surround_desired_states;

            // Simulate single behavior
            simulateSingleBehavior(ego_semantic_vehicle, surround_semantic_vehicles, ego_vehicle_desired_speed, ego_desired_state, surround_desired_states);

            // Update trajectories
            ego_trajectory.emplace_back(ego_desired_state);
            for (auto sur_desired_state_info: surround_desired_states) {
                surround_trajectories[sur_desired_state_info.first].emplace_back(sur_desired_state_info.second);
            }

            // Update state (the vehicle information in semantic vehicle)
            // Note that the reference lane information is not update in each round
            current_ego_vehicle = ego_desired_state;
            current_surround_vehicles = surround_desired_states;
            ego_semantic_vehicle.vehicle_ = current_ego_vehicle;
            for (auto& sur_semantic_vehicle_info: surround_semantic_vehicles) {
                sur_semantic_vehicle_info.second.vehicle_ = current_surround_vehicles[sur_semantic_vehicle_info.first];
            }
        }

        // Store trajectories information
        ego_traj_[index] = ego_trajectory;
        sur_veh_trajs_[index] = surround_trajectories;
        final_reference_lane_id_[index] = ego_semantic_vehicle.reference_lane_id_;

        // DEBUG
        // Visualization
        VisualizationMethods::visualizeTrajectory(ego_trajectory, vis_pub_, index);
        // Print the last predicted vehicle state
        ego_trajectory.back().print();
        // END DEBUG
        
        // Judge whether generate lane change behavior
        bool lane_change_flag{false};
        for (const VehicleBehavior& veh_beh: behavior_sequence) {
            if (veh_beh.lat_beh_ != LateralBehavior::LaneKeeping) {
                lane_change_flag = true;
                break;
            }
        }
        is_lane_changed_[index] = lane_change_flag;


        // Calculate target speed in nearest lane
        // Calculate ego vehicle's last predict state
        Vehicle ego_vehicle_last_desired_state = ego_trajectory.back();
        // Calculate speed max limit
        double speed_limit = mtf_->calculateSpeedLimit(ego_vehicle_last_desired_state);
        target_position_velocity_limit_[index] = speed_limit;

        // Calculate policy situation whether safe
        behavior_safety_[index] = PolicyEvaluater::calculateSafe(ego_trajectory, surround_trajectories, speed_limit);

        // Calculate policy situation cost
        behavior_sequence_cost_[index] = PolicyEvaluater::calculateCost(ego_trajectory, surround_trajectories, lane_change_flag, speed_limit);

    }

    // Simulate single vehicle behavior (lateral and longitudinal)
    void BehaviorPlannerCore::simulateSingleBehavior(const SemanticVehicle& ego_semantic_vehicle, const std::unordered_map<int, SemanticVehicle>& surround_semantic_vehicles, const double& ego_desired_velocity, Vehicle& ego_vehicle_next_state, std::unordered_map<int, Vehicle>& surround_vehicles_next_states) {
        // Get all semantic vehicles
        std::unordered_map<int, SemanticVehicle> all_semantic_vehicles = surround_semantic_vehicles;
        all_semantic_vehicles.insert({0, ego_semantic_vehicle});

        // Traverse all vehicles
        for (const auto& veh_info : all_semantic_vehicles) {
            // Calculate desired velocity
            double desired_velocity = veh_info.second.vehicle_.state_.velocity_;
            if (veh_info.first == 0) {
                // Ego vehicle
                desired_velocity = ego_desired_velocity;
            }

            // Calculate desired state
            Vehicle cur_vehicle_desired_state = ForwardExtender::extendState(mtf_, all_semantic_vehicles, veh_info.first, dt_, desired_velocity);

            // State cache
            if (veh_info.first == 0) {
                ego_vehicle_next_state = cur_vehicle_desired_state;
            } else {
                surround_vehicles_next_states[veh_info.first] = cur_vehicle_desired_state;
            }
        }

    }

    // Initialize container for multiple threads calculation
    // TODO: add detailed cost information for the information of each behavior sequence
    void BehaviorPlannerCore::initializeContainer(int length) {
        behavior_sequence_cost_.resize(length);
        behavior_safety_.resize(length);
        is_lane_changed_.resize(length);
        target_position_velocity_limit_.resize(length);
        ego_traj_.resize(length);
        sur_veh_trajs_.resize(length);
        final_reference_lane_id_.resize(length);
    }

} // End of namespace BehaviorPlanner
