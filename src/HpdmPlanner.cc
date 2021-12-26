/*
 * @Author: fujiawei0724
 * @Date: 2021-12-14 11:57:46
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-12-26 11:55:30
 * @Description: Hpdm planner.
 */

#include "Common.hpp"

namespace HpdmPlanner {

    /**
     * TODO: only support predicted time 4.0s with a time gap 0.4s, add more support parameters here
     * @brief transform an action index to a behavior sequence
     * @param action_index action index generate by model
     * @return behavior sequence
     */    
    std::vector<VehicleBehavior> ActionInterface::indexToBehSeq(const int& action_index) {
        // Calculate related information
        int lon_beh_val = action_index / 21;
        int lat_beh_val = action_index % 21;
        LongitudinalBehavior lon_beh = LongitudinalBehavior(lon_beh_val);
        LateralBehavior lat_beh;
        int change_begin_index = -1;
        if (lat_beh_val == 20) {
            // Without lane change
            lat_beh = LateralBehavior::LaneKeeping;
        } else {
            if (lat_beh_val % 2 == 0) {
                lat_beh = LateralBehavior::LaneChangeLeft;
            } else {
                lat_beh = LateralBehavior::LaneChangeRight;
            }
            change_begin_index = lat_beh_val / 2;
        }
        
        // Supple data 
        std::vector<VehicleBehavior> behavior_sequence;
        if (lat_beh == LateralBehavior::LaneKeeping) {
            for (int i = 0; i < 10; i++) {
                behavior_sequence.emplace_back(VehicleBehavior(lat_beh, lon_beh));
            }
        } else {
            for (int i = 0; i < 10; i++) {
                if (i < change_begin_index) {
                    behavior_sequence.emplace_back(VehicleBehavior(LateralBehavior::LaneKeeping, lon_beh));
                } else {
                    behavior_sequence.emplace_back(VehicleBehavior(lat_beh, lon_beh));
                }
            }
        }

        return behavior_sequence;

    }

    /**
     * @brief transform a series of action indices
     * @param {*}
     * @return {*}
     */   
    std::vector<std::vector<VehicleBehavior>> ActionInterface::indexVecToBehSeqVec(const std::vector<int>& action_indices) {
        int length = static_cast<int>(action_indices.size());
        std::vector<std::vector<VehicleBehavior>> behavior_sequence_vec(length);
        for (int i = 0; i < length; i++) {
            behavior_sequence_vec[i] = indexToBehSeq(action_indices[i]);
        }
        return behavior_sequence_vec;
    }

    /**
     * TODO: only support predicted time 4.0s with a time gap 0.4s, add more support parameters here
     * @brief transform an action index to a intention sequence
     * @param action_index action index generate by model
     * @return intention sequence
     */    
    std::vector<VehicleIntention> ActionInterface::indexToIntentionSeq(const int& action_index) {
        // Calculate related information
        int lon_beh_val = action_index / 21;
        int lat_beh_val = action_index % 21;
        double lon_vel_comp = static_cast<double>(lon_beh_val) - 5.0;
        LateralBehavior lat_beh;
        int change_begin_index = -1;
        if (lat_beh_val == 20) {
            // Without lane change
            lat_beh = LateralBehavior::LaneKeeping;
        } else {
            if (lat_beh_val % 2 == 0) {
                lat_beh = LateralBehavior::LaneChangeLeft;
            } else {
                lat_beh = LateralBehavior::LaneChangeRight;
            }
            change_begin_index = lat_beh_val / 2;
        }
        
        // Supple data 
        std::vector<VehicleIntention> intention_sequence;
        if (lat_beh == LateralBehavior::LaneKeeping) {
            for (int i = 0; i < 10; i++) {
                intention_sequence.emplace_back(VehicleIntention(lat_beh, lon_vel_comp));
            }
        } else {
            for (int i = 0; i < 10; i++) {
                if (i < change_begin_index) {
                    intention_sequence.emplace_back(VehicleIntention(LateralBehavior::LaneKeeping, lon_vel_comp));
                } else {
                    intention_sequence.emplace_back(VehicleIntention(lat_beh, lon_vel_comp));
                }
            }
        }

        return intention_sequence;
    }

    /**
     * @brief transform a series of action indices
     * @param {*}
     * @return {*}
     */   
    std::vector<std::vector<VehicleIntention>> ActionInterface::indexVecToIntentionSeqVec(const std::vector<int>& action_indices) {
        int length = static_cast<int>(action_indices.size());
        std::vector<std::vector<VehicleIntention>> intention_sequence_vec(length);
        for (int i = 0; i < length; i++) {
            intention_sequence_vec[i] = indexToIntentionSeq(action_indices[i]);
        }
        return intention_sequence_vec;
    }


    StateInterface::StateInterface(const Lane& nearest_lane) {
        stf_ = new StateTransformer(nearest_lane);
    }
    StateInterface::StateInterface() = default;
    StateInterface::~StateInterface() = default;

    /**
     * @brief generate state from environment information
     * @param lane_info lane information
     * @param ego_vehicle ego vehicle information
     * @param sur_vehicles surround vehicles information
     * @param state state array generated
     */
    void StateInterface::runOnce(const std::vector<double>& lane_info, const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& sur_vehicles, std::vector<double>* state) {
        std::vector<double> state_array;
        
        // ~Stage I: supple lane information
        assert(static_cast<int>(lane_info.size()) == 5);
        state_array.insert(state_array.end(), lane_info.begin(), lane_info.end());

        // ~Stage II: supple ego vehicle information
        std::vector<double> ego_vehicle_state_array = transformEgoVehicleState(ego_vehicle);
        assert(static_cast<int>(ego_vehicle_state_array.size()) == 9);
        state_array.insert(state_array.end(), ego_vehicle_state_array.begin(), ego_vehicle_state_array.end());

        // ~Stage III: supple surround vehicles information
        std::vector<double> surround_vehicles_states_array = transformSurroundVehicleState(sur_vehicles);
        assert(static_cast<int>(surround_vehicles_states_array.size()) == 80);
        state_array.insert(state_array.end(), surround_vehicles_states_array.begin(), surround_vehicles_states_array.end());

        *state = state_array;
    }

    /**
     * @brief transform single vehicle state
     * @param {*}
     * @return {*}
     */    
    std::vector<double> StateInterface::transformEgoVehicleState(const Vehicle& vehicle) {
        assert(vehicle.id_ == 0);
        return stf_->getFrenetEgoVehicleStateArray(vehicle);
    }

    /**
     * TODO: maybe a logic could be added here to distinct the differerce of different surround vehicles
     * @brief transform surround vehicles states
     * @param {*}
     * @return {*}
     */    
    std::vector<double> StateInterface::transformSurroundVehicleState(const std::unordered_map<int, Vehicle>& sur_vehicles) {
        // Initial surround vehicles states array, which incluides 10 vehicles most
        std::vector<double> sur_vehicles_states;
        // TODO: add logic to handle the situation where there are more than 10 surround vehicles
        if (static_cast<int>(sur_vehicles.size()) > 10) {
            printf("[HpdmPlanner][WARNING] there is more than 10 ten laned surround vehicles.\n");
        }

        // Traverse existed surround vehicles
        for (int i = 1; i < 11; i++) {
            std::vector<double> cur_sur_veh_frenet_state_array;
            if (sur_vehicles.count(i)) {
                cur_sur_veh_frenet_state_array = stf_->getFrenetSurroundVehicleStateArray(sur_vehicles.at(i));
            } else {
                // Supple empty surround vehicles
                cur_sur_veh_frenet_state_array = std::vector<double>(8, 0.0);
            }
            sur_vehicles_states.insert(sur_vehicles_states.end(), cur_sur_veh_frenet_state_array.begin(), cur_sur_veh_frenet_state_array.end());
        }

        assert(static_cast<int>(sur_vehicles_states.size()) == 80);

        return sur_vehicles_states;

    }

    TorchInterface::TorchInterface(const std::string& model_path) {
        model_path_ = model_path;
    }
    TorchInterface::~TorchInterface() = default;

    /**
     * @brief generate best action index from forwarding model
     * @param state state inputed to the model
     * @param action_index action generated by model
     */    
    void TorchInterface::runOnce(const std::vector<double>& state, std::vector<int>* candi_action_indices) {
        // Load model
        torch::jit::script::Module module = torch::jit::load(model_path_);

        // Convert data
        torch::Tensor state_tensor = torch::tensor(state).unsqueeze(0);
        std::vector<torch::jit::IValue> inputs;
        inputs.emplace_back(state_tensor);
        
        // Forward 
        torch::Tensor pred_res = module.forward(inputs).toTensor();
        std::tuple<torch::Tensor, torch::Tensor> result = pred_res.topk(16, 1);
        auto top_values = std::get<0>(result).view(-1);
        auto top_idxs = std::get<1>(result).view(-1);
        std::vector<int> res(top_idxs.data_ptr<long>(), top_idxs.data_ptr<long>() + top_idxs.numel());

        printf("[HpdmPlanner] candidate action indices include: \n");
        for (int i = 0; i < static_cast<int>(res.size()); i++) {
            printf("%d, ", res[i]);
        }
        printf("\n");

        // Cache 
        *candi_action_indices = res;
    }


    TrajectoryGenerator::TrajectoryGenerator(BehaviorPlanner::MapInterface* map_itf, double dt) {
        map_itf_ = map_itf;
        dt_ = dt;
    }
    TrajectoryGenerator::~TrajectoryGenerator() = default;

    /**
     * @brief Simulate single behavior sequence
     * @param surround_vehicles
     * @param ego_vehicle
     * @param behavior_sequence
     * @param ego_traj
     * @param safe
     * @param cost
     */    
    void TrajectoryGenerator::simulateSingleBehaviorSequence(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles, const BehaviorSequence& behavior_sequence, Trajectory* ego_traj, std::unordered_map<int, Trajectory>* sur_trajs, bool* safe, double* cost, Lane* target_behavior_reference_lane) {
        
        // Initialize semantic vehicles (include ego semantic vehicle and surround semantic vehicles)
        SemanticVehicle ego_semantic_vehicle = map_itf_->getEgoSemanticVehicle(ego_vehicle, behavior_sequence[0].lat_beh_);
        std::unordered_map<int, SemanticVehicle> surround_semantic_vehicles = map_itf_->getSurroundSemanticVehicles(surround_vehicles);

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
                ego_semantic_vehicle = map_itf_->getEgoSemanticVehicle(current_ego_vehicle, behavior_sequence[i].lat_beh_);
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
        
        // Judge whether generate lane change behavior
        bool lane_change_flag{false};
        for (const VehicleBehavior& veh_beh: behavior_sequence) {
            if (veh_beh.lat_beh_ != LateralBehavior::LaneKeeping) {
                lane_change_flag = true;
                break;
            }
        }

        // Calculate target speed in nearest lane
        // Calculate ego vehicle's last predict state
        Vehicle ego_vehicle_last_desired_state = ego_trajectory.back();
        // Calculate speed max limit
        double speed_limit = map_itf_->calculateSpeedLimit(ego_vehicle_last_desired_state);

        // Calculate policy situation whether safe
        bool is_safe = BehaviorPlanner::PolicyEvaluater::calculateSafe(ego_trajectory, surround_trajectories, speed_limit);

        // Calculate policy situation cost
        double behavior_cost = BehaviorPlanner::PolicyEvaluater::calculateCost(ego_trajectory, surround_trajectories, lane_change_flag, speed_limit);

        // Cache
        *ego_traj = ego_trajectory;
        *sur_trajs = surround_trajectories;
        *safe = is_safe;
        *cost = behavior_cost;
        *target_behavior_reference_lane = map_itf_->lane_set_[ego_semantic_vehicle.reference_lane_id_];
    }

    /**
     * @brief Simulate single behavior sequence
     * @param surround_vehicles
     * @param ego_vehicle
     * @param behavior_sequence
     * @param ego_traj
     * @param safe
     * @param cost
     */    
    void TrajectoryGenerator::simulateSingleIntentionSequence(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles, const IntentionSequence& intention_sequence, Trajectory* ego_traj, std::unordered_map<int, Trajectory>* sur_trajs, bool* safe, double* cost, Lane* target_intention_reference_lane) {
        
        // Initialize semantic vehicles (include ego semantic vehicle and surround semantic vehicles)
        SemanticVehicle ego_semantic_vehicle = map_itf_->getEgoSemanticVehicle(ego_vehicle, intention_sequence[0].lat_beh_);
        std::unordered_map<int, SemanticVehicle> surround_semantic_vehicles = map_itf_->getSurroundSemanticVehicles(surround_vehicles);

        // Determine desired speed based on longitudinal speed
        double ego_vehicle_desired_speed = std::max(ego_vehicle.state_.velocity_ + intention_sequence[0].lon_vel_comp_, 0.0);

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
        for (int i = 0; i < static_cast<int>(intention_sequence.size()); i++) {
            // Update the lateral behavior and reference lane for ego semantic vehicle
            if (i > 0 && intention_sequence[i].lat_beh_ != intention_sequence[i - 1].lat_beh_) {
                ego_semantic_vehicle = map_itf_->getEgoSemanticVehicle(current_ego_vehicle, intention_sequence[i].lat_beh_);
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
        
        // Judge whether generate lane change behavior
        bool lane_change_flag{false};
        for (const VehicleIntention& veh_intention: intention_sequence) {
            if (veh_intention.lat_beh_ != LateralBehavior::LaneKeeping) {
                lane_change_flag = true;
                break;
            }
        }

        // Calculate target speed in nearest lane
        // Calculate ego vehicle's last predict state
        Vehicle ego_vehicle_last_desired_state = ego_trajectory.back();
        // Calculate speed max limit
        double speed_limit = map_itf_->calculateSpeedLimit(ego_vehicle_last_desired_state);

        // Calculate policy situation whether safe
        bool is_safe = BehaviorPlanner::PolicyEvaluater::calculateSafe(ego_trajectory, surround_trajectories, speed_limit);

        // Calculate policy situation cost
        double behavior_cost = BehaviorPlanner::PolicyEvaluater::calculateCost(ego_trajectory, surround_trajectories, lane_change_flag, speed_limit);

        // Cache
        *ego_traj = ego_trajectory;
        *sur_trajs = surround_trajectories;
        *safe = is_safe;
        *cost = behavior_cost;
        *target_intention_reference_lane = map_itf_->lane_set_[ego_semantic_vehicle.reference_lane_id_];
    }

    // Simulate single vehicle behavior (lateral and longitudinal)
    void TrajectoryGenerator::simulateSingleBehavior(const SemanticVehicle& ego_semantic_vehicle, const std::unordered_map<int, SemanticVehicle>& surround_semantic_vehicles, const double& ego_desired_velocity, Vehicle& ego_vehicle_next_state, std::unordered_map<int, Vehicle>& surround_vehicles_next_states) {
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
            Vehicle cur_vehicle_desired_state = BehaviorPlanner::ForwardExtender::extendState(map_itf_, all_semantic_vehicles, veh_info.first, dt_, desired_velocity);

            // State cache
            if (veh_info.first == 0) {
                ego_vehicle_next_state = cur_vehicle_desired_state;
            } else {
                surround_vehicles_next_states[veh_info.first] = cur_vehicle_desired_state;
            }
        }
    }

    /**
     * @brief simulate all candidates behavior
     * @param {*}
     * @return {*}
     */
    void TrajectoryGenerator::simulateCandidatesBehaviorSequences(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles, const std::vector<BehaviorSequence>& candi_sequences, Trajectory* ego_traj, std::unordered_map<int, Trajectory>* sur_trajs, bool* safe, double* cost, Lane* target_reference_lane, int* final_action_index) {
        // Initialize containers
        int candi_length = static_cast<int>(candi_sequences.size());
        candi_ego_trajs_.resize(candi_length);
        candi_sur_trajs_.resize(candi_length);
        candi_safes_.resize(candi_length);
        candi_costs_.resize(candi_length);
        candi_reference_lanes_.resize(candi_length);

        // Calculate with multi threads
        std::vector<std::thread> threads(candi_length);
        for (int i = 0; i < candi_length; i++) {
            threads[i] = std::thread(&TrajectoryGenerator::simulateSingleCandiBehaviorSequence, this, ego_vehicle, surround_vehicles, candi_sequences[i], i);
        }
        for (int i = 0; i < candi_length; i++) {
            threads[i].join();
        }

        // Select the best sequence
        int win_idx = -1;
        double win_cost = MAX_VALUE;
        for (int i = 0; i < candi_length; i++) {
            if (!candi_safes_[i]) {
                continue;
            }
            if (candi_costs_[i] < win_cost) {
                win_idx = i;
                win_cost = candi_costs_[i];
            }
        }
        if (win_idx == -1) {
            *safe = false;
            return;
        }

        // Cache
        *ego_traj = candi_ego_trajs_[win_idx];
        *safe = candi_safes_[win_idx];
        *cost = win_cost;
        *target_reference_lane = candi_reference_lanes_[win_idx];
        *final_action_index = win_idx;
    }

    /**
     * @brief simulate all candidates behavior
     * @param {*}
     * @return {*}
     */
    void TrajectoryGenerator::simulateCandidatesIntentionSequences(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles, const std::vector<IntentionSequence>& candi_sequences, Trajectory* ego_traj, std::unordered_map<int, Trajectory>* sur_trajs, bool* safe, double* cost, Lane* target_reference_lane, int* final_action_index) {
        // Initialize containers
        int candi_length = static_cast<int>(candi_sequences.size());
        candi_ego_trajs_.resize(candi_length);
        candi_sur_trajs_.resize(candi_length);
        candi_safes_.resize(candi_length);
        candi_costs_.resize(candi_length);
        candi_reference_lanes_.resize(candi_length);

        // // Calculate with multi threads
        // std::vector<std::thread> threads(candi_length);
        // for (int i = 0; i < candi_length; i++) {
        //     threads[i] = std::thread(&TrajectoryGenerator::simulateSingleCandiIntentionSequence, this, ego_vehicle, surround_vehicles, candi_sequences[i], i);
        // }
        // for (int i = 0; i < candi_length; i++) {
        //     threads[i].join();
        // }

        // DEBUG
        for (int i = 0; i < candi_length; i++) {
            simulateSingleCandiIntentionSequence(ego_vehicle, surround_vehicles, candi_sequences[i], i);
        }
        // END DEBUG

        // Select the best sequence
        int win_idx = -1;
        double win_cost = MAX_VALUE;
        for (int i = 0; i < candi_length; i++) {
            if (!candi_safes_[i]) {
                continue;
            }
            if (candi_costs_[i] < win_cost) {
                win_idx = i;
                win_cost = candi_costs_[i];
            }
        }

        // Cache
        *ego_traj = candi_ego_trajs_[win_idx];
        *safe = candi_safes_[win_idx];
        *cost = win_cost;
        *target_reference_lane = candi_reference_lanes_[win_idx];
        *final_action_index = win_idx;
    }

    /**
     * @brief multi thread interface 
     * @param {*}
     * @return {*}
     */    
    void TrajectoryGenerator::simulateSingleCandiBehaviorSequence(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles, const BehaviorSequence& executed_sequence, int index) {
        // Initialize
        Trajectory ego_traj;
        std::unordered_map<int, Trajectory> sur_trajs; 
        bool safe = false;
        double cost = 0.0; 
        Lane target_reference_lane;

        // Calculate 
        simulateSingleBehaviorSequence(ego_vehicle, surround_vehicles, executed_sequence, &ego_traj, &sur_trajs, &safe, &cost, &target_reference_lane);

        // Cache
        candi_ego_trajs_[index] = ego_traj;
        candi_sur_trajs_[index] = sur_trajs;
        candi_safes_[index] = safe;
        candi_costs_[index] = cost;
        candi_reference_lanes_[index] = target_reference_lane;
    }

    /**
     * @brief multi thread interface 
     * @param {*}
     * @return {*}
     */    
    void TrajectoryGenerator::simulateSingleCandiIntentionSequence(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles, const IntentionSequence& executed_sequence, int index) {
        // Initialize
        Trajectory ego_traj;
        std::unordered_map<int, Trajectory> sur_trajs; 
        bool safe = false;
        double cost = 0.0; 
        Lane target_reference_lane;

        // Calculate 
        simulateSingleIntentionSequence(ego_vehicle, surround_vehicles, executed_sequence, &ego_traj, &sur_trajs, &safe, &cost, &target_reference_lane);

        // Cache
        candi_ego_trajs_[index] = ego_traj;
        candi_sur_trajs_[index] = sur_trajs;
        candi_safes_[index] = safe;
        candi_costs_[index] = cost;
        candi_reference_lanes_[index] = target_reference_lane;
    }

    HpdmPlannerCore::HpdmPlannerCore(BehaviorPlanner::MapInterface* map_itf, const Lane& nearest_lane, const std::string& model_path) {
        map_itf_ = map_itf;
        traj_generator_ = new TrajectoryGenerator(map_itf);
        state_itf_ = new StateInterface(nearest_lane);
        torch_itf_ = new TorchInterface(model_path);
    }
    HpdmPlannerCore::HpdmPlannerCore(BehaviorPlanner::MapInterface* map_itf, const Lane& nearest_lane, const std::string& model_path, const ros::Publisher& vis_pub) {
        map_itf_ = map_itf;
        traj_generator_ = new TrajectoryGenerator(map_itf);
        state_itf_ = new StateInterface(nearest_lane);
        torch_itf_ = new TorchInterface(model_path);
        vis_pub_ = vis_pub;

    }
    HpdmPlannerCore::~HpdmPlannerCore() = default;

    // Load data
    void HpdmPlannerCore::load(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles, const std::vector<double>& lane_info) {
        ego_vehicle_ = ego_vehicle;
        surround_vehicles_ = surround_vehicles;
        lane_info_ = lane_info;
    }

    // Run HPDM planner
    void HpdmPlannerCore::runHpdmPlanner(int lon_candidate_num, std::vector<Vehicle>* ego_traj, std::unordered_map<int, std::vector<Vehicle>>* sur_trajs, Lane* target_reference_lane, bool* safe, double* cost) {
        // ~Stage I: construct state array
        std::vector<double> state_array;
        state_itf_->runOnce(lane_info_, ego_vehicle_, surround_vehicles_, &state_array);
        
        // // DEBUG
        // for (const auto& state_it : state_array) {
        //     std::cout << state_it << std::endl;
        // }
        // // END DEBUG

        // ~Stage II: model predict to generate action index
        std::vector<int> candi_action_idxs;
        torch_itf_->runOnce(state_array, &candi_action_idxs);

        // ~Stage III: generate behavior / intention sequence from action index, and do pre-process
        std::vector<std::vector<VehicleBehavior>> behavior_sequence_vec_raw;
        std::vector<std::vector<VehicleIntention>> intention_sequence_vec_raw;
        std::vector<std::vector<VehicleBehavior>> behavior_sequence_vec;
        std::vector<std::vector<VehicleIntention>> intention_sequence_vec;
        if (lon_candidate_num == 3) {
            behavior_sequence_vec_raw = ActionInterface::indexVecToBehSeqVec(candi_action_idxs);
            for (const auto& beh_seq : behavior_sequence_vec_raw) {
                if (beh_seq.back().lat_beh_ == LateralBehavior::LaneChangeLeft && !map_itf_->left_lane_exist_) {
                    continue;
                }
                if (beh_seq.back().lat_beh_ == LateralBehavior::LaneChangeRight && !map_itf_->right_lane_exist_) {
                    continue;
                }
                behavior_sequence_vec.emplace_back(beh_seq);
            }
        } else if (lon_candidate_num == 11) {
            intention_sequence_vec_raw = ActionInterface::indexVecToIntentionSeqVec(candi_action_idxs);
            for (const auto& intention_seq : intention_sequence_vec_raw) {
                if (intention_seq.back().lat_beh_ == LateralBehavior::LaneChangeLeft && !map_itf_->left_lane_exist_) {
                    continue;
                }
                if (intention_seq.back().lat_beh_ == LateralBehavior::LaneChangeRight && !map_itf_->right_lane_exist_) {
                    continue;
                }
                intention_sequence_vec.emplace_back(intention_seq);
            }
        } else {
            assert(false);
        }

        // // DEBUG
        // printf("DEBUG valid behavior sequence number: %d.\n", behavior_sequence_vec.size());
        // // END DEBUG

        // ~Stage IV: generate trajectories for all vehicles and additional information 
        std::vector<Vehicle> ego_trajectory;
        std::unordered_map<int, std::vector<Vehicle>> sur_trajectories;
        bool is_safe = false;
        double policy_cost = 0.0;
        Lane target_ref_lane;
        int final_win_index = -1;

        if (lon_candidate_num == 3) {
            traj_generator_->simulateCandidatesBehaviorSequences(ego_vehicle_, surround_vehicles_, behavior_sequence_vec, &ego_trajectory, &sur_trajectories, &is_safe, &policy_cost, &target_ref_lane, &final_win_index);
        } else if (lon_candidate_num == 11) {
            traj_generator_->simulateCandidatesIntentionSequences(ego_vehicle_, surround_vehicles_, intention_sequence_vec, &ego_trajectory, &sur_trajectories, &is_safe, &policy_cost, &target_ref_lane, &final_win_index);
        } else {
            assert(false);
        }

        // Visualization and print
        VisualizationMethods::visualizeTrajectory(ego_trajectory, vis_pub_, 0);
        printf("[HpdmPLanner] selected action index: %d, is safe: %d, cost: %lf.\n", candi_action_idxs[final_win_index], is_safe, policy_cost);


        *ego_traj = ego_trajectory;
        *sur_trajs = sur_trajectories;
        *safe = is_safe;
        *cost = policy_cost;
        *target_reference_lane = target_ref_lane;
    }
    
} // End of HpdmPlanner
