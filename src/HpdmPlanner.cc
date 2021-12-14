/*
 * @Author: fujiawei0724
 * @Date: 2021-12-14 11:57:46
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-12-14 11:59:32
 * @Description: Hpdm planner.
 */

#include "Common.hpp"

namespace HpdmPlanner {
    HpdmPlannerCore::HpdmPlannerCore(BehaviorPlanner::MapInterface* map_itf, const Lane& nearest_lane, const std::string& model_path) {
        traj_generator_ = new TrajectoryGenerator(map_itf);
        state_itf_ = new StateInterface(nearest_lane);
        torch_itf_ = new TorchInterface(model_path);
    }
    HpdmPlannerCore::HpdmPlannerCore(BehaviorPlanner::MapInterface* map_itf, const Lane& nearest_lane, const std::string& model_path, const ros::Publisher& vis_pub) {
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

        // ~Stage II: model predict to generate action index
        int action_index = -1;
        torch_itf_->runOnce(state_array, &action_index);

        // ~Stage III: generate behavior / intention sequence from action index 
        std::vector<VehicleBehavior> behavior_sequence;
        std::vector<VehicleIntention> intention_sequence;
        if (lon_candidate_num == 3) {
            behavior_sequence = ActionInterface::indexToBehSeq(action_index);
        } else if (lon_candidate_num == 11) {
            intention_sequence = ActionInterface::indexToIntentionSeq(action_index);
        } else {
            assert(false);
        }

        // ~Stage IV: generate trajectories for all vehicles and additional information 
        std::vector<Vehicle> ego_trajectory;
        std::unordered_map<int, std::vector<Vehicle>> sur_trajectories;
        bool is_safe = false;
        double policy_cost = 0.0;
        Lane target_ref_lane;
        if (lon_candidate_num == 3) {
            traj_generator_->simulateSingleBehaviorSequence(ego_vehicle_, surround_vehicles_, behavior_sequence, &ego_trajectory, &sur_trajectories, &is_safe, &policy_cost, &target_ref_lane);
        } else if (lon_candidate_num == 11) {
            traj_generator_->simulateSingleIntentionSequence(ego_vehicle_, surround_vehicles_, intention_sequence, &ego_trajectory, &sur_trajectories, &is_safe, &policy_cost, &target_ref_lane);
        } else {
            assert(false);
        }

        // Visualization and print information
        VisualizationMethods::visualizeTrajectory(ego_trajectory, vis_pub_, 0);
        printf("[HpdmPLanner] action index: %d, is safe: %d, cost: %lf.\n", action_index, is_safe, policy_cost);


        *ego_traj = ego_trajectory;
        *sur_trajs = sur_trajectories;
        *safe = is_safe;
        *cost = policy_cost;
        *target_reference_lane = target_ref_lane;
    }
    
} // End of HpdmPlanner
