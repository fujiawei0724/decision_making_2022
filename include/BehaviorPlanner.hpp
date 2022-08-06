/*
 * @Author: fujiawei0724
 * @Date: 2021-11-08 18:50:38
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-08-05 21:02:25
 * @Descripttion: Behavior planner core.
 */

#pragma once

#include "Const.hpp"
#include "VehicleState.hpp"

namespace BehaviorPlanner {

using namespace Common;

// Config for behavior planner 
class BehaviorPlannerConfig {
public:
    // TODO: adjust parameters
    static constexpr double look_ahead_min_distance{3.0};
    static constexpr double look_ahead_max_distance{50.0};
    static constexpr double steer_control_gain{2.0};
    static constexpr double wheelbase_length{2.8498};

    static constexpr double max_lon_acc_jerk{5.0};
    static constexpr double max_lon_brake_jerk{5.0};
    static constexpr double max_lat_acceleration_abs{1.5};
    static constexpr double max_lat_jerk_abs{3.0};
    static constexpr double max_steer_angle_abs{45.0 / 180.0 * PI};
    static constexpr double max_steer_rate{0.39};
    static constexpr double max_curvature_abs{0.33};
    
};

class MapInterface {
public:
    // Constructor
    MapInterface(const std::map<LaneId, bool>& lane_exist, const std::map<LaneId, ParametricLane>& lane_info);

    // Destructor
    ~MapInterface();

    // Calculate nearest lane for a vehicle
    LaneId calculateNearestLaneId(const Vehicle& vehicle);

    // Calculate nearest lane for a position
    LaneId calculateNearestLaneId(const Eigen::Matrix<double, 2, 1>& position);

    // Calculate reference lane for a vehicle
    LaneId calculateReferenceLaneId(const LaneId& nearest_lane_id, const LateralBehavior& lat_beh);

    // Predict lateral behavior for surround vehicles based on vehicle information
    LateralBehavior predictSurroundVehicleLateralBehavior(const Vehicle& vehicle, const LaneId& nearest_lane_id);

    // Get semantic vehicle state (add lane information to vehicle state)
    SemanticVehicle getSingleSurroundSemanticVehicle(const Vehicle& surround_vehicle); 

    // Get multiple semantic vehicles
    std::unordered_map<int, SemanticVehicle> getSurroundSemanticVehicles(const std::unordered_map<int, Vehicle>& surround_vehicles);

    // Get ego semantic vehicle state
    // Note that ego vehicle's reference lane could be updated with the behavior sequence
    SemanticVehicle getEgoSemanticVehicle(const Vehicle& ego_vehicle, const LateralBehavior& ego_vehicle_lat_beh);

    // Get leading vehicle state
    bool getLeadingVehicle(const SemanticVehicle& cur_semantic_vehicle, const std::unordered_map<int, SemanticVehicle>& other_semantic_vehicles_set, Vehicle& leading_vehicle);

    // Calculate the speed limit given a vehicle state
    double calculateSpeedLimit(const Vehicle& vehicle);

    // Calculate point in lane
    bool isInLane(const PathPlanningUtilities::Point2f& position);

    // Calculate the orientation of a lane point that has the minimum distance from the specified position
    double calculateNearestPointOrientation(const PathPlanningUtilities::Point2f& position);

    /**
     * @brief Generate lane information for HPDM 
     * @param {*}
     * @return {*}
     */   
    std::vector<double> getLaneInfo();

    /**
     * @brief Calculate nearest lane for HPDM
     * @param {*}
     * @return {*}
     */
    ParametricLane calculateNearestLane(const Vehicle& vehicle);

    // Lane information in map interface
    bool center_lane_exist_{false};
    bool left_lane_exist_{false};
    bool right_lane_exist_{false};
    std::map<LaneId, ParametricLane> lane_set_;
};

// Intelligent driver model
class IDM {
public:
    IDM();
    ~IDM();

    double calculateAcceleration(double cur_s, double leading_s, double cur_velocity, double leading_velocity, double desired_velocity);

    double calculateVelocity(double input_cur_s, double input_leading_s, double input_cur_velocity, double input_leading_velocity, double dt, double desired_velocity);

    typedef boost::array<double, 4> InternalState;
    void operator()(const InternalState &x, InternalState &dxdt, const double /* t */);
    InternalState internal_state_;
    double desired_velocity_{0.0};

    static constexpr double vehicle_length_{5.0};
    static constexpr double minimum_spacing_{2.0};
    static constexpr double desired_headaway_time_{1.0};
    static constexpr double acceleration_{1.7};
    static constexpr double comfortable_braking_deceleration_{3.0};
    static constexpr double hard_braking_deceleration_{5.0};
    static constexpr double exponent_{4.0};

};

// Predict the vehicle desired state use velocity and steer
class IdealSteerModel {
public:

    // Constructor
    IdealSteerModel(double wheelbase_len, double max_lon_acc, double max_lon_dec, double max_lon_acc_jerk, double max_lon_dec_jerk, double max_lat_acc, double max_lat_jerk, double max_steering_angle, double max_steer_rate, double max_curvature);
    // Destructor
    ~IdealSteerModel();

    void setControl(const std::pair<double, double>& control);

    void setState(const State& state);

    void truncateControl(double dt);
    void step(double dt);

    // Update internal state
    void updateInternalState();

    typedef boost::array<double, 5> InternalState;
    void operator()(const InternalState &x, InternalState &dxdt, const double /* t */);

    double wheelbase_len_;
    double max_lon_acc_;
    double max_lon_dec_;
    double max_lon_acc_jerk_;
    double max_lon_dec_jerk_;
    double max_lat_acc_;
    double max_lat_jerk_;
    double max_steering_angle_;
    double max_steer_rate_;
    double max_curvature_;

    // State
    std::pair<double, double> control_{0.0, 0.0};
    State state_;
    // Internal_state[0] means x position, internal_state[1] means y position, internal_state[2] means angle, internal_state[3] means velocity, internal_state[4] means steer
    // Eigen::Matrix<double, 5, 1> internal_state_{Eigen::Matrix<double, 5, 1>::Zero()};
    InternalState internal_state_;

    // Parameters need to calculated
    double desired_lon_acc_{0.0};
    double desired_lat_acc_{0.0};
    double desired_steer_rate_{0.0};
};

// Update all vehicles' states
class ForwardExtender {
public:
    // Calculate vehicle steer 
    static double calculateSteer(SemanticVehicle* semantic_vehicle);

    // Calculate vehicle velocity 
    static double calculateVelocity(MapInterface* mtf, const SemanticVehicle& cur_semantic_vehicle, const std::unordered_map<int, SemanticVehicle>& semantic_vehicles, double dt, double desired_velocity);

    // Calculate desired vehicle stateuse steer and velocity based on ideal steer model
    static Vehicle calculateDesiredVehicleState(const Vehicle& veh, const double& steer, const double& velocity, const double& dt);


    /**
     * @introduction: Forward one step, this function is for all vehicle agents, not only for ego vehicle
     * @param semantic_vehicles denotes the all vehicles (include the ego vehicle)
     * @return current vehicle's extended state
     */   
    // TODO: add detailed logic for ego vehicle state update, which means a different calculation method with surround vehicles 
    static Vehicle extendState(MapInterface* mtf, std::unordered_map<int, SemanticVehicle>& semantic_vehicles, int current_vehicle_id, double dt, double desired_velocity);
};


// Generator vehicle behavior sequence 
// TODO: add the consideration of current behavior
class BehaviorGenerator {
public:
    
    // Constructor
    BehaviorGenerator(MapInterface* mtf, int sequence_length);

    // Destructor
    ~BehaviorGenerator();

    // Generate vehicle behavior sequence
    std::vector<std::vector<VehicleBehavior>> generateBehaviorSequence();

    // Generate MPDM vehicle behavior sequence
    std::vector<std::vector<VehicleBehavior>> generateMPDMBehavior();

    // Add lane change behavior to supply behavior sequence
    static std::vector<VehicleBehavior> completeBehaviorSequence(const std::vector<VehicleBehavior>& cur_beh_seq, LateralBehavior lat_beh, LongitudinalBehavior lon_beh, int num);
    
    bool is_lane_keeping_available_{false};
    bool is_lane_change_left_available_{false};
    bool is_lane_change_right_available_{false};
    int sequence_length_;
};

class PolicyEvaluater {
public:
    using Trajectory = std::vector<Vehicle>;
    
    // Calculate all costs
    // TODO: add safety cost consider the collision and collision velocity
    static double calculateCost(const Trajectory& ego_trajectory, const std::unordered_map<int, Trajectory>& surround_trajectories, const bool& is_lane_changed, const double& lane_speed_limit);

    // Calculate lane change action cost
    static double calculateActionCost(const bool& is_lane_changed);

    // Calculate efficiency cost, due to ego current velocity and lane limit velocity
    static double calculateEfficiencyCost(const Trajectory& ego_trajectory, const double& speed_limit);

    // Calculate consistence cost
    static double calculateConsistenceCost(const Trajectory& ego_trajectory, const ParametricLane& pre_reference_lane, const Vehicle& pre_ego_desired_vehicle_state);

    // Judge is safe
    static bool calculateSafe(const Trajectory& ego_trajectory, const std::unordered_map<int, Trajectory>& surround_trajectories, double speed_limit);
};




// Transform ego vehicle and obstacle vehicle information to behavior planner interface
class VehicleInterface {
public:

    // Calculate ego vehicle
    static Vehicle getEgoVehicle(const Eigen::Matrix<double, 2, 1>& position, const double& theta, const double& curvature, const double& velocity, const double& acceleration, const double& steer, const double& length, const double& width);

    /**
     * @brief: Calculate surround vehicle from perception obstacles
     * @param unlaned_obstacles means the obstacles ar detached from the lane network, they are employed to generate occupied semantic cubes in trajectory planner. 
     * @return The vehicles could be handle in behavior planner.
     */    
    static std::unordered_map<int, Vehicle> getSurroundVehicles(MapInterface* mtf, const std::vector<Obstacle>& obstacles, std::vector<Obstacle>& unlane_obstacles);

    // Calculate single surround vehicle
    static bool getSingleSurroundVehicle(MapInterface* mtf, const Obstacle& obstacle, int index, std::pair<int, Vehicle>& sur_veh_info);


};

// Behavior planner core
class BehaviorPlannerCore {
 public:
    using Trajectory = std::vector<Vehicle>;
    using BehaviorSequence = std::vector<VehicleBehavior>;
    // Add the variability of behavior sequence
    BehaviorPlannerCore(MapInterface* mtf, double predict_time_span, double dt);
    BehaviorPlannerCore(MapInterface* mtf, double predict_time_span, double dt, const ros::Publisher& vis_pub);
    ~BehaviorPlannerCore();

    // Behavior planner runner
    bool runBehaviorPlanner(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles, Trajectory* ego_best_traj, std::unordered_map<int, Trajectory>* sur_best_trajs, ParametricLane* target_behavior_reference_lane);

    // Evaluate all policies
    void evaluatePolicies(int& winner_index, double& winner_cost);

    // Simulate all situation and store trajectories information
    void simulateAllBehaviors(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles);

    // Simluate multiple single behavior sequence synchronous
    void simulateMultipleBehaviorSequence(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles, const std::vector<BehaviorSequence>& behavior_set, const int& behavior_sequence_start_index, const int& behavior_sequence_executed_num, const int& sequence_num);

    // Simulate single behavior sequence
    void simulateSingleBehaviorSequence(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles, const BehaviorSequence& behavior_sequence, int index);

    // Simulate single vehicle behavior (lateral and longitudinal)
    void simulateSingleBehavior(const SemanticVehicle& ego_semantic_vehicle, const std::unordered_map<int, SemanticVehicle>& surround_semantic_vehicles, const double& ego_desired_velocity, Vehicle& ego_vehicle_next_state, std::unordered_map<int, Vehicle>& surround_vehicles_next_states);
    

    // Initialize container for multiple threads calculation
    // TODO: add detailed cost information for the information of each behavior sequence
    void initializeContainer(int length);

    // Load consistence information
    void load(const ParametricLane& prev_ref_lane, const Vehicle& prev_ego_veh_desired_state);

    MapInterface* mtf_{nullptr};
    double predict_time_span_{0.0};
    double dt_{0.0};
    // DEBUG visualization
    ros::Publisher vis_pub_;

    // Store multiple thread information
    // Store the final cost information
    std::vector<double> behavior_sequence_cost_;
    std::vector<bool> behavior_safety_;
    // Store the trajectory information
    std::vector<LaneId> final_reference_lane_id_;
    std::vector<bool> is_lane_changed_;
    std::vector<double> target_position_velocity_limit_;
    std::vector<Trajectory> ego_traj_;
    std::vector<std::unordered_map<int, Trajectory>> sur_veh_trajs_;

    bool with_consistence_{false};
    ParametricLane previous_reference_lane_;
    Vehicle previous_ego_veh_desired_state_;

};

} // End of namespace BehaviorPlanner