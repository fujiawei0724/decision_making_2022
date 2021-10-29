/*
 * @Author: fujiawei0724
 * @Date: 2021-10-27 11:36:32
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-10-29 15:59:56
 * @Descripttion: The class for EUDM behavior planner, such as the vehicle state and vehicle trajectory
 */

#pragma once

#include <unordered_map>
#include <map>
#include <algorithm>
#include <functional>
#include <stdio.h>
#include "Const.hpp"
#include "Point.hpp"
#include "Point.hpp"
#include "Const.hpp"
#include "Compare.hpp"
#include "Tools.hpp"
#include "Lane.hpp"
#include "Rectangle.hpp"

namespace BehaviorPlanner {

// TODO: break up the code to several files

// Lane id  
enum class LaneId {
    CenterLane = 0,
    LeftLane, 
    RightLane,
    Undefined,
}

// Lateral behavior
enum class LateralBehavior {
    LaneKeeping = 0,
    LaneChangeLeft,
    LaneChangeRight,
    MaxCount = 3,
};

// Longitudinal behavior
enum class LongitudinalBehavior {
    Normal = 0,
    Aggressive,
    Conservative,
    MaxCount = 3,
};

// Vehicle behavior 
class VehicleBehavior {
public:
    VehicleBehavior() = default;
    VehicleBehavior(const LateralBehavior& lat_beh, const LongitudinalBehavior& lon_beh) {
        lat_beh_ = lat_beh;
        lon_beh_ = lon_beh;
    }
    ~VehicleBehavior() {

    }

    LateralBehavior lat_beh_;
    LongitudinalBehavior lon_beh_;
};


// The description of vehicle state in world coordination
class State {
public: 
    // Constructor
    State() = default;
    State(double time_stamp, const Eigen::Matrix<double, 2, 1>& position, double theta, double curvature, double velocity, double acceleration, double steer) {
        time_stamp_ = time_stamp;
        position_ = position;
        theta_ = theta;
        curvature_ = curvature;
        velocity_ = velocity;
        acceleration_ = acceleration;
        steer_ = steer;
    }

    // Destructor
    ~State() {

    }

    void print() {
        printf("State time stamp: %lf\n", time_stamp_);
        printf("State position x: %lf\n", position_.x_);
        printf("State position y: %lf\n", position_.y_);
        printf("State theta: %lf\n", theta_);
        printf("State curvature: %lf\n", curvature_);
        printf("State velocity: %lf\n", velocity_);
        printf("State acceleration: %lf\n", acceleration_);
        printf("State steer: %lf\n", steer_);
    }

    double time_stamp_{0.0};
    Eigen::Matrix<double, 2, 1> position_{0.0, 0.0};
    double theta_{0.0};
    double curvature_{0.0};
    double velocity_{0.0};
    double acceleration_{0.0};
    double steer_{0.0};
};

/**
 * @introduction: The description of vehicle state in frenet coordination.
 * @param vec_ds_ denote the lateral offset based on the arc length s.
 * @param vec_dt_ denote the lateral offset based on the tim t. (Maybe useless)
 */
class FrenetState {
public: 
    // Constructor
    FrenetState() = default;
    FrenetState(const Eigen::Matrix<double, 3, 1>& s, const Eigen::Matrix<double, 3, 1>& ds, const Eigen::Matrix<double, 3, 1>& dt) {
        vec_s_ = s;
        vec_ds_ = ds;
        vec_dt_ = dt;
    }

    // Destructor
    ~FrenetState() {

    }

    /**
     * @introduction: Load the data to construct frenet state. Mayne need add a parameter to determine lateral dimension's related parameters(s or t).
     * @param s means the longitudinal dimension in frenet coordinate.
     * @param d means the lateral dimension in frenet coordinate
     */    
    void load(const Eigen::Matrix<double, 3, 1>& s, const Eigen::Matrix<double, 3, 1>& d) {
        vec_s_ = s;
        vec_ds_ = d;

        // Calculate vec_dt_ based on known parameters
        vec_dt_[0] = vec_ds_[0];
        vec_dt_[1] = vec_s_[1] * vec_ds_[1];
        vec_dt_[2] = vec_ds_[2] * vec_s_[1] * vec_s_[1] + vec_ds_[1] * vec_s_[2];
    }

    double time_stamp_{0.0};
    Eigen::Matrix<double, 3, 1> vec_s_{Eigen::Matrix<double, 3, 1>::Zero()};
    Eigen::Matrix<double, 3, 1> vec_ds_{Eigen::Matrix<double, 3, 1>::Zero()};
    Eigen::Matrix<double, 3, 1> vec_dt_{Eigen::Matrix<double, 3, 1>::Zero()};

    
};

// Transform between world state and frenet state
class StateTransformer {
public:
    StateTransformer() = default;
    StateTransformer(const Lane& lane) {
        lane_ = lane;
    }

    
    // Transform world state position to frenet state position     
    Eigen::Matrix<double, 2, 1> getFrenetPointFromPoint(const Eigen::Matrix<double, 2, 1>& state_position) const {
        // Transform point formulation
        PathPlanningUtilities::Point2f path_state_position;
        path_state_position.x_ = state_position(0);
        path_state_position.y_ = state_position(1);

        // Transform 2D point using matrix calculation
        Eigen::Matrix<double, 2, 1> frenet_state_position{0.0, 0.0};
        std::vector<PathPlanningUtilities::CoordinationPoint> lane_coordination = lane_.getLaneCoordnation();
        size_t start_index_of_lane = Tools::findNearestPositionIndexInCoordination(lane_coordination, path_state_position);
        std::vector<TransMatrix> trans_matrixes = lane_.getLaneTransMatrix();
        TransMatrix trans_matrix = trans_matrixes[start_index_of_lane];
        Eigen::Vector2d start_point_in_world_v2d(state_position(0), state_position(1));
        Eigen::Vector2d start_point_position_in_frenet;
        Tools::transferPointCoordinateSystem(trans_matrix, start_point_in_world_v2d, &start_point_position_in_frenet);
        frenet_state_position(0) = start_point_position_in_frenet(0);
        frenet_state_position(1) = start_point_position_in_frenet(1);

        return frenet_state_position;
    }

    // Get frenet state
    FrenetState getFrenetStateFromState(const State& state) const {
        // Get the most nearest lane point information
        std::vector<PathPlanningUtilities::CoordinationPoint> lane_coordination = lane_.getLaneCoordnation();
        size_t start_index_of_lane = Tools::findNearestPositionIndexInCoordination(lane_coordination, state.position_);
        PathPlanningUtilities::CoordinationPoint nearest_lane_point = lane_coordination[start_index_of_lane];
        Eigen::Matrix<double, 2, 1> lane_position{nearest_lane_point.worldpos_.position_.x_, nearest_lane_point.worldpos_.position_.y_};

        // Get curvature and curvature derivative information
        // TODO: load the curvature derivative information from map server
        double curvature = nearest_lane_point.worldpos_.kappa_;
        double curvature_derivative = 0.0;

        // Get the arc length
        std::vector<TransMatrix> trans_matrixes = lane_.getLaneTransMatrix();
        TransMatrix trans_matrix = trans_matrixes[start_index_of_lane];
        Eigen::Vector2d start_point_in_world_v2d(state.position_.x_, state.position_.y_);
        Eigen::Vector2d start_point_position_in_frenet;
        Tools::transferPointCoordinateSystem(trans_matrix, start_point_in_world_v2d, &start_point_position_in_frenet);
        double arc_length = start_point_position_in_frenet(0);

        // Get tangent vector and normal vector, the norm is set with 1.0
        double lane_orientation = nearest_lane_point.worldpos_.theta_;
        double y = tan(lane_orientation);
        double x = 1.0;
        Eigen::Matrix<double, 2, 1> lane_tangent_vec{x, y};
        lane_tangent_vec /= lane_tangent_vec.norm();
        Eigen::Matrix<double, 2, 1> lane_normal_vec{-lane_tangent_vec(1), lane_tangent_vec(0)};

        // Check tolerance
        const double step_tolerance = 0.5;
        if (fabs((state.position_ - lane_position).dot(lane_tangent_vec)) > step_tolerance) {
            printf("[StatsTransformer] Offset larger than tolerance.\n");
        }

        double d = (state.position_ - lane_position).dot(lane_normal_vec);
        double one_minus_curd = 1.0 - curvature * d;
        if (one_minus_curd < MIDDLE_EPS) {
            printf("[StateTransformer] d not valid for transform.\n");
        }
        double delta_theta = Tools::safeThetaTransform(state.theta_ - lane_orientation);

        double cn_delta_theta = cos(delta_theta);
        double tan_delta_theta = tan(delta_theta);
        double ds = one_minus_curd * tan_delta_theta;
        double dss = -(curvature_derivative * d + curvature * ds) * tan_delta_theta + one_minus_curd / pow(cn_delta_theta, 2) * (state.curvature_ * one_minus_curd / cn_delta_theta - curvature);
        double sp = state.velocity_ * cn_delta_theta / one_minus_curd;

        double delta_theta_derivative = 1.0 / (1.0 + pow(tan_delta_theta, 2)) * (dss * one_minus_curd + curvature * pow(ds, 2)) / pow(one_minus_curd, 2);

        double spp = (s.acceleration - pow(sp, 2) / cn_delta_theta * (one_minus_curd * tan_delta_theta * delta_theta_derivative - (curvature_derivative * d + curvature * ds))) * cn_delta_theta / one_minus_curd;

        // Construct frenet state
        FrenetState frenet_state;
        frenet_state.load(Eigen::Matrix<double, 3, 1>(arc_length, sp, spp), Eigen::Matrix<double, 3, 1>(d, ds, dss));

        return frenet_state;
    }

    // Get state 
    State getStateFromFrenetState(const FrenetState& frenet_state) const {
        // Determine the nearest position to frenet state
        std::vector<PathPlanningUtilities::CoordinationPoint> lane_coordination = lane_.getLaneCoordnation();
        PathPlanningUtilities::CoordinationPoint lane_position;
        for (const PathPlanningUtilities::CoordinationPoint& lane_point: lane_coordination) {
            if (lane_point.station_ > frenet_state.vec_s_[0]) {
                lane_position = lane_point;
                break;
            }
        }
        Eigen::Matrix<double, 2, 1> lane_pos{lane_position.worldpos_.position_.x_, lane_position.worldpos_.position_.y_};


        // Get curvature and curvature derivative information
        // TODO: load the curvature derivative information from map server
        double curvature = lane_position.worldpos_.kappa_;
        double curvature_derivative = 0.0;

        double one_minus_curd = 1.0 - curvature * frenet_state.vec_s_[0];
        if (one_minus_curd < MIDDLE_EPS) {
            printf("[StateTransformer] one_minus_curd not valid for transform.\n")
        }

        // Get tangent vector and normal vector, the norm is set with 1.0
        double lane_orientation = nearest_lane_point.worldpos_.theta_;
        double y = tan(lane_orientation);
        double x = 1.0;
        Eigen::Matrix<double, 2, 1> vec_tangent{x, y};
        lane_tangent_vec /= lane_tangent_vec.norm();
        Eigen::Matrix<double, 2, 1> vec_normal{-lane_tangent_vec(1), lane_tangent_vec(0)};
        double tan_delta_theta = frenet_state.vec_ds_[1] / one_minus_curd;
        double delta_theta = atan2(frenet_state.vec_ds_[1], one_minus_curd);
        double cn_delta_theta = cos(delta_theta);

        // Construct state in world
        State state;
        state.position_ = vec_normal * frenet_state.vec_ds_[0] + lane_pos;
        state.velocity_ = frenet_state.vec_s_[1] * one_minus_curd / cn_delta_theta;
        state.theta_ = Tools::safeThetaTransform(delta_theta + lane_orientation);

        double lhs = (frenet_state.vec_ds_[2] + (curvature_derivative * frenet_state.vec_ds_[0] + curvature * frenet_state.vec_ds_[1]) * tan_delta_theta) * cn_delta_theta * cn_delta_theta / one_minus_curd;
        state.curvature_ = (lhs + curvature) * cn_delta_theta / one_minus_curd;
        double delta_theta_derivative = 1.0 / (1.0 + tan_delta_theta * tan_delta_theta) * (frenet_state.vec_ds_[2] * one_minus_curd + curvature * frenet_state.vec_ds_[1] * frenet_state.vec_ds_[1]) / pow(one_minus_curd, 2);
        state.acceleration_ = frenet_state.vec_s_[2] * one_minus_curd / cn_delta_theta + frenet_state.vec_s_[1] * frenet_state.vec_s_[1] / cn_delta_theta * (one_minus_curd * tan_delta_theta * delta_theta_derivative - (curvature_derivative * frenet_state.vec_ds_[0] + curvature * frenet_state.vec_ds_[1]));
        state.time_stamp_ = frenet_state.time_stamp_;

        return state;
    }

    Lane lane_;
};

class Vehicle {
public:
    // Constructor
    Vehicle() = default;
    Vehicle(int id, const State& state, double length, double width) {
        id_ = id;
        state_ = state;
        length_ = length;
        width_ = width;
    }

    // Destructor
    ~Vehicle() {

    }

    int id_{0};
    State state_;
    double length_{0.0};
    double width_{0.0};
};

// Vehicle state with semantic information, such as current lane and reference lane
class SemanticVehicle {
public:
    // Constructor
    SemanticVehicle() = default;
    SemanticVehicle(const Vehicle& vehicle, const LateralBehavior& lat_beh, const LongitudinalBehavior& lon_beh, const LaneId& nearest_lane_id, const LaneId& reference_lane_id, const Lane& nearest_lane, const Lane& reference_lane) {
        vehicle_ = vehicle;
        lat_beh_ = lat_beh;
        lon_beh_ = lon_beh;
        nearest_lane_id_ = nearest_lane_id;
        reference_lane_id_ = reference_lane_id;
        nearest_lane_ = nearest_lane;
        reference_lane_id_ = reference_lane_id;
    }

    // Destructor
    ~SemanticVehicle() {
        
    }

    Vehicle vehicle_;
    LateralBehavior lat_beh_;
    LongitudinalBehavior lon_beh_;
    LaneId nearest_lane_id_{Undefined};
    LaneId reference_lane_id_{Undefined};
    Lane nearest_lane_;
    Lane reference_lane_;
};

// Generator vehicle behavior sequence 
// TODO: add the consideration of current behavior
class BehaviorGenerator {
public:
    
    // Constructor
    BehaviorGenerator(int sequence_length) {
        sequence_length_ = sequence_length;
    }

    // Destructor
    ~BehaviorGenerator() {

    }

    // Generate vehicle behavior sequence
    std::vector<std::vector<VehicleBehavior>> generateBehaviorSequence() {
        // Initialize result
        std::vector<std::vector<VehicleBehavior>> veh_beh_seq;

        // Traverse all behavior possibility
        for (int lon = 0; lon < static_cast<int>(LongitudinalBehavior::MaxCount); lon++) {
            std::vector<VehicleBehavior> cur_beh_seq;
            for (int beh_index = 0; beh_index < sequence_length_; beh_index++) {
                for (int lat = 0; lat < static_cast<int>(LateralBehavior::MaxCount); lat++) {
                    if (LateralBehavior(lat) != LateralBehavior::LaneKeeping) {

                        // Complete lane change situations
                        veh_beh_seq.emplace_back(completeBehaviorSequence(cur_beh_seq, LateralBehavior(lat), LongitudinalBehavior(lon), sequence_length_ - beh_index));
                    }
                }
                cur_beh_seq.emplace_back(VehicleBehavior(LateralBehavior::LaneKeeping, LongitudinalBehavior(lon)));
            }
            veh_beh_seq.emplace_back(cur_beh_seq);
        }

        return veh_beh_seq;

    }

    // Add lane change behavior to supple behavior sequence
    static std::vector<VehicleBehavior> completeBehaviorSequence(const std::vector<VehicleBehavior>& cur_beh_seq, LateralBehavior lat_beh, LongitudinalBehavior lon_beh, int num) {
        // Initialize
        std::vector<VehicleBehavior> completed_beh_seq = cur_beh_seq;

        // Add lane change behavior 
        for (int i = 0; i < num; i++) {
            completed_beh_seq.emplace_back(VehicleBehavior(lat_beh, lon_beh));
        }

        return completed_beh_seq;
    }
    

    int sequence_length_;
};

class MapInterface {
public:
    // Constructor
    MapInterface(const std::unordered_map<LaneId, bool>& lane_exist, const std::unordered_map<LaneId, Lane>& lane_info) {
        // Initialize lane information
        if (lane_exist[LaneId::CenterLane]) {
            center_lane_exist_ = true;
        }
        if (lane_exist[LaneId::LeftLane]) {
            left_lane_exist_ = true;
        }
        if (lane_exist[LaneId::RightLane]) {
            right_lane_exist_ = true;
        }
        lane_set_ = lane_info;
    }

    // Destructor
    ~MapInterface() {

    }

    // Calculate nearest lane for a vehicle
    LaneId calculateNearestLaneId(const Vehicle& vehicle) {
        // Calculate distance to each lane
        std::vector<std::pair<LaneId, double>> lanes_distances{{LaneId::CenterLane, MAX_VALUE}, {LaneId::LeftLane, MAX_VALUE}, {LaneId::RightLane, MAX_VALUE}};
        if (center_lane_exist_) {
            lanes_distances[LaneId::CenterLane] = center_lane_.calculateDistanceFromPosition(vehicle.state_.position_);
        }
        if (left_lane_exist_) {
            lanes_distances[LaneId::LeftLane] = left_lane_.calculateDistanceFromPosition(vehicle.state_.position_);
        }
        if (right_lane_exist_) {
            lane_distances[LaneId::RightLane] = right_lane_.calculateDistanceFromPosition(vehicle.state_.position_);
        }

        std::sort(lanes_distances.begin(), lanes_distances.end(), [&] (const std::pair<LaneId, double>& a, const std::pair<LaneId, double>& b) {return a.second < b.second;});
        assert(lanes_distances[0].second < MAX_VALUE);

        // Judge whether excess the tolerance
        // TODO: add logic to handle this situation
        if (lanes_distances[0].second > 5.0) {
            return LaneId::Undefined;
        }

        return lanes_distances[0].first;
    }

    // Calculate reference lane for a vehicle
    LaneId calculateReferenceLaneId(const LaneId& nearest_lane_id, const LateralBehavior& lat_beh) {
        if (nearest_lane_id == LaneId::CenterLane) {
            if (lat_beh == LateralBehavior::LaneKeeping) {
                return LaneId::CenterLane;
            } else if (lat_beh == LateralBehavior::LaneChangeLeft) {
                return LaneId::LeftLane;
            } else if (lat_beh == LateralBehavior::LaneChangeRight) {
                return LaneId::RightLane;
            } else {
                assert(false);
            }
        } else if (nearest_lane_id == LaneId::LeftLane) {
            if (lat_beh == LateralBehavior::LaneKeeping) {
                return LaneId::LeftLane;
            } else if (lat_beh == LateralBehavior::LaneChangeLeft) {
                assert(false);
            } else if (lat_beh == LateralBehavior::LaneChangeRight) {
                return LaneId::CenterLane;
            } else {
                assert(false);
            }
        } else if (nearest_lane_id == LaneId::RightLane) {
            if (lat_beh == LateralBehavior::LaneKeeping) {
                return LaneId::RightLane;
            } else if (lat_beh == LateralBehavior::LaneChangeLeft) {
                return LaneId::CenterLane;
            } else if (lat_beh == LateralBehavior::LaneChangeRight) {
                assert(false);
            }
        } else {
            assert(false);
        }
        return LaneId::Undefined;
    }

    // Predict lateral behavior for surround vehicles based on vehicle information
    LateralBehavior predictSurroundVehicleLateralBehavior(const Vehicle& vehicle, const LaneId& nearest_lane_id) {
        // Get nearest lane
        Lane nearest_lane = lane_set_[nearest_lane_id];

        // Get frenet state
        StateTransformer stf(nearest_lane);
        FrenetState frenet_state = stf.getFrenetStateFromState(vehicle.state_);

        // Rule based judgement for potential lateral behavior
        LateralBehavior potential_lateral_behavior;
        const double lat_distance_threshold = 0.4;
        const double lat_vel_threshold = 0.35;
        if (frenet_state.vec_dt_[0] > lat_distance_threshold && frenet_state.vec_dt_[1] > lat_vel_threshold) {
            potential_lateral_behavior = LateralBehavior::LaneChangeLeft;
        } else if (frenet_state.vec_dt_[0] < -lat_distance_threshold && frenet_state.vec_dt_[1] < -lat_vel_threshold) {
            potential_lateral_behavior = LateralBehavior::LaneChangeRight;
        } else {
            potential_lateral_behavior = LateralBehavior::LaneKeeping;
        }

        // TODO: add logic to handle the situation where the vehicle on the left lane and generate the lane change left behavior. The same situation also occurs on the right lane. 
        if (potential_lateral_behavior == LateralBehavior::LaneChangeLeft && nearest_lane_id == LaneId::LeftLane) {
            potential_lateral_behavior = LateralBehavior::LaneKeeping;
        } else if (potential_lateral_behavior == LateralBehavior::LaneChangeRight && nearest_lane_id ==LaneId::RightLane) {
            potential_lateral_behavior = LateralBehavior::LaneKeeping;
        }

        return potential_lateral_behavior;

    }

    // Get semantic vehicle state (add lane information to vehicle state)
    SemanticVehicle getSurroundSemanticVehicle(const Vehicle& surround_vehicle) {
        // Calculate nearest lane
        LaneId nearest_lane_id = calculateNearestLaneId(surround_vehicle.state_.position_);
        Lane nearest_lane = lane_set_[nearest_lane_id];

        // Calculate potential behavior
        LateralBehavior potential_lateral_behavior = predictSurroundVehicleLateralBehavior(surround_vehicle, nearest_lane_id);

        // Calculate reference lane 
        LaneId reference_lane_id = calculateReferenceLaneId(nearest_lane_id, potential_lateral_behavior);
        Lane reference_lane = lane_set_[reference_lane_id];

        return SemanticVehicle(surround_vehicle, potential_lateral_behavior, LongitudinalBehavior::Normal, nearest_lane_id, nearest_lane, reference_lane_id, reference_lane);
    } 

    // Get ego semantic vehicle state
    // Note that ego vehicle's reference lane could be updated with the behavior sequence
    SemanticVehicle getEgoSemanticVehicle(const Vehicle& ego_vehicle, const VehicleBehavior& ego_vehicle_behavior) {
        // Calculate nearest lane
        LaneId nearest_lane_id = calculateNearestLaneId(ego_vehicle.state_.position_);
        Lane nearest_lane = lane_set_[nearest_lane_id];

        // Transform vehicle behavior to lateral behavior and longitudinal behavior
        LateralBehavior lat_beh = ego_vehicle_behavior.lat_beh_;
        LongitudinalBehavior lon_beh = ego_vehicle_behavior.lon_beh_;

        // Update reference lane
        LaneId reference_lane_id = calculateReferenceLaneId(nearest_lane_id, lat_beh);
        Lane reference_lane = lane_set_[reference_lane_id];

        return SemanticVehicle(ego_vehicle, lat_beh, lon_beh, nearest_lane_id, nearest_lane, reference_lane_id, reference_lane);
    }

    // Get leading vehicle state
    bool getLeadingVehicle(const SemanticVehicle& cur_semantic_vehicle, const std::unordered_map<int, SemanticVehicle>& other_semantic_vehicles_set, Vehicle& leading_vehicle) {
        // Initialize 
        Vehicle leading_veh;        
        int min_diff_index{MAX_VALUE};
        bool leading_veh_existed{false};

        // Get reference lane id and reference lane
        LaneId ref_lane_id = cur_semantic_vehicle.reference_lane_id_;
        Lane ref_lane = cur_semantic_vehicle.reference_lane_;

        // Calculate current vehicle index in reference lane
        size_t current_vehicle_index = ref_lane.findCurrenPositionIndexInLane(cur_semantic_vehicle.vehicle_.state_.position_(0), cur_semantic_vehicle.vehicle_.state_.position_(1));

        // Traverse vehicles
        for (const auto& sem_veh: other_semantic_vehicles_set) {
            if (sem_veh.first == cur_semantic_vehicle.vehicle_.id_) {
                continue;
            }

            if(sem_veh.second.nearest_lane_id_ == cur_semantic_vehicle.reference_lane_id_) {
                // Calculate this vehicle index in reference lane (its nearest lane)
                size_t this_vehicle_index = ref_lane.findCurrenPositionIndexInLane(sem_veh.second.vehicle_.state_.position_(0), sem_veh.second.vehicle_.state_.position_(1));

                if (static_cast<int>(this_vehicle_index) > static_cast<int>(current_vehicle_index) && ((this_vehicle_index - current_vehicle_index) < min_diff_index)) {
                    min_diff_index = static_cast<int>(this_vehicle_index) - static_cast<int>(current_vehicle_index);
                    leading_veh = sem_veh.second.vehicle_;
                    leading_veh_existed = true;
                }
            }
        }

        leading_vehicle = leading_veh;
        return leading_veh_existed;
    } 

    // Lane information in map interface
    bool center_lane_exist_{false};
    bool left_lane_exist_{false};
    bool right_lane_exist_{false};
    std::unordered_map<LaneId, Lane> lane_set_;
};

class IDM {
    static double calculateAcceleration(double cur_s, double leading_s, double cur_velocity, double leading_velocity, double desired_velocity) {
        double a_free = cur_velocity <= desired_velocity ? acceleration_ * (1 - pow(cur_velocity / (desired_velocity + EPS), exponent_)) : -comfortable_braking_deceleration_ * (1 - pow(desired_velocity / (cur_velocity + EPS), acceleration_ * exponent_ / comfortable_braking_deceleration_));

        double s_alpha = std::max(0.0 + EPS, leading_s - cur_s - vehicle_length_)
        double z = (minimum_spacing_ + std::max(0.0, cur_velocity * desired_headaway_time_ + cur_velocity * (cur_velocity - leading_velocity) / (2.0 * sqrt(acceleration_ * comfortable_braking_deceleration_)))) / s_alpha;

        // Calculate output acceleration
        double a_out;
        if (cur_velocity <= desired_velocity) {
            a_out = z >= 1.0 ? acceleration_ * (1 - pow(z, 2)) : a_free * (1.0 - pow(z, 2.0 * acceleration_ / (a_free + EPS)));
        } else {
            a_out = z >= 0.0 ? a_free + acceleration_ * (1 - pow(z, 2)) : a_free;
        }
        a_out = std::max(std::min(acceleration_, a_out), -hard_braking_deceleration_);

        return a_out;
    }

    static double calculateVelocity(double input_cur_s, double input_leading_s, double input_cur_velocity, double input_leading_velocity, double dt, double desired_velocity) {
        
        // Define linear predict function
        // TODO: use odeint to speed up calculation consumption  
        std::function <std::vector<double> (std::vector<double>, double)> linearPredict = [&](const std::vector<double>& current_states, double t_gap) {
            double cur_s = current_states[0], leading_s = current_states[1], cur_velocity = current_states[2], leading_velocity = current_states[3];

            double acc = calculateAcceleration(cur_s, leading_s, cur_velocity, leading_velocity, desired_velocity);
            acc = std::max(acc, -std::min(hard_braking_deceleration_, cur_velocity / t_gap));
            double next_cur_s = cur_s + cur_velocity * t_gap + 0.5 * acc * t_gap * t_gap;
            double next_leading_s = leading_s + leading_velocity * t_gap;
            double next_cur_velocity = cur_velocity + acc * t_gap;
            double next_leading_velocity = leading_velocity;

            std::vector<double> next_states{next_cur_s, next_leading_s, next_cur_velocity, next_leading_velocity};

            return next_states;
        };

        // States cache
        std::vector<double> predicted_states{input_cur_s, input_leading_s, input_cur_velocity, input_leading_velocity};

        int iteration_num = 40;
        for (int i = 0; i < iteration_num; i++) {
            predicted_states = linearPredict(predicted_states, dt / static_cast<double>(iteration_num));
        }

        return predicted_states[2];
    }

    double vehicle_length_{5.0};
    double minimum_spacing_{2.0};
    double desired_headaway_time_{1.0};
    double acceleration_{2.0};
    double comfortable_braking_deceleration_{3.0};
    double hard_braking_deceleration_{5.0};
    double exponent_{4};
};






} // End of namespace BehaviorPlanner


