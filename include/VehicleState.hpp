/*
 * @Author: fujiawei0724
 * @Date: 2021-10-27 11:36:32
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-11-08 18:57:55
 * @Descripttion: The description of vehicle in different coordinations. 
 */

#pragma once

#include <unordered_map>
#include <thread>
#include <map>
#include <algorithm>
#include <functional>
#include <stdio.h>
#include "Const.hpp"
#include "Point.hpp"
#include "Compare.hpp"
#include "Tools.hpp"
#include "Lane.hpp"
#include "Rectangle.hpp"
#include "Obstacle.hpp"

namespace Common {

// Lane id  
enum class LaneId {
    CenterLane = 0,
    LeftLane, 
    RightLane,
    Undefined,
};

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
        printf("State position x: %lf\n", position_(0));
        printf("State position y: %lf\n", position_(1));
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
        size_t start_index_of_lane = lane_.findCurrenPositionIndexInLane(state.position_);
        PathPlanningUtilities::CoordinationPoint nearest_lane_point = lane_coordination[start_index_of_lane];
        Eigen::Matrix<double, 2, 1> lane_position{nearest_lane_point.worldpos_.position_.x_, nearest_lane_point.worldpos_.position_.y_};

        // Get curvature and curvature derivative information
        // TODO: load the curvature derivative information from map server
        double curvature = nearest_lane_point.worldpos_.kappa_;
        double curvature_derivative = 0.0;

        // Get the arc length
        std::vector<TransMatrix> trans_matrixes = lane_.getLaneTransMatrix();
        TransMatrix trans_matrix = trans_matrixes[start_index_of_lane];
        Eigen::Vector2d start_point_in_world_v2d(state.position_(0), state.position_(1));
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

        double spp = (state.acceleration_ - pow(sp, 2) / cn_delta_theta * (one_minus_curd * tan_delta_theta * delta_theta_derivative - (curvature_derivative * d + curvature * ds))) * cn_delta_theta / one_minus_curd;

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
            printf("[StateTransformer] one_minus_curd not valid for transform.\n");
        }

        // Get tangent vector and normal vector, the norm is set with 1.0
        double lane_orientation = lane_position.worldpos_.theta_;
        double y = tan(lane_orientation);
        double x = 1.0;
        Eigen::Matrix<double, 2, 1> lane_tangent_vec{x, y};
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
        generateRectangle();
    }

    // Destructor
    ~Vehicle() {

    }

    // Generate rectangle
    void generateRectangle() {
        rectangle_ = Rectangle(state_.position_(0), state_.position_(1), state_.theta_, width_, length_);
    }

    int id_{0};
    State state_;
    double length_{0.0};
    double width_{0.0};
    Rectangle rectangle_;
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
    LaneId nearest_lane_id_{LaneId::Undefined};
    LaneId reference_lane_id_{LaneId::Undefined};
    Lane nearest_lane_;
    Lane reference_lane_;
};









} // End of namespace BehaviorPlanner


