/*
 * @Author: fujiawei0724
 * @Date: 2021-10-27 11:36:32
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-10-28 11:25:40
 * @Descripttion: The class for EUDM behavior planner, such as the vehicle state and vehicle trajectory
 */

#pragma once

#include <unordered_map>
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
    State(double time_stamp, const PathPlanningUtilities::Point2f& position, double theta, double curvature, double velocity, double acceleration, double steer) {
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
    PathPlanningUtilities::Point2f position_;
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

    void load() {
        
    }

    Eigen::Matrix<double, 3, 1> vec_s_{Eigen::Matrix<double, 3, 1>::Zero()};
    Eigen::Matrix<double, 3, 1> vec_ds_{Eigen::Matrix<double, 3, 1>::Zero()};
    Eigen::Matrix<double, 3, 1> vec_dt_{Eigen::Matrix<double, 3, 1>::Zero()};

    
};

class Vehicle {
public:
    // Constructor
    Vehicle() = default;
    Vehicle() {

    }

    // Destructor
    ~Vehicle() {

    }
};

// Vehicle state with semantic information, such as current lane and reference lane
class SemanticVehicle {
public:
    // Constructor
    SemanticVehicle() = default;
    SemanticVehicle(Vehicle* vehicle) {
        vehicle_ = vehicle;
    }

    // Destructor
    ~SemanticVehicle() {
        
    }

    VehicleStateWorld* vehicle_;
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
            center_lane_ = lane_info[LaneId::CenterLane];
        }
        if (lane_exist[LaneId::LeftLane]) {
            left_lane_exist_ = true;
            left_lane_ = lane_info[LaneId::LeftLane];
        }
        if (lane_exist[LaneId::RightLane]) {
            right_lane_exist_ = true;
            right_lane_ = lane_info[LaneId::RightLane];
        }
    }

    // Destructor
    ~MapInterface() {

    }

    // Get semantic vehicle state (add lane information to vehicle state)
    SemanticVehicleState getSemanticVehicle() {
        
    } 

    // Get leading vehicle state
    SemanticVehicleState getLeadingSemanticVehicle() {

    }

    // Lane information in map interface
    bool center_lane_exist_{false};
    bool left_lane_exist_{false};
    bool right_lane_exist_{false};
    Lane center_lane_;
    Lane left_lane_;
    Lane right_lane_;
};


} // End of namespace BehaviorPlanner


