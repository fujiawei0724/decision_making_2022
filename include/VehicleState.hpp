/*
 * @Author: fujiawei0724
 * @Date: 2021-10-27 11:36:32
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-10-27 20:36:13
 * @Descripttion: The class for EUDM behavior planner, such as the vehicle state and vehicle trajectory
 */

#pragma once

#include <unordered_map>
#include "Point.hpp"
#include "Point.hpp"
#include "Const.hpp"
#include "Compare.hpp"
#include "Tools.hpp"
#include "Lane.hpp"

namespace BehaviorPlanner{

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


// The description of vehicle in world
class VehicleStateWorld {
public: 
    // Constructor
    VehicleStateWorld() = default;
    VehicleStateWorld() {
        
    }

    // Destructor
    ~VehicleStateWorld() {

    }

    
};

// Vehicle state with semantic information, such as current lane and reference lane
class SemanticVehicleState {
public:
    // Constructor
    SemanticVehicleState() = default;
    SemanticVehicleState(VehicleStateWorld* vehicle_state_world) {
        vehicle_state_world_ = vehicle_state_world;
    }

    // Destructor
    ~SemanticVehicleState() {
        
    }

    VehicleStateWorld* vehicle_state_world_;
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


