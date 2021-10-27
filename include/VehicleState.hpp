/*
 * @Author: fujiawei0724
 * @Date: 2021-10-27 11:36:32
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-10-27 14:51:00
 * @Descripttion: The class for EUDM behavior planner, such as the vehicle state and vehicle trajectory
 */

#pragma once

#include "Point.hpp"
#include "Point.hpp"
#include "Const.hpp"
#include "Compare.hpp"
#include "Tools.hpp"
#include "Lane.hpp"

namespace BehaviorPlanner{

// Lateral behavior
enum class LateralBehavior {
    LaneKeeping = 0,
    LaneChangeLeft,
    LaneChangeRight,
};

// Longitudinal behavior
enum class LongitudinalBehavior {
    Normal = 0,
    Aggressive,
    Conservative,
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

struct BehaviorSequence {

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
    

    int sequence_length_;
};


} // End of namespace BehaviorPlanner


