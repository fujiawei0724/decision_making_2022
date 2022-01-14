/*
 * @Author: fujiawei0724
 * @Date: 2021-12-01 21:10:42
 * @LastEditTime: 2022-01-14 16:23:21
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

    MapInterface::MapInterface(const std::map<LaneId, bool>& lane_exist, const std::map<LaneId, Lane>& lane_info) {
        // Initialize lane information
        if (lane_exist.at(LaneId::CenterLane)) {
            center_lane_exist_ = true;
        }
        if (lane_exist.at(LaneId::LeftLane)) {
            left_lane_exist_ = true;
        }
        if (lane_exist.at(LaneId::RightLane)) {
            right_lane_exist_ = true;
        }
        lane_set_ = lane_info;
    }
    MapInterface::~MapInterface() {

    }

    LaneId MapInterface::calculateNearestLaneId(const Vehicle& vehicle) {
        return calculateNearestLaneId(vehicle.state_.position_);
    }

    LaneId MapInterface::calculateNearestLaneId(const Eigen::Matrix<double, 2, 1>& position) {
        // Calculate distance to each lane
        std::vector<std::pair<LaneId, double>> lanes_distances{{LaneId::CenterLane, MAX_VALUE}, {LaneId::LeftLane, MAX_VALUE}, {LaneId::RightLane, MAX_VALUE}};
        if (center_lane_exist_) {
            lanes_distances[static_cast<int>(LaneId::CenterLane)].second = lane_set_[LaneId::CenterLane].calculateDistanceFromPosition(position);
        }
        if (left_lane_exist_) {
            lanes_distances[static_cast<int>(LaneId::LeftLane)].second = lane_set_[LaneId::LeftLane].calculateDistanceFromPosition(position);
        }
        if (right_lane_exist_) {
            lanes_distances[static_cast<int>(LaneId::RightLane)].second = lane_set_[LaneId::RightLane].calculateDistanceFromPosition(position);
        }

        std::sort(lanes_distances.begin(), lanes_distances.end(), [&] (const std::pair<LaneId, double>& a, const std::pair<LaneId, double>& b) {return a.second < b.second;});
        assert(lanes_distances[0].second < MAX_VALUE);

        // Judge whether excess the tolerance
        // TODO: add logic to handle this situation
        // if (lanes_distances[0].second > 5.0) {
        //     return LaneId::Undefined;
        // }

        return lanes_distances[0].first;
    }

    LaneId MapInterface::calculateReferenceLaneId(const LaneId& nearest_lane_id, const LateralBehavior& lat_beh) {
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
                // printf("[BehaviorPlanner]: keeping lang change left.\n");
                if (left_lane_exist_) {
                    return LaneId::LeftLane;
                } else {
                    return LaneId::CenterLane;
                }
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
                // printf("[BehaviorPlanner]: keeping lang change right.\n");
                if (right_lane_exist_) {
                    return LaneId::RightLane;
                } else {
                    return LaneId::CenterLane;
                }
            } else {
                assert(false);
            }
        } else {
            assert(false);
        }
        return LaneId::Undefined;
    }

    LateralBehavior MapInterface::predictSurroundVehicleLateralBehavior(const Vehicle& vehicle, const LaneId& nearest_lane_id) {
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

    SemanticVehicle MapInterface::getSingleSurroundSemanticVehicle(const Vehicle& surround_vehicle) {
        // Calculate nearest lane
        LaneId nearest_lane_id = calculateNearestLaneId(surround_vehicle.state_.position_);
        Lane nearest_lane = lane_set_[nearest_lane_id];

        // Calculate potential behavior
        LateralBehavior potential_lateral_behavior = predictSurroundVehicleLateralBehavior(surround_vehicle, nearest_lane_id);

        // Calculate reference lane 
        LaneId reference_lane_id = calculateReferenceLaneId(nearest_lane_id, potential_lateral_behavior);
        Lane reference_lane = lane_set_[reference_lane_id];

        return SemanticVehicle(surround_vehicle, nearest_lane_id, reference_lane_id, nearest_lane, reference_lane);
    }

    std::unordered_map<int, SemanticVehicle> MapInterface::getSurroundSemanticVehicles(const std::unordered_map<int, Vehicle>& surround_vehicles) {
        std::unordered_map<int, SemanticVehicle> surround_semantic_vehicles;
        for (const auto& sur_veh: surround_vehicles) {
            surround_semantic_vehicles[sur_veh.first] = getSingleSurroundSemanticVehicle(sur_veh.second);
        }
        return surround_semantic_vehicles;
    }

    // Get ego semantic vehicle state
    // Note that ego vehicle's reference lane could be updated with the behavior sequence
    SemanticVehicle MapInterface::getEgoSemanticVehicle(const Vehicle& ego_vehicle, const LateralBehavior& ego_vehicle_lat_beh) {
        // Calculate nearest lane
        LaneId nearest_lane_id = calculateNearestLaneId(ego_vehicle.state_.position_);
        Lane nearest_lane = lane_set_[nearest_lane_id];

        // Update reference lane
        LaneId reference_lane_id = calculateReferenceLaneId(nearest_lane_id, ego_vehicle_lat_beh);
        Lane reference_lane;
        if (lane_set_[reference_lane_id].lane_existance_) {
            reference_lane = lane_set_[reference_lane_id];
        } else {
            reference_lane = nearest_lane;
            reference_lane_id = nearest_lane_id;
        }

        return SemanticVehicle(ego_vehicle, nearest_lane_id, reference_lane_id, nearest_lane, reference_lane);
    }

    // Get leading vehicle state
    bool MapInterface::getLeadingVehicle(const SemanticVehicle& cur_semantic_vehicle, const std::unordered_map<int, SemanticVehicle>& other_semantic_vehicles_set, Vehicle& leading_vehicle) {
        // Initialize 
        Vehicle leading_veh;        
        int min_diff_index = static_cast<int>(MAX_VALUE);
        bool leading_veh_existed{false};

        // Get reference lane id and reference lane
        // LaneId ref_lane_id = cur_semantic_vehicle.reference_lane_id_;
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

                if (static_cast<int>(this_vehicle_index) > static_cast<int>(current_vehicle_index) && ((static_cast<int>(this_vehicle_index) - static_cast<int>(current_vehicle_index)) < min_diff_index)) {
                    min_diff_index = static_cast<int>(this_vehicle_index) - static_cast<int>(current_vehicle_index);
                    leading_veh = sem_veh.second.vehicle_;
                    leading_veh_existed = true;
                }
            }
        }

        leading_vehicle = leading_veh;
        return leading_veh_existed;
    }

    // Calculate the speed limit given a vehicle state
    double MapInterface::calculateSpeedLimit(const Vehicle& vehicle) {
        // Calculate nearest lane and its speed limit
        LaneId nearest_lane_id = calculateNearestLaneId(vehicle);
        Lane nearest_lane = lane_set_[nearest_lane_id];
        std::vector<double> speed_limits = nearest_lane.getLaneVelocityLimitation();
        
        // Calculate the point index in the lane 
        size_t point_index = nearest_lane.findCurrenPositionIndexInLane(vehicle.state_.position_);

        return speed_limits[point_index];
    }

    // Calculate point in lane
    bool MapInterface::isInLane(const PathPlanningUtilities::Point2f& position) {
        if (center_lane_exist_) {
            if (lane_set_[LaneId::CenterLane].isInLane(position)) {
                return true;
            }
        }
        if (left_lane_exist_) {
            if (lane_set_[LaneId::LeftLane].isInLane(position)) {
                return true;
            }
        }
        if (right_lane_exist_) {
            if (lane_set_[LaneId::RightLane].isInLane(position)) {
                return true;
            }
        }
        return false;
    }

    // Calculate the orientation of a lane point that has the minimum distance from the specified position
    double MapInterface::calculateNearestPointOrientation(const PathPlanningUtilities::Point2f& position) {
        // Calculate the nearest lane
        Eigen::Matrix<double, 2, 1> point_position{position.x_, position.y_};
        LaneId nearest_lane_id = calculateNearestLaneId(point_position);
        Lane nearest_lane = lane_set_[nearest_lane_id];

        // Calculate the nearest lane point from the position
        size_t nearest_lane_point_index = nearest_lane.findCurrenPositionIndexInLane(point_position);

        double orientation = nearest_lane.getLaneCoordnation()[nearest_lane_point_index].worldpos_.theta_;

        return orientation;
    }

    /**
     * @brief Generate lane information for HPDM 
     * @param {*}
     * @return {*}
     */   
    std::vector<double> MapInterface::getLaneInfo() {
        // Encode left and right lane existence and gap information
        std::vector<double> lane_info(5, 0.0);
        PathPlanningUtilities::Point2f center_lane_start_point = lane_set_[LaneId::CenterLane].getLaneCenterPathInWorld()[0]; 
        if (left_lane_exist_) {
            lane_info[0] = 1.0;
            PathPlanningUtilities::Point2f left_lane_start_point = lane_set_[LaneId::LeftLane].getLaneCenterPathInWorld()[0];
            lane_info[2] = (center_lane_start_point - left_lane_start_point).norm();
        }
        if (right_lane_exist_) {
            lane_info[1] = 1.0;
            PathPlanningUtilities::Point2f right_lane_start_point = lane_set_[LaneId::RightLane].getLaneCenterPathInWorld()[0];
            lane_info[3] = (center_lane_start_point - right_lane_start_point).norm();
        }

        // Calculate speed limit 
        // TODO: maybe need consider the speed limit in different position in lane
        double speed_limit = lane_set_[LaneId::CenterLane].getLaneVelocityLimitation()[0];
        lane_info[4] = speed_limit;

        return lane_info;
    }

    /**
     * @brief Calculate nearest lane for HPDM
     * @param {*}
     * @return {*}
     */
    Lane MapInterface::calculateNearestLane(const Vehicle& vehicle) {
        return lane_set_[calculateNearestLaneId(vehicle)];
    }
    
    IDM::IDM() = default;

    IDM::~IDM() = default;


    double IDM::calculateAcceleration(double cur_s, double leading_s, double cur_velocity, double leading_velocity, double desired_velocity) {
        double a_free = cur_velocity <= (desired_velocity + MIDDLE_EPS) ? acceleration_ * (1 - pow(cur_velocity / (desired_velocity + SMALL_EPS), exponent_)) : -comfortable_braking_deceleration_ * (1 - pow(desired_velocity / (cur_velocity + SMALL_EPS), acceleration_ * exponent_ / comfortable_braking_deceleration_));

        double s_alpha = std::max(0.0 + SMALL_EPS, leading_s - cur_s - vehicle_length_);
        double z = (minimum_spacing_ + std::max(0.0, cur_velocity * desired_headaway_time_ + cur_velocity * (cur_velocity - leading_velocity) / (2.0 * sqrt(acceleration_ * comfortable_braking_deceleration_)))) / s_alpha;

        // Calculate output acceleration
        double a_out;
        if (cur_velocity <= desired_velocity + MIDDLE_EPS) {
            a_out = z >= 1.0 ? acceleration_ * (1 - pow(z, 2)) : a_free * (1.0 - pow(z, 2.0 * acceleration_ / (a_free + SMALL_EPS)));
        } else {
            a_out = z >= 0.0 ? a_free + acceleration_ * (1 - pow(z, 2)) : a_free;
        }
        a_out = std::max(std::min(acceleration_, a_out), -hard_braking_deceleration_);

        return a_out;
    }

    double IDM::calculateVelocity(double input_cur_s, double input_leading_s, double input_cur_velocity, double input_leading_velocity, double dt, double desired_velocity) {
        
        // // Define linear predict function
        // // TODO: use odeint to speed up calculation consumption  
        // std::function<std::vector<double> (std::vector<double>, double)> linearPredict = [&](const std::vector<double>& current_states, double t_gap) {
        //     double cur_s = current_states[0], leading_s = current_states[1], cur_velocity = current_states[2], leading_velocity = current_states[3];

        //     double acc = calculateAcceleration(cur_s, leading_s, cur_velocity, leading_velocity, desired_velocity);
        //     acc = std::max(acc, -std::min(hard_braking_deceleration_, cur_velocity / t_gap));
        //     double next_cur_s = cur_s + cur_velocity * t_gap + 0.5 * acc * t_gap * t_gap;
        //     double next_leading_s = leading_s + leading_velocity * t_gap;
        //     double next_cur_velocity = cur_velocity + acc * t_gap;
        //     double next_leading_velocity = leading_velocity;

        //     std::vector<double> next_states{next_cur_s, next_leading_s, next_cur_velocity, next_leading_velocity};

        //     return next_states;
        // };

        internal_state_[0] = input_cur_s;
        internal_state_[1] = input_leading_s;
        internal_state_[2] = input_cur_velocity;
        internal_state_[3] = input_leading_velocity;
        desired_velocity_ = desired_velocity;

        boost::numeric::odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);


        // // States cache
        // std::vector<double> predicted_states{input_cur_s, input_leading_s, input_cur_velocity, input_leading_velocity};

        // int iteration_num = 40;
        // for (int i = 0; i < iteration_num; i++) {
        //     predicted_states = linearPredict(predicted_states, dt / static_cast<double>(iteration_num));
        // }

        return internal_state_[2];
    }


    // For odeint use
    void IDM::operator()(const InternalState &x, InternalState &dxdt, const double dt) {
        double input_cur_s = x[0];
        double input_leading_s = x[1];
        double input_cur_velocity = x[2];
        double input_leading_velocity = x[3];

        double acc = calculateAcceleration(input_cur_s, input_leading_s, input_cur_velocity, input_leading_velocity, desired_velocity_);
        acc = std::max(acc, -std::min(hard_braking_deceleration_, input_cur_s / dt));
        dxdt[0] = input_cur_velocity;
        dxdt[1] = input_leading_velocity;
        dxdt[2] = acc;
        dxdt[3] = 0.0;  // assume other vehicle keep the current velocity
    }



    constexpr double IDM::vehicle_length_;
    constexpr double IDM::minimum_spacing_;
    constexpr double IDM::desired_headaway_time_;
    constexpr double IDM::acceleration_;
    constexpr double IDM::comfortable_braking_deceleration_;
    constexpr double IDM::hard_braking_deceleration_;
    constexpr double IDM::exponent_;

    // Constructor
    IdealSteerModel::IdealSteerModel(double wheelbase_len, double max_lon_acc, double max_lon_dec, double max_lon_acc_jerk, double max_lon_dec_jerk, double max_lat_acc, double max_lat_jerk, double max_steering_angle, double max_steer_rate, double max_curvature) {
        // Load parameters
        wheelbase_len_ = wheelbase_len;
        max_lon_acc_ = max_lon_acc;
        max_lon_dec_ = max_lon_dec;
        max_lon_acc_jerk_ = max_lon_acc_jerk;
        max_lon_dec_jerk_ = max_lon_dec_jerk;
        max_lat_acc_ = max_lat_acc;
        max_lat_jerk_ = max_lat_jerk;
        max_steering_angle_ = max_steering_angle;
        max_steer_rate_ = max_steer_rate;
        max_curvature_ = max_curvature;
    }
    // Destructor
    IdealSteerModel::~IdealSteerModel() {
        
    }

    void IdealSteerModel::setControl(const std::pair<double, double>& control) {
        control_ = control;
    }

    void IdealSteerModel::setState(const State& state) {
        state_ = state;
    }

    void IdealSteerModel::truncateControl(double dt) {
        desired_lon_acc_ = (control_.second - state_.velocity_) / dt;
        double desired_lon_jerk = (desired_lon_acc_ - state_.acceleration_) / dt;
        desired_lon_jerk = Tools::truncate(desired_lon_jerk, -max_lon_dec_jerk_, max_lon_acc_jerk_);
        desired_lon_acc_ = desired_lon_jerk * dt + state_.acceleration_;
        desired_lon_acc_ = Tools::truncate(desired_lon_acc_, -max_lon_dec_, max_lon_acc_);
        control_.second = std::max(state_.velocity_ + desired_lon_acc_ * dt, 0.0);
        desired_lat_acc_ = pow(control_.second, 2) * (tan(control_.first) / wheelbase_len_);
        double lat_acc_ori = pow(state_.velocity_, 2.0) * state_.curvature_;
        double lat_jerk_desired = (desired_lat_acc_ - lat_acc_ori) / dt;
        lat_jerk_desired = Tools::truncate(lat_jerk_desired, -max_lat_jerk_, max_lat_jerk_);
        desired_lat_acc_ = lat_jerk_desired * dt + lat_acc_ori;
        desired_lat_acc_ = Tools::truncate(desired_lat_acc_, -max_lat_acc_, max_lat_acc_);
        control_.first = atan(desired_lat_acc_ * wheelbase_len_ / std::max(pow(control_.second, 2.0), 0.1 * BIG_EPS));
        desired_steer_rate_ = Tools::safeThetaTransform(control_.first - state_.steer_) / dt;
        desired_steer_rate_ = Tools::truncate(desired_steer_rate_, -max_steer_rate_, max_steer_rate_);
        control_.first = Tools::safeThetaTransform(state_.steer_ + desired_steer_rate_ * dt);
    }

    void IdealSteerModel::step(double dt) {

        state_.steer_ = atan(state_.curvature_ * wheelbase_len_);
        updateInternalState();
        control_.second = std::max(0.0, control_.second);
        control_.first = Tools::truncate(control_.first, -max_steering_angle_, max_steering_angle_);
        truncateControl(dt);
        desired_lon_acc_ = (control_.second - state_.velocity_) / dt;
        desired_steer_rate_ = Tools::safeThetaTransform(control_.first - state_.steer_);

        // // TODO: use odeint to speed up calculation consumption
        // std::function<Eigen::Matrix<double, 5, 1> (Eigen::Matrix<double, 5, 1>, double)> linearPredict = [&](const Eigen::Matrix<double, 5, 1>& cur_state, double t_gap) {
        //     Eigen::Matrix<double, 5, 1> predicted_state{Eigen::Matrix<double, 5, 1>::Zero()};
        //     predicted_state[0] = cur_state[0] + t_gap * cos(cur_state[2]) * cur_state[3];
        //     predicted_state[1] = cur_state[1] + t_gap * sin(cur_state[2]) * cur_state[3];
        //     predicted_state[2] = cur_state[2] + t_gap * tan(cur_state[4]) * cur_state[3] / wheelbase_len_;
        //     predicted_state[3] = cur_state[3] + t_gap * desired_lon_acc_;
        //     predicted_state[4] = cur_state[4] + t_gap * desired_steer_rate_;
            
        //     return predicted_state;
        // };
        
        // Eigen::Matrix<double, 5, 1> predict_state = internal_state_;
        // int iteration_num = 40;
        // for (int i = 0; i < iteration_num; i++) {
        //     predict_state = linearPredict(predict_state, dt / static_cast<double>(iteration_num));
        // }

        boost::numeric::odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);

        state_.position_(0) = internal_state_[0];
        state_.position_(1) = internal_state_[1];
        state_.theta_ = Tools::safeThetaTransform(internal_state_[2]);
        state_.velocity_ = internal_state_[3];
        state_.acceleration_ = desired_lon_acc_;
        state_.curvature_ = tan(internal_state_[4]) * 1.0 / wheelbase_len_;
        state_.steer_ = Tools::safeThetaTransform(internal_state_[4]);

        // Note that the time stamp of state is not update here, that value should be updated in the forward extender

        updateInternalState();
    }

    // Update internal state
    void IdealSteerModel::updateInternalState() {
        internal_state_[0] = state_.position_(0);
        internal_state_[1] = state_.position_(1);
        internal_state_[2] = state_.theta_;
        internal_state_[3] = state_.velocity_;
        internal_state_[4] = state_.steer_;
    }

    // For odeint use
    void IdealSteerModel::operator()(const InternalState &x, InternalState &dxdt, const double /* t */) {
        State cur_state;
        cur_state.position_(0) = x[0];
        cur_state.position_(1) = x[1];
        cur_state.theta_ = x[2];
        cur_state.velocity_ = x[3];
        cur_state.steer_ = x[4];

        dxdt[0] = cos(cur_state.theta_) * cur_state.velocity_;
        dxdt[1] = sin(cur_state.theta_) * cur_state.velocity_;
        dxdt[2] = tan(cur_state.steer_) * cur_state.velocity_ / wheelbase_len_;
        dxdt[3] = desired_lon_acc_;
        dxdt[4] = desired_steer_rate_;
    }

    // Calculate vehicle steer 
    double ForwardExtender::calculateSteer(SemanticVehicle* semantic_vehicle) {
        // Determine look ahead distance in reference lane
        double look_ahead_distance = std::min(std::max(BehaviorPlannerConfig::look_ahead_min_distance, semantic_vehicle->vehicle_.state_.velocity_ * BehaviorPlannerConfig::steer_control_gain), BehaviorPlannerConfig::look_ahead_max_distance);

        // Calculate target point based on look ahead distance
        Eigen::Matrix<double, 2, 1> target_point = semantic_vehicle->reference_lane_.calculateTargetLanePosition(semantic_vehicle->vehicle_.state_.position_, look_ahead_distance);

        // Calculate look ahead distance in world 
        double look_ahead_distance_world = (target_point - semantic_vehicle->vehicle_.state_.position_).norm();

        // Calculate target angle and diff angle
        double target_angle = atan2(target_point(1) - semantic_vehicle->vehicle_.state_.position_(1), target_point(0) - semantic_vehicle->vehicle_.state_.position_(0));
        double diff_angle = Tools::safeThetaTransform(target_angle - semantic_vehicle->vehicle_.state_.theta_);

        // Calculate target steer 
        double target_steer = Tools::calculateSteer(BehaviorPlannerConfig::wheelbase_length, diff_angle, look_ahead_distance_world);

        return target_steer;        
    }

    // Calculate vehicle velocity 
    double ForwardExtender::calculateVelocity(MapInterface* mtf, const SemanticVehicle& cur_semantic_vehicle, const std::unordered_map<int, SemanticVehicle>& semantic_vehicles, double dt, double desired_velocity) {
        // Calculate leading vehicle
        Vehicle leading_vehicle;
        bool leading_vehicle_exist = mtf->getLeadingVehicle(cur_semantic_vehicle, semantic_vehicles, leading_vehicle);
        IDM idm;

        double target_velocity = 0.0;
        if (!leading_vehicle_exist) {
            // Don't exist leading vehicle
            double virtual_leading_vehicle_distance = 100.0 + 100.0 * cur_semantic_vehicle.vehicle_.state_.velocity_;
            target_velocity = idm.calculateVelocity(0.0, virtual_leading_vehicle_distance, cur_semantic_vehicle.vehicle_.state_.velocity_, cur_semantic_vehicle.vehicle_.state_.velocity_, dt, desired_velocity);
        } else {
            // The distance between leading vehicle and ego vehicle could also be calculated in frenet frame
            // With leading vehicle
            Lane corresponding_lane = cur_semantic_vehicle.reference_lane_;

            // Calculate cur vehicle index and leading vehicle index in corresponding lane
            size_t cur_vehicle_index = corresponding_lane.findCurrenPositionIndexInLane(cur_semantic_vehicle.vehicle_.state_.position_);
            size_t leading_vehicle_index = corresponding_lane.findCurrenPositionIndexInLane(leading_vehicle.state_.position_);

            // Calculate gap distance
            double leading_cur_distance = (static_cast<int>(leading_vehicle_index) - static_cast<int>(cur_vehicle_index)) * 0.1;

            target_velocity = idm.calculateVelocity(0.0, leading_cur_distance, cur_semantic_vehicle.vehicle_.state_.velocity_, leading_vehicle.state_.velocity_, dt, desired_velocity);
        }

        // std::cout << "Target velocity: " << target_velocity << std::endl;

        return target_velocity;
    }

    // Calculate desired vehicle stateuse steer and velocity based on ideal steer model
    Vehicle ForwardExtender::calculateDesiredVehicleState(const Vehicle& veh, const double& steer, const double& velocity, const double& dt) {
        // Load parameters for ideal steer model
        using Config = BehaviorPlannerConfig;
        IdealSteerModel ideal_steer_model(Config::wheelbase_length, IDM::acceleration_, IDM::hard_braking_deceleration_, Config::max_lon_acc_jerk, Config::max_lon_brake_jerk, Config::max_lat_acceleration_abs, Config::max_lat_jerk_abs, Config::max_steer_angle_abs, Config::max_steer_rate, Config::max_curvature_abs);
        ideal_steer_model.setState(veh.state_);
        ideal_steer_model.setControl(std::make_pair(steer, velocity));
        ideal_steer_model.step(dt);

        // Calculate preicted state and predict vehicle information
        State predicted_state = ideal_steer_model.state_;
        predicted_state.time_stamp_ = veh.state_.time_stamp_ + dt;
        Vehicle predicted_vehicle_state(veh.id_, predicted_state, veh.length_, veh.width_);

        return predicted_vehicle_state;
    }


    /**
     * @introduction: Forward one step, this function is for all vehicle agents, not only for ego vehicle
     * @param semantic_vehicles denotes the all vehicles (include the ego vehicle)
     * @return current vehicle's extended state
     */   
    // TODO: add detailed logic for ego vehicle state update, which means a different calculation method with surround vehicles 
    Vehicle ForwardExtender::extendState(MapInterface* mtf, std::unordered_map<int, SemanticVehicle>& semantic_vehicles, int current_vehicle_id, double dt, double desired_velocity) {
        // Load current semantic vehicle 
        SemanticVehicle cur_semantic_vehicle = semantic_vehicles[current_vehicle_id];

        // Calculate steer 
        double steer = calculateSteer(&cur_semantic_vehicle);

        // Calculate velocity
        double velocity = calculateVelocity(mtf, cur_semantic_vehicle, semantic_vehicles, dt, desired_velocity);

        Vehicle desired_vehicle_state = calculateDesiredVehicleState(cur_semantic_vehicle.vehicle_, steer, velocity, dt);

        return desired_vehicle_state;
    }

    // Constructor
    BehaviorGenerator::BehaviorGenerator(MapInterface* mtf, int sequence_length) {
        is_lane_keeping_available_ = mtf->center_lane_exist_;
        is_lane_change_left_available_ = mtf->left_lane_exist_;
        is_lane_change_right_available_ = mtf->right_lane_exist_;
        sequence_length_ = sequence_length;
    }

    // Destructor
    BehaviorGenerator::~BehaviorGenerator() {

    }

    // Generate vehicle behavior sequence
    std::vector<std::vector<VehicleBehavior>> BehaviorGenerator::generateBehaviorSequence() {
        // Initialize result
        std::vector<std::vector<VehicleBehavior>> veh_beh_seq;

        // Traverse all behavior possibility
        for (int lon = 0; lon < static_cast<int>(LongitudinalBehavior::MaxCount); lon++) {
            std::vector<VehicleBehavior> cur_beh_seq;
            for (int beh_index = 0; beh_index < sequence_length_; beh_index++) {
                for (int lat = 0; lat < static_cast<int>(LateralBehavior::MaxCount); lat++) {
                    // Shield the situation where the corresponding lane doesn't exist 
                    if (!is_lane_keeping_available_ && LateralBehavior(lat) == LateralBehavior::LaneKeeping) {
                        continue;
                    }
                    if (!is_lane_change_left_available_ && LateralBehavior(lat) == LateralBehavior::LaneChangeLeft) {
                        continue;
                    }
                    if (!is_lane_change_right_available_ && LateralBehavior(lat) == LateralBehavior::LaneChangeRight) {
                        continue;
                    }

                    // Shield lane keeping situation
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
    std::vector<VehicleBehavior> BehaviorGenerator::completeBehaviorSequence(const std::vector<VehicleBehavior>& cur_beh_seq, LateralBehavior lat_beh, LongitudinalBehavior lon_beh, int num) {
        // Initialize
        std::vector<VehicleBehavior> completed_beh_seq = cur_beh_seq;

        // Add lane change behavior 
        for (int i = 0; i < num; i++) {
            completed_beh_seq.emplace_back(VehicleBehavior(lat_beh, lon_beh));
        }

        return completed_beh_seq;
    }

    // Calculate all costs
    // TODO: add safety cost consider the collision and collision velocity
    double PolicyEvaluater::calculateCost(const Trajectory& ego_trajectory, const std::unordered_map<int, Trajectory>& surround_trajectories, const bool& is_lane_changed, const double& lane_speed_limit) {
        double cost = calculateActionCost(is_lane_changed) + calculateEfficiencyCost(ego_trajectory, lane_speed_limit);
        return cost;
    }

    // Calculate lane change action cost
    double PolicyEvaluater::calculateActionCost(const bool& is_lane_changed) {
        double action_cost = 0.0;
        if (is_lane_changed) {
            action_cost = 0.3;
        }
        return action_cost;
    }

    // Calculate efficiency cost, due to ego current velocity and lane limit velocity
    double PolicyEvaluater::calculateEfficiencyCost(const Trajectory& ego_trajectory, const double& speed_limit) {
        Vehicle ego_vehicle_last_state = ego_trajectory.back();

        double efficiency_cost = 0.0;
        if (speed_limit >= ego_vehicle_last_state.state_.velocity_) {
            efficiency_cost = (speed_limit - ego_vehicle_last_state.state_.velocity_) / 10.0;
        } else{
            // Velocity excess the lane limitation
            // Note that these situations are labeled as unsafe, which isn't handled there
        }
        return efficiency_cost;
    }

    // Calculate consistence cost
    double PolicyEvaluater::calculateConsistenceCost(const Trajectory& ego_trajectory, const Lane& pre_reference_lane, const Vehicle& pre_ego_desired_vehicle_state) {
        double consistence_cost = 0.0;
        
        // TODO: adjust parameters in detailed
        // Calculate velocities gap between current desired state and previous desired state
        Vehicle current_desired_state = ego_trajectory.back();
        double velocities_gap = fabs(current_desired_state.state_.velocity_ - pre_ego_desired_vehicle_state.state_.velocity_);
        if (velocities_gap > 4.0) {
            consistence_cost += 0.2;
        }

        // Calculate the deviance between current desired state and previous reference lane
        int corresponding_index = pre_reference_lane.findCurrenPositionIndexInLane(current_desired_state.state_.position_);
        Eigen::Matrix<double, 2, 1> corresponding_point{pre_reference_lane.getLaneCenterPathInWorld()[corresponding_index].x_, pre_reference_lane.getLaneCenterPathInWorld()[corresponding_index].y_};
        double distance = (current_desired_state.state_.position_ - corresponding_point).norm();
        consistence_cost += distance / 10.0;
        // if (consistence_cost > 0.2) {
        //     consistence_cost += 5.0;
        // }

        return consistence_cost;
    }

    // Judge is safe
    bool PolicyEvaluater::calculateSafe(const Trajectory& ego_trajectory, const std::unordered_map<int, Trajectory>& surround_trajectories, double speed_limit) {
        // Judge whether excess the lane speed limit
        Vehicle ego_vehicle_last_state = ego_trajectory.back();
        if (ego_vehicle_last_state.state_.velocity_ > speed_limit) {
            return false;
        }

        // Judge whether collision
        for (int time_stamp_index = 0; time_stamp_index < static_cast<int>(ego_trajectory.size()); time_stamp_index++) {
            Rectangle ego_vehicle_rec = ego_trajectory[time_stamp_index].rectangle_;
            
            // Traverse surround vehicles
            for (const auto& sur_veh_traj_info: surround_trajectories) {
                Rectangle sur_veh_rec = sur_veh_traj_info.second[time_stamp_index].rectangle_;
                if (Tools::isCollision(&ego_vehicle_rec, &sur_veh_rec)) {
                    return false;
                }
            }
        }

        return true;
    }

    // Calculate ego vehicle
    Vehicle VehicleInterface::getEgoVehicle(const Eigen::Matrix<double, 2, 1>& position, const double& theta, const double& curvature, const double& velocity, const double& acceleration, const double& steer, const double& length, const double& width) {
        State ego_vehicle_state = State(0.0, position, theta, curvature, velocity, acceleration, steer);
        return Vehicle(0, ego_vehicle_state, length, width);
    }

    /**
     * @brief: Calculate surround vehicle from perception obstacles
     * @param unlaned_obstacles means the obstacles ar detached from the lane network, they are employed to generate occupied semantic cubes in trajectory planner. 
     * @return The vehicles could be handle in behavior planner.
     */    
    std::unordered_map<int, Vehicle> VehicleInterface::getSurroundVehicles(MapInterface* mtf, const std::vector<DecisionMaking::Obstacle>& obstacles, std::vector<DecisionMaking::Obstacle>& unlane_obstacles) {
        int sur_veh_index = 1;
        std::unordered_map<int, Vehicle> surround_vehicles;
        for (const auto& obs: obstacles) {
            std::pair<int, Vehicle> sur_veh_info;
            if (getSingleSurroundVehicle(mtf, obs, sur_veh_index, sur_veh_info)) {
                surround_vehicles.insert(sur_veh_info);
                sur_veh_index += 1;
            } else {
                unlane_obstacles.emplace_back(obs);
            }
        }
        return surround_vehicles;
    }

    // Calculate single surround vehicle
    bool VehicleInterface::getSingleSurroundVehicle(MapInterface* mtf, const DecisionMaking::Obstacle& obstacle, int index, std::pair<int, Vehicle>& sur_veh_info) {
        // Judge whether the position is out of the lane network
        if (!mtf->isInLane(obstacle.getObstaclePosition())) {
            return false;
        }

        // Filt the vehicle whose orientation is unreliable
        double obstacle_speed_orientation = Tools::safeThetaTransform(obstacle.getObstacleVelocityDirection());
        double nearest_lane_point_orientation = mtf->calculateNearestPointOrientation(obstacle.getObstaclePosition());
        if (Tools::safeThetaTransform(fabs(obstacle_speed_orientation - nearest_lane_point_orientation)) > PI / 3.0) {
            return false;
        }

        // Transform data from obstacle
        Eigen::Matrix<double, 2, 1> position{obstacle.getObstaclePosition().x_, obstacle.getObstaclePosition().y_};
        double orientation = obstacle.getObstacleOrientation();
        if (obstacle.getObstacleVelocity() != 0.0) {
            // Replace vehicle orientation with vehicle speed orientation
            orientation = Tools::safeThetaTransform(obstacle.getObstacleVelocityDirection());
        }

        // Construct suround vehicle information 
        State sur_vehicle_state = State(0.0, position, orientation, 0.0, obstacle.getObstacleVelocity(), 0.0, 0.0);
        Vehicle sur_vehicle = Vehicle(index, sur_vehicle_state, obstacle.getObstacleLength(), obstacle.getObstacleWidth());
        sur_veh_info = std::make_pair(index, sur_vehicle);

        return true;
    }



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

        // Visualization best traj and print
        VisualizationMethods::visualizeTrajectory(ego_traj_[winner_index], vis_pub_, 0);
        printf("[BehaviorPlanner] selected action index: %d, with the cost: %lf.\n", winner_index, winner_cost);

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

        // // DEBUG
        // std::cout << "Best action index: " << winner_index << std::endl;
        // std::cout << "Best action cost: " << winner_cost << std::endl;
        // // END DEBUG
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
        //     simulateSingleBehaviorSequence(ego_vehicle, surround_vehicles, behavior_set[62], 62);
        // }
        // // END DEBUG
    }

    // Simulate single behavior sequence
    void BehaviorPlannerCore::simulateSingleBehaviorSequence(const Vehicle& ego_vehicle, const std::unordered_map<int, Vehicle>& surround_vehicles, const BehaviorSequence& behavior_sequence, int index) {
        
        // Initialize semantic vehicles (include ego semantic vehicle and surround semantic vehicles)
        SemanticVehicle ego_semantic_vehicle = mtf_->getEgoSemanticVehicle(ego_vehicle, behavior_sequence[0].lat_beh_);
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
        // std::cout << "Behavior sequence index: " << index << " ego vehicle desired speed: " << ego_vehicle_desired_speed << std::endl;
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
                ego_semantic_vehicle = mtf_->getEgoSemanticVehicle(current_ego_vehicle, behavior_sequence[i].lat_beh_);
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
        // VisualizationMethods::visualizeTrajectory(ego_trajectory, vis_pub_, index);
        // Print the last predicted vehicle state
        // ego_trajectory.back().print();
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
