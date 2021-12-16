/*
 * @Author: fujiawei0724
 * @Date: 2021-10-27 11:36:32
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-12-16 15:07:14
 * @Descripttion: The description of vehicle in different coordinations. 
 */

#pragma once

#include <cmath>
#include <unordered_map>
#include <thread>
#include <map>
#include <array>
#include <algorithm>
#include <memory>
#include <functional>
#include <stdio.h>
#include <pthread.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/Gmpzf.h>
#include <CGAL/MP_Float.h>
#include <torch/torch.h>
#include <torch/script.h>
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
    Conservative = 0,
    Normal,
    Aggressive,
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
    ~VehicleBehavior() = default;

    LateralBehavior lat_beh_;
    LongitudinalBehavior lon_beh_;
};

// Vehicle intention
class VehicleIntention {
 public:
    VehicleIntention() = default;
    VehicleIntention(const LateralBehavior& lat_beh, const double& vel_comp) {
        lat_beh_ = lat_beh;
        lon_vel_comp_ = vel_comp;
    }
    ~VehicleIntention() = default;

    LateralBehavior lat_beh_;
    double lon_vel_comp_{0.0};
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

    // DEBUG    
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

    // DEBUG 
    void print() {
        printf("Frenet state time stamp: %lf\n", time_stamp_);
        printf("Frenet state vec_s[0]: %lf\n", vec_s_[0]);
        printf("Frenet state vec_s[1]: %lf\n", vec_s_[1]);
        printf("Frenet state vec_s[2]: %lf\n", vec_s_[2]);
        printf("Frenet state vec_ds_[0]: %lf\n", vec_ds_[0]);
        printf("Frenet state vec_ds_[1]: %lf\n", vec_ds_[1]);
        printf("Frenet state vec_ds_[2]: %lf\n", vec_ds_[2]);
        printf("Frenet state vec_dt_[0]: %lf\n", vec_dt_[0]);
        printf("Frenet state vec_dt_[1]: %lf\n", vec_dt_[1]);
        printf("Frenet state vec_dt_[2]: %lf\n", vec_dt_[2]);
    }

    double time_stamp_{0.0};
    Eigen::Matrix<double, 3, 1> vec_s_{Eigen::Matrix<double, 3, 1>::Zero()};
    Eigen::Matrix<double, 3, 1> vec_ds_{Eigen::Matrix<double, 3, 1>::Zero()};
    Eigen::Matrix<double, 3, 1> vec_dt_{Eigen::Matrix<double, 3, 1>::Zero()};

    
};

// The description of vehicle information in frenet state
class FsVehicle {
 public:
    // Constructor
    FsVehicle() = default;
    FsVehicle(const FrenetState& frenet_state, const std::vector<Eigen::Matrix<double, 2, 1>>& vertex) {
        fs_ = frenet_state;
        vertex_ = vertex;
    }
    // Destructor
    ~FsVehicle() = default;

    FrenetState fs_;
    std::vector<Eigen::Matrix<double, 2, 1>> vertex_;
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
        generateVertice();
    }

    // Destructor
    ~Vehicle() {

    }

    // Generate rectangle
    void generateRectangle() {
        rectangle_ = Rectangle(state_.position_(0), state_.position_(1), state_.theta_, width_, length_);
    }

    // Generate four vertice
    void generateVertice() {
        vertice_.resize(4);
        Eigen::Matrix<double, 2, 1> vertex_1, vertex_2, vertex_3, vertex_4;
        vertex_1(0) = state_.position_(0) + length_ * 0.5 * cos(state_.theta_) - width_ * 0.5 * sin(state_.theta_);
        vertex_1(1) = state_.position_(1) + length_ * 0.5 * sin(state_.theta_) + width_ * 0.5 * cos(state_.theta_);
        vertice_[0] = vertex_1;
        vertex_2(0) = state_.position_(0) + length_ * 0.5 * cos(state_.theta_) + width_ * 0.5 * sin(state_.theta_);
        vertex_2(1) = state_.position_(1) + length_ * 0.5 * sin(state_.theta_) - width_ * 0.5 * cos(state_.theta_);
        vertice_[1] = vertex_2;
        vertex_3(0) = state_.position_(0) - length_ * 0.5 * cos(state_.theta_) + width_ * 0.5 * sin(state_.theta_);
        vertex_3(1) = state_.position_(1) - length_ * 0.5 * sin(state_.theta_) - width_ * 0.5 * cos(state_.theta_);
        vertice_[2] = vertex_3;
        vertex_4(0) = state_.position_(0) - length_ * 0.5 * cos(state_.theta_) - width_ * 0.5 * sin(state_.theta_);
        vertex_4(1) = state_.position_(1) - length_ * 0.5 * sin(state_.theta_) + width_ * 0.5 * cos(state_.theta_);
        vertice_[3] = vertex_4;
    }

    // DEBUG
    void print() {
        printf("Vehicle id: %d\n", id_);
        state_.print();
    }

    int id_{0};
    State state_;
    double length_{0.0};
    double width_{0.0};
    Rectangle rectangle_;
    std::vector<Eigen::Matrix<double, 2, 1>> vertice_{};
};

// Transform between world state and frenet state
class StateTransformer {
 public:
    StateTransformer() = default;
    StateTransformer(const Lane& lane) {
        lane_ = lane;
    }
    ~StateTransformer() = default;

    
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
        const double step_tolerance = 0.2;
        if (fabs((state.position_ - lane_position).dot(lane_tangent_vec)) > step_tolerance) {
            printf("[StatsTransformer] offset %lf larger than tolerance.\n", fabs((state.position_ - lane_position).dot(lane_tangent_vec)));
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
        frenet_state.time_stamp_ = state.time_stamp_;

        return frenet_state;
    }

    // Transform frenet position to world position
    Eigen::Matrix<double, 2, 1> getPointFromFrenetPoint(const Eigen::Matrix<double, 2, 1>& frenet_point) {
        // Determine the nearest position to frenet state
        std::vector<PathPlanningUtilities::CoordinationPoint> lane_coordination = lane_.getLaneCoordnation();
        
        // Origin transformation, no use
        // PathPlanningUtilities::CoordinationPoint lane_position;
        // for (const PathPlanningUtilities::CoordinationPoint& lane_point: lane_coordination) {
        //     if (lane_point.station_ > frenet_point(0)) {
        //         lane_position = lane_point;
        //         break;
        //     }
        // }
        
        double lane_position_x = 0.0, lane_position_y = 0.0, lane_position_theta = 0.0, lane_position_curvature = 0.0;
        int i = 0;
        for (; i < static_cast<int>(lane_coordination.size()); i++) {
            if (lane_coordination[i].station_ > frenet_point(0)) {
                if (i == 0) {
                    printf("[StateTransformer] transformed point out of range.\n");
                }
                break;
            }
        }
        double remain_station = frenet_point(0) - lane_coordination[i - 1].station_;
        QuinticSpline::calculatePointInfo(lane_coordination[i - 1].worldpos_, lane_coordination[i].worldpos_, LANE_GAP_DISTANCE, remain_station, &lane_position_x, &lane_position_y, &lane_position_theta, &lane_position_curvature);

        Eigen::Matrix<double, 2, 1> lane_pos{lane_position_x, lane_position_y};
        
        // Get tangent vector and normal vector, the norm is set with 1.0
        double lane_orientation = lane_position_theta;
        double y = tan(lane_orientation);
        double x = 1.0;
        Eigen::Matrix<double, 2, 1> lane_tangent_vec{x, y};
        lane_tangent_vec /= lane_tangent_vec.norm();
        Eigen::Matrix<double, 2, 1> vec_normal{-lane_tangent_vec(1), lane_tangent_vec(0)};
        Eigen::Matrix<double, 2, 1> point = vec_normal * frenet_point(1) + lane_pos;
        return point;
    }

    // Get state 
    // TODO: add interpolation to improve precision
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

    // Transform vehicle to frenet vehicle
    FsVehicle getFsVehicleFromVehicle(const Vehicle& vehicle) {
        // Transform state information 
        FrenetState frenet_state = getFrenetStateFromState(vehicle.state_);
        
        // Transform four vertice from world frame to frenet frame
        std::vector<Eigen::Matrix<double, 2, 1>> fs_vertice;
        for (const auto& world_vertex : vehicle.vertice_) {
            Eigen::Matrix<double, 2, 1> fs_vertex = getFrenetPointFromPoint(world_vertex);
            fs_vertice.emplace_back(fs_vertex);
        }
        
        FsVehicle fs_vehicle = FsVehicle(frenet_state, fs_vertice);
        return fs_vehicle;
    }

    // Transform trajectory to frenet trajectory
    std::vector<FsVehicle> getFrenetTrajectoryFromTrajectory(const std::vector<Vehicle>& traj) {
        int traj_size = static_cast<int>(traj.size());
        std::vector<FsVehicle> fs_traj(traj_size);
        for (int i = 0; i < traj_size; i++) {
            fs_traj[i] = getFsVehicleFromVehicle(traj[i]);
        }
        return fs_traj;
    }

    // Transform ego vehicle world state to the state array in frenet (for HPDM)
    std::vector<double> getFrenetEgoVehicleStateArray(const Vehicle& ego_vehicle) {
        std::vector<double> ego_veh_state_array;

        // Note that the value need to convert is position.x, position.y, theta, velocity, acceleration, and curvature. To simplify the calculation, the valicity and acceleration have not been converted at the initial version.
        // TODO: convert velocity and acceleration if need

        // Convert position, theta, curvature
        std::vector<PathPlanningUtilities::CoordinationPoint> lane_coordination = lane_.getLaneCoordnation();
        std::vector<TransMatrix> trans_matrixes = lane_.getLaneTransMatrix();
        size_t start_index_of_lane = lane_.findCurrenPositionIndexInLane(ego_vehicle.state_.position_);
        TransMatrix trans_matrix = trans_matrixes[start_index_of_lane];
        Eigen::Vector2d start_point_in_world_v2d(ego_vehicle.state_.position_(0), ego_vehicle.state_.position_(1));
        Eigen::Vector2d start_point_position_in_frenet;
        Tools::transferPointCoordinateSystem(trans_matrix, start_point_in_world_v2d, &start_point_position_in_frenet);
        double start_point_theta_in_frenet;
        Tools::transferThetaCoordinateSystem(trans_matrix, ego_vehicle.state_.theta_, &start_point_theta_in_frenet);
        double start_point_kappa_in_frenet;
        start_point_kappa_in_frenet = ego_vehicle.state_.curvature_ - cos(start_point_theta_in_frenet)*cos(start_point_theta_in_frenet)*cos(start_point_theta_in_frenet)*1.0/(1.0/lane_coordination[start_index_of_lane].worldpos_.kappa_ - start_point_position_in_frenet(1));

        // Supple data
        // Note that ego vehicle's size is identical with training model, length: 5.0, width: 1.95
        ego_veh_state_array = std::vector<double>{start_point_position_in_frenet(0), start_point_position_in_frenet(1), start_point_theta_in_frenet, 5.0, 1.95, ego_vehicle.state_.velocity_, ego_vehicle.state_.acceleration_, start_point_kappa_in_frenet, ego_vehicle.state_.steer_};

        return ego_veh_state_array;
    }

    // Transform single surround vehicle world state to the state array in frenet (for HPDM)
    std::vector<double> getFrenetSurroundVehicleStateArray(const Vehicle& surround_vehicle) {
        std::vector<double> surround_veh_state_array;

        // Note that the value need to convert is position.x, position.y, theta, velocity, acceleration. Different from ego vehicle, curvature and steer information could not be provided, and a special flag is added at the head of the vector to represent whether current surround vehicle is existed. To simplify the calculation, the valicity and acceleration have not been converted at the initial version. 
        // TODO: convert velocity and acceleration if need

        // Convert position, theta
        std::vector<PathPlanningUtilities::CoordinationPoint> lane_coordination = lane_.getLaneCoordnation();
        std::vector<TransMatrix> trans_matrixes = lane_.getLaneTransMatrix();
        size_t start_index_of_lane = lane_.findCurrenPositionIndexInLane(surround_vehicle.state_.position_);
        TransMatrix trans_matrix = trans_matrixes[start_index_of_lane];
        Eigen::Vector2d start_point_in_world_v2d(surround_vehicle.state_.position_(0), surround_vehicle.state_.position_(1));
        Eigen::Vector2d start_point_position_in_frenet;
        Tools::transferPointCoordinateSystem(trans_matrix, start_point_in_world_v2d, &start_point_position_in_frenet);
        double start_point_theta_in_frenet;
        Tools::transferThetaCoordinateSystem(trans_matrix, surround_vehicle.state_.theta_, &start_point_theta_in_frenet);

        // Supple data 
        surround_veh_state_array = std::vector<double>{1.0, start_point_position_in_frenet(0), start_point_position_in_frenet(1), start_point_theta_in_frenet, surround_vehicle.length_, surround_vehicle.width_, surround_vehicle.state_.velocity_, surround_vehicle.state_.acceleration_};

        return surround_veh_state_array;
    }

    Lane lane_;
};


// Vehicle state with semantic information, such as current lane and reference lane
class SemanticVehicle {
 public:
    // Constructor
    SemanticVehicle() = default;
    SemanticVehicle(const Vehicle& vehicle, const LaneId& nearest_lane_id, const LaneId& reference_lane_id, const Lane& nearest_lane, const Lane& reference_lane) {
        vehicle_ = vehicle;
        nearest_lane_id_ = nearest_lane_id;
        reference_lane_id_ = reference_lane_id;
        nearest_lane_ = nearest_lane;
        reference_lane_ = reference_lane;
    }

    // Destructor
    ~SemanticVehicle() {
        
    }

    Vehicle vehicle_;
    LaneId nearest_lane_id_{LaneId::Undefined};
    LaneId reference_lane_id_{LaneId::Undefined};
    Lane nearest_lane_;
    Lane reference_lane_;
};


class Point2i {
 public:
    Point2i() = default;
    Point2i(int x, int y) {
        x_ = x;
        y_ = y;
    }
    ~Point2i() = default;

    int x_{0};
    int y_{0};
};

class Point3i : public Point2i {
 public:
    Point3i() : Point2i() {
        
    }
    Point3i(int x, int y, int z) : Point2i(x, y) {
        z_ = z;
    }
    ~Point3i() = default;

    int z_{0};
};

class Point2f {
 public:
    Point2f() = default;
    Point2f(double x, double y) {
        x_ = x;
        y_ = y;
    }
    ~Point2f() = default;

    double x_{0.0};
    double y_{0.0};
};
 
class Point3f {
 public:
    Point3f() = default;
    Point3f(double x, double y, double z) {
        x_ = x;
        y_ = y;
        z_ = z;
    }
    ~Point3f() = default;

    double x_{0.0};
    double y_{0.0};
    double z_{0.0};
};


// The semantic cube to constrain the position of trajectory interpolation points
template <typename T>
class SemanticCube {
 public:
    // Constructor
    SemanticCube() = default;
    SemanticCube(int id, T s_start, T s_end, T d_start, T d_end, T t_start, T t_end) {
        id_ = id;
        s_start_ = s_start;
        s_end_ = s_end;
        d_start_ = d_start;
        d_end_ = d_end;
        t_start_ = t_start;
        t_end_ = t_end;
    }
    // Destructor
    ~SemanticCube() = default;

    // DEBUG
    void print() {
        // std::cout << "semantic cube id: " << id_ << std::endl;
        std::cout << "s_start: " << s_start_ << std::endl;
        std::cout << "s_end: " << s_end_ << std::endl;
        std::cout << "d_start: " << d_start_ << std::endl;
        std::cout << "d_end: " << d_end_ << std::endl;
        std::cout << "t_start: " << t_start_ << std::endl;
        std::cout << "t_end: " << t_end_ << std::endl;
    }

    T s_start_;
    T s_end_;
    T d_start_;
    T d_end_;
    T t_start_;
    T t_end_;
    int id_{-1}; // Means ego index in a cubes sequence, start from 0
};

class ShapeUtils {
 public:
    static void getCvPoint2iVecUsingCommonPoint2iVec(const std::vector<Point2i>& pts_in, std::vector<cv::Point2i>& pts_out) {
        int num = pts_in.size();
        pts_out.resize(num);
        for (int i = 0; i < num; i++) {
            getCvPoint2iUsingCommonPoint2i(pts_in[i], pts_out[i]);
        }
    }

    static void getCvPoint2iUsingCommonPoint2i(const Point2i& pt_in, cv::Point2i& pt_out) {
        pt_out.x = pt_in.x_;
        pt_out.y = pt_in.y_;
    }

    /**
     * @brief Generate initial semantic cubes using two continuous seeds
     * @param seed input seeds
     * @param index semantic cube index
     * @return the constructed semantic cube
     */   
    static SemanticCube<int> generateInitialCoordSemanticCube(const Point3i& seed_1, const Point3i& seed_2, const int& index = -1) {
        SemanticCube<int> semantic_cube;
        semantic_cube.s_start_ = std::min(seed_1.x_, seed_2.x_);
        semantic_cube.s_end_ = std::max(seed_1.x_, seed_2.x_);
        semantic_cube.d_start_ = std::min(seed_1.y_, seed_1.y_);
        semantic_cube.d_end_ = std::max(seed_1.y_, seed_2.y_);
        semantic_cube.t_start_ = std::min(seed_1.z_, seed_2.z_);
        semantic_cube.t_end_ = std::max(seed_1.z_, seed_2.z_);
        semantic_cube.id_ = index;
        
        return semantic_cube;
    } 


};




// Equal constraints related to start point and end point
class EqualConstraint {
 public:
    // Constructor
    EqualConstraint() = default;
    EqualConstraint(double s, double d_s, double dd_s, double d, double d_d, double dd_d) {
        s_ = s;
        d_s_ = d_s;
        dd_s_ = dd_s;
        d_ = d;
        d_d_ = d_d;
        dd_d_ = dd_d;
        s_info_ = {s, d_s, dd_s};
        d_info_ = {d, d_d, dd_d};
    }
    // Destructor
    ~EqualConstraint() = default;

    void load(const FsVehicle& fs_veh) {
        s_ = fs_veh.fs_.vec_s_[0];
        d_s_ = fs_veh.fs_.vec_s_[1];
        dd_s_ = fs_veh.fs_.vec_s_[2];
        d_ = fs_veh.fs_.vec_dt_[0];
        d_d_ = fs_veh.fs_.vec_dt_[1];
        dd_d_ = fs_veh.fs_.vec_dt_[2];
        s_info_ = {s_, d_s_, dd_s_};
        d_info_ = {d_, d_d_, dd_d_};
    }

    std::array<double, 3> toDimensionS() const {
        return s_info_;
    }

    std::array<double, 3> toDimensionD() const {
        return d_info_;
    }

    // s means the longitudinal dimension, d means the latitudinal dimension, d_ means the first derivative, dd_ denotes the second derivative
    double s_{0.0};
    double d_s_{0.0};
    double dd_s_{0.0};
    double d_{0.0};
    double d_d_{0.0};
    double dd_d_{0.0};
    std::array<double, 3> s_info_{};
    std::array<double, 3> d_info_{};
};

// // Unequal constraints related to intermediate points' position
// // Note that the unequal constraints are only related to the position
// class UnequalConstraint {
// public:
//     // Constructor
//     UnequalConstraint() = default;
//     UnequalConstraint(double s_low, double s_up, double d_low, double d_up) {
//         s_low_ = s_low;
//         s_up_ = s_up;
//         d_low_ = d_low;
//         d_up_ = d_up;
//     }
//     // Destructor 
//     ~UnequalConstraint() = default;
    
//     double s_low_;
//     double s_up_;
//     double d_low_;
//     double d_up_;
// };


// Grip map used in ND lattice representation
template <typename T, int N_DIM>
class GridMapND {
 public:
    enum ValType{
        OCCUPIED = 70,
        FREE = 0,
        SCANNED_OCCUPIED = 128,
        UNKNOWN = 0,
    };

    // Constructor
    GridMapND() = default;
    GridMapND(const std::array<int, N_DIM>& dims_size, const std::array<double, N_DIM>& dims_resolution, const std::array<std::string, N_DIM>& dims_name) {
        dims_size_ = dims_size;
        dims_resolution_ = dims_resolution;
        dims_name_ = dims_name;

        setNDimSteps(dims_size_);
        setDataSize(dims_size_);
        data_ = std::vector<T>(data_size_, 0);
        origin_.fill(0);
    }
    // Destructor
    ~GridMapND() = default;

    inline std::array<int, N_DIM> dims_size() const { 
        return dims_size_; 
    }
    inline int dims_size(const int& dim) const { 
        return dims_size_.at(dim); 
    }
    inline std::array<int, N_DIM> dims_step() const { 
        return dims_step_; 
    }
    inline int dims_step(const int& dim) const { 
        return dims_step_.at(dim); 
    }
    inline std::array<double, N_DIM> dims_resolution() const {
        return dims_resolution_;
    }
    inline double dims_resolution(const int& dim) const {
        return dims_resolution_.at(dim);
    }
    inline std::array<std::string, N_DIM> dims_name() const { 
        return dims_name_; 
    }
    inline std::string dims_name(const int& dim) const {
        return dims_name_.at(dim);
    }
    inline std::array<double, N_DIM> origin() const { 
        return origin_; 
    }
    inline int data_size() const { 
        return data_size_; 
    }
    inline const std::vector<T>* data() const {         
        return &data_; 
    }
    inline T data(const int& i) const { 
        return data_[i];
    }
    inline T* get_data_ptr() { 
        return data_.data(); 
    }
    inline const T* data_ptr() const { 
        return data_.data(); 
    }

    inline void set_origin(const std::array<double, N_DIM>& origin) {
        origin_ = origin;
    }
    inline void set_dims_size(const std::array<int, N_DIM>& dims_size) {
        dims_size_ = dims_size;
        setNDimSteps(dims_size);
        setDataSize(dims_size);
    }
    inline void set_dims_resolution(const std::array<double, N_DIM>& dims_resolution) {
        dims_resolution_ = dims_resolution;
    }
    inline void set_dims_name(const std::array<std::string, N_DIM>& dims_name) {
        dims_name_ = dims_name;
    }
    inline void set_data(const std::vector<T>& in) { 
        data_ = in; 
    }

    /**
     * @brief Set all data in array to 0
     */
    inline void clear_data() { 
        data_ = std::vector<T>(data_size_, 0); 
    }

    /**
     * @brief Fill all data in array using value
     *
     * @param val input value
     */
    inline void fill_data(const T& val) {
        data_ = std::vector<T>(data_size_, val);
    }


    // Calculate data size
    void setDataSize(const std::array<int, N_DIM>& dims_size) {
        int total_ele_num = 1;
        for (int i = 0; i < N_DIM; i++) {
            total_ele_num *= dims_size[i];
        }
        data_size_ = total_ele_num;
    }

    /**
     * @brief Set the steps of N-dim array
     * @brief E.g. A x-y-z map's steps are {1, x, x*y}
     * @param dims_size input the size of dimension
     */  
    void setNDimSteps(const std::array<int, N_DIM>& dims_size) {
        int step = 1;
        for (int i = 0; i < N_DIM; i++) {
            dims_step_[i] = step;
            step *= dims_size[i];
        }
    }

    /**
     * @brief Get the Value Using Coordinate
     *
     * @param coord Input coordinate
     * @param val Output value
     * @return is success
     */
    bool getValueUsingCoordinate(const std::array<int, N_DIM>& coord, T& val) const {
        if (!checkCoordInRange(coord)) {
            return false;
        }
        int idx = getMonoIdxUsingNDimIdx(coord);
        val = data_[idx];
        return true;
    }


    /**
     * @brief Get the Value Using Global Position
     *
     * @param p_w Input global position
     * @param val Output value
     * @return is success
     */
    bool getValueUsingGlobalPosition(const std::array<double, N_DIM>& p_w, T& val) const {
        std::array<int, N_DIM> coord = getCoordUsingGlobalPosition(p_w);
        return getValueUsingCoordinate(coord, val);
    }

    /**
     * @brief Check if the input value is equal to the value in map
     *
     * @param p_w Input global position
     * @param val_in Input value
     * @return result
     */
    bool checkIfEqualUsingGlobalPosition(const std::array<double, N_DIM>& p_w, const T& val_in) const {
        std::array<int, N_DIM> coord = getCoordUsingGlobalPosition(p_w);
        T val;
        if (!getValueUsingCoordinate(coord, val)) {
            return false;
        }
        return val == val_in;
    }

    /**
     * @brief Check if the input value is equal to the value in map
     *
     * @param coord Input coordinate
     * @param val_in Input value
     * @return result
     */
    bool checkIfEqualUsingCoordinate(const std::array<int, N_DIM>& coord, const T& val_in) {
        T val;
        if (!getValueUsingCoordinate(coord, val)) {
            return false;
        } 

        return val == val_in;
        
    }

    /**
     * @brief Set the Value Using Coordinate
     * @param coord Coordinate of the map
     * @param val Input value
     * @return is success
     */
    bool setValueUsingCoordinate(const std::array<int, N_DIM>& coord, const T& val) {
        if (!checkCoordInRange(coord)) {
            printf("[GridMapND] Out of range\n");
            return false;
        }
        int idx = getMonoIdxUsingNDimIdx(coord);
        data_[idx] = val;
        return true;
    }

  /**
   * @brief Set the Value Using Global Position
   *
   * @param p_w Global position
   * @param val Input value
   * @return is success
   */
    bool setValueUsingGlobalPosition(const std::array<double, N_DIM>& p_w, const T& val) {
        std::array<int, N_DIM> coord = getCoordUsingGlobalPosition(p_w);
        return setValueUsingCoordinate(coord, val);
    }

    /**
     * @brief Get the Coordinate Using Global Position
     *
     * @param p_w Input global position
     * @return std::array<int, N_DIM> Output coordinate
     */
    std::array<int, N_DIM> getCoordUsingGlobalPosition(const std::array<double, N_DIM>& p_w) const {
        std::array<int, N_DIM> coord = {};
        for (int i = 0; i < N_DIM; i++) {
            coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
        }
        return coord;
    }

    /**
     * @brief Get the Rounded Position Using Global Position object
     *
     * @param p_w Input global position
     * @return std::array<decimal_t, N_DIM> Output global position
     */
    std::array<double, N_DIM> getRoundedPosUsingGlobalPosition(const std::array<double, N_DIM>& p_w) const {
        std::array<int, N_DIM> coord = {};
        for (int i = 0; i < N_DIM; i++) {
            coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
        }
        std::array<double, N_DIM> round_pos = {};
        for (int i = 0; i < N_DIM; i++) {
            round_pos[i] = coord[i] * dims_resolution_[i] + origin_[i];
        }
        return round_pos;
    }

    /**
     * @brief Get the Global Position Using Coordinate
     *
     * @param coord Input coordinate
     * @return Output global position
     */
    std::array<double, N_DIM> getGlobalPositionUsingCoordinate(const std::array<int, N_DIM>& coord) const {
        std::array<double, N_DIM> p_w = {};
        for (int i = 0; i < N_DIM; i++) {
            p_w[i] = coord[i] * dims_resolution_[i] + origin_[i];
        }
        return p_w;
    }

    /**
     * @brief Get the Coordinate Using Global Metric On Single Dimension
     *
     * @param metric Input global 1-dim position
     * @param i Dimension
     * @return idx Output 1-d coordinate
     */
    int getCoordUsingGlobalMetricOnSingleDim(const double& metric, const int& i) {
        int idx = std::round((metric - origin_[i]) / dims_resolution_[i]);
        return idx;
    }


    /**
     * @brief Get the Global Metric Using Coordinate On Single Dim object
     *
     * @param idx Input 1-d coordinate
     * @param i Dimension
     * @return metric Output 1-d position
     */
    double getGloalMetricUsingCoordOnSingleDim(const int& idx, const int& i) {
        double metric = idx * dims_resolution_[i] + origin_[i];
        return metric;
    }

    /**
     * @brief Check if the input coordinate is in map range
     *
     * @param coord Input coordinate
     * @return true In range
     * @return false Out of range
     */
    bool checkCoordInRange(const std::array<int, N_DIM>& coord) const {
        for (int i = 0; i < N_DIM; i++) {
            if (coord[i] < 0 || coord[i] >= dims_size_[i]) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Check if the input 1-d coordinate is in map range
     *
     * @param idx Input 1-d coordinate
     * @param i Dimension
     * @return true In range
     * @return false Out of range
     */
    bool checkCoordInRangeOnSingleDim(const int& idx, const int& i) const {
        return (idx >= 0) && (idx < dims_size_[i]);
    }

    /**
     * @brief Get the mono index using N-dim index
     *
     * @param idx Input N-dim index
     * @return int Output 1-dim index
     */
    int getMonoIdxUsingNDimIdx(const std::array<int, N_DIM>& idx) const {
        int mono_idx = 0;
        for (int i = 0; i < N_DIM; i++) {
            mono_idx += dims_step_[i] * idx[i];
        }
        return mono_idx;
    }

    /**
     * @brief Get N-dim index using mono index
     *
     * @param idx Input mono index
     * @return std::array<int, N_DIM> Output N-dim index
     */
    std::array<int, N_DIM> getNDimIdxUsingMonoIdx(const int& idx) const {
        std::array<int, N_DIM> idx_nd = {};
        int tmp = idx;
        for (int i = N_DIM - 1; i >= 0; i--) {
            idx_nd[i] = tmp / dims_step_[i];
            tmp = tmp % dims_step_[i];
        }
        return idx_nd;
    }



    std::array<int, N_DIM> dims_size_;
    std::array<int, N_DIM> dims_step_;
    std::array<double, N_DIM> dims_resolution_;
    std::array<std::string, N_DIM> dims_name_;
    std::array<double, N_DIM> origin_;

    int data_size_{0};
    std::vector<T> data_;

    

};

// The data bridge between behavior planner and trajectory planner
class BpTpBridge {
 public:
    BpTpBridge(const Lane& ego_reference_lane) {
        state_trans_itf_ = new StateTransformer(ego_reference_lane);
    }
    ~BpTpBridge() = default;

    // Transform trajectory from frenet to world
    std::vector<Point3f> getTrajFromTrajFs(const std::vector<Point3f>& traj_fs) {
        int length = static_cast<int>(traj_fs.size());
        std::vector<Point3f> traj(length);

        for (int i = 0; i < length; i++) {
            Eigen::Matrix<double, 2, 1> point_fs{traj_fs[i].x_, traj_fs[i].y_};
            Eigen::Matrix<double, 2, 1> point = state_trans_itf_->getPointFromFrenetPoint(point_fs);
            traj[i] = Point3f(point(0), point(1), traj_fs[i].z_); 
        }

        return traj;
    }

    // Transform 2d trajectory points
    std::vector<Point2f> get2dTrajFrom2dTrajFs(const std::vector<Point2f>& traj_fs) {
        int length = static_cast<int>(traj_fs.size());
        std::vector<Point2f> traj(length);

        for (int i = 0; i < length; i++) {
            Eigen::Matrix<double, 2, 1> point_fs{traj_fs[i].x_, traj_fs[i].y_};
            Eigen::Matrix<double, 2, 1> point = state_trans_itf_->getPointFromFrenetPoint(point_fs);
            traj[i] = Point2f(point(0), point(1)); 
        }

        return traj;
    }

    // Transform ego state (update after behavior planning)
    FsVehicle getFsVehicle(const Vehicle& vehicle) {
        return state_trans_itf_->getFsVehicleFromVehicle(vehicle);
    }

    // Transform ego trajectory 
    std::vector<FsVehicle> getEgoFrenetTrajectory(const std::vector<Vehicle>& ego_traj) {
        return state_trans_itf_->getFrenetTrajectoryFromTrajectory(ego_traj);
    }

    // Transform trajectories of surround vehicles that on lane network
    std::unordered_map<int, std::vector<FsVehicle>> getSurFrenetTrajectories(const std::unordered_map<int, std::vector<Vehicle>>& sur_trajs) {
        std::unordered_map<int, std::vector<FsVehicle>> sur_trajs_fs;
        for (const auto& sur_traj : sur_trajs) {
            sur_trajs_fs.insert({sur_traj.first, state_trans_itf_->getFrenetTrajectoryFromTrajectory(sur_traj.second)});
        }
        return sur_trajs_fs;
    }

    // Transform trajectories of unlaned obstacles from simple prediction
    // TODO: distinguish the unlaned obstacles from laned vehicles
    std::unordered_map<int, std::vector<FsVehicle>> getUnlanedSurFrenetTrajectories(const std::vector<DecisionMaking::Obstacle>& unlaned_obstacles) {
        int unlaned_obstacle_index = 10001;
        std::unordered_map<int, std::vector<Vehicle>> sur_unlaned_trajs;
        for (const auto& obs : unlaned_obstacles) {
            if (obs.getObstacleVelocity() == 0.0) {
                // Static obstacles are not handled here
                continue;
            }
            
            // Determine current position
            Eigen::Matrix<double, 2, 1> cur_position{obs.getObstaclePosition().x_, obs.getObstaclePosition().y_};
            State cur_state = State(0.0, cur_position, obs.getObstacleOrientation(), 0.0, obs.getObstacleVelocity(), 0.0, 0.0);

            // Traverse each predicted path for single unlaned obstacles
            // If an obstacle has two or more predicted trajectories, it will be calculated as two obstacles
            for (const auto& pred_traj : obs.getPredictedTrajectorySet()) {
                // Initialize predicted pathes
                std::vector<Vehicle> unlaned_obs_traj;
                unlaned_obs_traj.emplace_back(Vehicle(unlaned_obstacle_index, cur_state, obs.getObstacleLength(), obs.getObstacleWidth()));

                // Supple predicted information 
                for (int i = 1; i < static_cast<int>(pred_traj.size()); i++) {
                    Eigen::Matrix<double, 2, 1> this_position{pred_traj[i].position_.x_, pred_traj[i].position_.y_};
                    State this_state = State(i * 0.4, this_position, pred_traj[i].theta_, pred_traj[i].kappa_, obs.getObstacleVelocity(), 0.0, 0.0);
                    unlaned_obs_traj.emplace_back(Vehicle(unlaned_obstacle_index, this_state, obs.getObstacleLength(), obs.getObstacleWidth()));
                }

                // Cache
                sur_unlaned_trajs[unlaned_obstacle_index] = unlaned_obs_traj;

                unlaned_obstacle_index += 1;
            }
        }

        // Transform to frenet state
        std::unordered_map<int, std::vector<FsVehicle>> sur_unlaned_trajs_fs;
        for (const auto& sur_unlaned_traj : sur_unlaned_trajs) {
            sur_unlaned_trajs_fs.insert({sur_unlaned_traj.first, state_trans_itf_->getFrenetTrajectoryFromTrajectory(sur_unlaned_traj.second)});
        }

        return sur_unlaned_trajs_fs;
    }

    StateTransformer* state_trans_itf_{nullptr};
    
};


} // End of namespace BehaviorPlanner


