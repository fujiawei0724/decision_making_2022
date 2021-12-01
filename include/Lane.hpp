/*
    Copyright [2020] Jian ZhiQiang
*/

#ifndef LANE_INCLUDE_COMMON_HPP_
#define LANE_INCLUDE_COMMON_HPP_

#include <path_planning_msgs/BoundedCurve.h>
#include "Point.hpp"
#include "Path.hpp"
#include "Const.hpp"
#include "Compare.hpp"
#include "Tools.hpp"

//===================================================== 道路类 ======================================================
class Lane{
 public:
    enum GuidanceType {
        CHANGE_LEFT = 1,
        KEEP_CENTER = 2,
        CHANGE_RIGHT = 4,
        CENTER_LEFT = 3,
        CENTER_RIGHT = 6,
        ALL_AVAILABLE = 7
    };

    Lane() {}

    ~Lane() {}

    // 将道路设置为可用状态
    void enable() {
        this->lane_existance_ = true;
    }

    // 将道路设置为不可用状态
    void disable() {
        this->lane_existance_ = false;
    }

    bool getLaneExistance() const {
        return this->lane_existance_;
    }

    // 生成道路中线信息(包括坐标系、中线在world系、中线在frenet系)(每一点的坐标系以前向为正，后向为负，左向为正，右向为负)
    void generateLaneCenter(path_planning_msgs::BoundedCurve geometry) {
        std::vector<PathPlanningUtilities::CoordinationPoint>().swap(this->lane_coorination_);
        PathPlanningUtilities::Path().swap(this->lane_center_path_in_world_);
        PathPlanningUtilities::Path().swap(this->lane_center_path_in_frenet_);
        double station = 0.0;
        for (size_t i = 0; i < geometry.points.size(); i++) {
            PathPlanningUtilities::CoordinationPoint coordinate_point;
            PathPlanningUtilities::Point2f point_in_world;
            PathPlanningUtilities::Point2f point_in_frenet;
            if (i > 0) {
                station += sqrt((geometry.points[i].center_point.x - geometry.points[i-1].center_point.x)*(geometry.points[i]
                .center_point.x - geometry.points[i-1].center_point.x) + (geometry.points[i].center_point.y - geometry.points[i-1]
                .center_point.y)*(geometry.points[i].center_point.y - geometry.points[i-1].center_point.y));
            }
            coordinate_point.worldpos_.position_.x_ = geometry.points[i].center_point.x;
            coordinate_point.worldpos_.position_.y_ = geometry.points[i].center_point.y;
            coordinate_point.worldpos_.theta_ = geometry.points[i].center_point.theta;
            coordinate_point.worldpos_.kappa_ = geometry.points[i].center_point.kappa;
            coordinate_point.station_ = station;
            coordinate_point.min_height_ = std::min(-0.1, -geometry.points[i].right_distance);
            coordinate_point.max_height_ = std::max(0.1, geometry.points[i].left_distance);
            point_in_world.x_ = geometry.points[i].center_point.x;
            point_in_world.y_ = geometry.points[i].center_point.y;
            point_in_frenet.x_ = station;
            point_in_frenet.y_ = 0.0;
            this->lane_coorination_.push_back(coordinate_point);
            this->lane_center_path_in_world_.push_back(point_in_world);
            this->lane_center_path_in_frenet_.push_back(point_in_frenet);
        }
    }

    // 获取道路中线信息(包括坐标系、中线在world系、中线在frenet系)
    const std::vector<PathPlanningUtilities::CoordinationPoint> &getLaneCoordnation() const {
        return this->lane_coorination_;
    }

    const PathPlanningUtilities::Path &getLaneCenterPathInWorld() const {
        return this->lane_center_path_in_world_;
    }

    const PathPlanningUtilities::Path &getLaneCenterPathInFrenet() const {
        return this->lane_center_path_in_frenet_;
    }

    // 确定道路中每一个点的变换矩阵
    void generateLaneTransMatrix() {
        std::vector<TransMatrix>().swap(this->lane_transmatrix_);
        for (size_t i = 0; i < this->lane_coorination_.size(); i++) {
            TransMatrix trans_matrix;
            Eigen::Matrix2d matrix_2d;
            matrix_2d << cos(this->lane_coorination_[i].worldpos_.theta_), -sin(this->lane_coorination_[i].worldpos_.theta_), sin(this->lane_coorination_[i].worldpos_.theta_), cos(this->lane_coorination_[i].worldpos_.theta_);
            trans_matrix.rotation_ = matrix_2d.inverse();
            // ROS_INFO_STREAM( trans_matrix.rotation_ ) );
            Eigen::Vector2d world_position(this->lane_coorination_[i].worldpos_.position_.x_, this->lane_coorination_[i].worldpos_.position_.y_);
            Eigen::Vector2d sd_position(this->lane_coorination_[i].station_, 0.0);
            trans_matrix.trans_ = sd_position - trans_matrix.rotation_*world_position;
            this->lane_transmatrix_.push_back(trans_matrix);
        }
    }

    const std::vector<TransMatrix> &getLaneTransMatrix() const {
        return this->lane_transmatrix_;
    }

    // 确定道路限速
    // void setLaneVelocityLimitaion(double lane_velocity_limitation) {
    //     this->lane_velocity_limitation_ = lane_velocity_limitation;
    // }

    // double getLaneVelocityLimitation() {
    //     return this->lane_velocity_limitation_;
    // }

    // 确定道路限速
    void setLaneVelocityLimitation(const std::vector<double> &lane_velocity_limitation) {
        this->lane_velocity_limitation_ = lane_velocity_limitation;
    }

    const std::vector<double> &getLaneVelocityLimitation() const {
        return this->lane_velocity_limitation_;
    }

    // 确定道路最低速度
    void setLowVelocity(const std::vector<double> &lane_lowest_velocity) {
        this->lane_lowest_velocity_ = lane_lowest_velocity;
    }

    const std::vector<double> &getLowVelocity() const {
        return this->lane_lowest_velocity_;
    }

    // 确定道路是否是被引导道路
    void setGuidedFlag(bool is_guided) {
        this->is_guided_ = is_guided;
    }

    bool getGuidedFlag() const {
        return this->is_guided_;
    }

    // 找出当前位置在道路中对应的下标
    size_t findCurrenPositionIndexInLane(double position_x, double position_y) const {
        size_t index = lane_coorination_.size() - 1;
        PathPlanningUtilities::Point2f current_position;
        current_position.x_ = position_x;
        current_position.y_ = position_y;
        for (size_t i = 0; i < this->lane_coorination_.size(); i++) {
            PathPlanningUtilities::Point2f local_position = Tools::calcNewCoordinationPosition(this->lane_coorination_[i].worldpos_, current_position);
            if (!Tools::isLarge(local_position.x_, 0.0)) {
                index = i;
                break;
            }
        }
        return index;
    }

    void setTurn(int turn) {
        this->turn_ = turn;
    }

    int getTurn() const {
        return this->turn_;
    }

    // 点是否在道路内
    bool judgePoint2fInLane(const PathPlanningUtilities::Point2f &position, size_t start_index, size_t end_index, double width) const {
        PathPlanningUtilities::Path lane_center_path_in_world = this->getLaneCenterPathInWorld();
        for (size_t i = start_index; i < end_index; i++) {
            double tmp_distance = PathPlanningUtilities::calcDistance(position, lane_center_path_in_world[i]);
            if (tmp_distance <= width * 0.5) {
                return true;
            }
        }
        return false;
    }

    // 设置道路的优先级
    void setLanePriority(double priority) {
        this->priority_ = priority;
    }

    // 获取道路优先级
    double getLanePriority() const {
        return this->priority_;
    }

    // // 确定道路中是否有停止线
    // void setStopLineExistance(bool stop_line_existance){
    //     this->stop_line_existance_ = stop_line_existance;
    // }

    // bool getStopLineExistance() const{
    //     return this->stop_line_existance_;
    // }

    // 确定道路类型
    // void setLaneType(size_t lane_type){
    //     this->lane_type_ = lane_type;
    // }

    // size_t getLaneType() const{
    //     return this->lane_type_;
    // }

    // void setLaneTypeChangeDistance(double lane_type_changing_distance){
    //     this->lane_type_changing_distance_ = lane_type_changing_distance;
    // }

    // double getLaneTypeChangeDistance() const{
    //     return this->lane_type_changing_distance_;
    // }

    // --------------------The function below is added for behavior planning-----------------------
    
    // Calculate distance from a position to lane
    double calculateDistanceFromPosition(const Eigen::Matrix<double, 2, 1>& cur_pos) {
        // Get point2f
        PathPlanningUtilities::Point2f current_point;
        current_point.x_ = cur_pos(0);
        current_point.y_ = cur_pos(1);

        // Calculate the nearest point 
        // TODO: this function maybe invalid for long distance calculation, edit the calculation logic
        size_t nearest_point_index = Tools::findNearestPositionIndexInCoordination(lane_coorination_, current_point);
        PathPlanningUtilities::Point2f nearest_point = lane_coorination_[nearest_point_index].worldpos_.position_;

        double distance = sqrt(pow(current_point.x_ - nearest_point.x_, 2.0) + pow(current_point.y_ - nearest_point.y_, 2.0));

        return distance;
    }

    // Calculate target lane point from a position and a specified distance
    Eigen::Matrix<double, 2, 1> calculateTargetLanePosition(const Eigen::Matrix<double, 2, 1>& position, double distance) {
        // Calculate nearest lane index
        int position_x = position(0), position_y = position(1);
        size_t lane_point_index = findCurrenPositionIndexInLane(position_x, position_y);
        
        // Calculate target point 
        size_t target_lane_point_index = lane_point_index + static_cast<size_t>(distance / 0.1);

        std::cout << "Target lane point index: " << target_lane_point_index << std::endl;
        std::cout << "Distance: " << distance << std::endl;
        std::cout << "Lane point index: " << lane_point_index << std::endl;
        std::cout << "Lane whole length: " << lane_coorination_.size() << std::endl;

        PathPlanningUtilities::CoordinationPoint target_path_point = lane_coorination_[target_lane_point_index];
        Eigen::Matrix<double, 2, 1> target_point{target_path_point.worldpos_.position_.x_, target_path_point.worldpos_.position_.y_};

        return target_point;
    }

    // Find nearest lane point index from a position
    size_t findCurrenPositionIndexInLane(const Eigen::Matrix<double, 2, 1>& position) const {
        int position_x = position(0), position_y = position(1);
        return findCurrenPositionIndexInLane(position_x, position_y);
    }

    // Judge position whether in lane
    bool isInLane(const PathPlanningUtilities::Point2f& position) {
        // Calculate nearest pat point index and lane width
        size_t nearest_index = findCurrenPositionIndexInLane(position.x_, position.y_);
        PathPlanningUtilities::CoordinationPoint nearest_path_point = lane_coorination_[nearest_index];
        double lane_width = std::max(fabs(nearest_path_point.max_height_), fabs(nearest_path_point.min_height_));

        // Calculate distance
        double distance = sqrt(pow(position.x_ - nearest_path_point.worldpos_.position_.x_, 2.0) + pow(position.y_ - nearest_path_point.worldpos_.position_.y_, 2.0));

        if (distance <= lane_width) {
            return true;
        } else {
            return false;
        }
    }


 private:
    bool lane_existance_ = false;   // 道路是否可用
    std::vector<PathPlanningUtilities::CoordinationPoint> lane_coorination_;  // 道路完整信息
    std::vector<TransMatrix> lane_transmatrix_;  // 道路上每一点形成的坐标系，用于world与frenet转化
    PathPlanningUtilities::Path lane_center_path_in_world_;  // 道路中线在world系下的坐标
    PathPlanningUtilities::Path lane_center_path_in_frenet_;  // 道路中线在frenet系下的坐标
    bool is_guided_;  // 道路是否是被引导道路
    std::vector<double> lane_velocity_limitation_;  // 道路上每一点的速度上限
    std::vector<double> lane_lowest_velocity_;  // 道路上每一点的速度下限
    int turn_;  // 转向情况
    double priority_;  // 道路的优先级
    // size_t lane_type_; // 道路的类型
    // bool stop_line_existance_ = false; // 道路中是否存在停止线
    // double lane_type_changing_distance_; // 道路类型变换位置，也是停止线位置
    // int traffic_light_id_; // 道路中是否存在交通灯
    // int turn_direction_; // 道路的转向
};

#endif
