/*
 * @Author: fujiawei0724
 * @Date: 2021-12-20 17:01:13
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-07-14 15:47:01
 * @Description: Lane components
 */

#include "Common.hpp"

Lane::Lane() {}

Lane::~Lane() {}

// 将道路设置为可用状态
void Lane::enable() {
    this->lane_existance_ = true;
}

// 将道路设置为不可用状态
void Lane::disable() {
    this->lane_existance_ = false;
}

bool Lane::getLaneExistance() const {
    return this->lane_existance_;
}

// 生成道路中线信息(包括坐标系、中线在world系、中线在frenet系)(每一点的坐标系以前向为正，后向为负，左向为正，右向为负)
void Lane::generateLaneCenter(path_planning_msgs::BoundedCurve geometry) {
    std::vector<PathPlanningUtilities::CoordinationPoint>().swap(this->lane_coorination_);
    PathPlanningUtilities::Path().swap(this->lane_center_path_in_world_);
    PathPlanningUtilities::Path().swap(this->lane_center_path_in_frenet_);
    double station = 0.0;

    // // DEBUG
    // std::cout << "Lane points num: " << geometry.points.size() << std::endl;
    // // END DEBUG

    int valid_lane_points_num = std::min(static_cast<int>(geometry.points.size()), static_cast<int>(130.0 / static_cast<double>(LANE_GAP_DISTANCE)));
    for (int i = 0; i < valid_lane_points_num; i++) {
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
        this->lane_curve_.emplace_back(coordinate_point.worldpos_);
        this->lane_coorination_.push_back(coordinate_point);
        this->lane_center_path_in_world_.push_back(point_in_world);
        this->lane_center_path_in_frenet_.push_back(point_in_frenet);
    }
}

// 获取道路中线信息(包括坐标系、中线在world系、中线在frenet系)
const std::vector<PathPlanningUtilities::CoordinationPoint> &Lane::getLaneCoordnation() const {
    return this->lane_coorination_;
}

const PathPlanningUtilities::Path &Lane::getLaneCenterPathInWorld() const {
    return this->lane_center_path_in_world_;
}

const PathPlanningUtilities::Path &Lane::getLaneCenterPathInFrenet() const {
    return this->lane_center_path_in_frenet_;
}

// 确定道路中每一个点的变换矩阵
void Lane::generateLaneTransMatrix() {
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

const std::vector<TransMatrix> &Lane::getLaneTransMatrix() const {
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
void Lane::setLaneVelocityLimitation(const std::vector<double> &lane_velocity_limitation) {
    this->lane_velocity_limitation_ = lane_velocity_limitation;
}

const std::vector<double> &Lane::getLaneVelocityLimitation() const {
    return this->lane_velocity_limitation_;
}

// 确定道路最低速度
void Lane::setLowVelocity(const std::vector<double> &lane_lowest_velocity) {
    this->lane_lowest_velocity_ = lane_lowest_velocity;
}

const std::vector<double> &Lane::getLowVelocity() const {
    return this->lane_lowest_velocity_;
}

// 确定道路是否是被引导道路
void Lane::setGuidedFlag(bool is_guided) {
    this->is_guided_ = is_guided;
}

bool Lane::getGuidedFlag() const {
    return this->is_guided_;
}

// 找出当前位置在道路中对应的下标
// TODO: check this logic, maybe has problem
// Note that although this method may has some problem, in the new tyrajectory planning, this calculation method is not used
int Lane::findCurrenPositionIndexInLane(double position_x, double position_y) const {
    // size_t index = lane_coorination_.size() - 1;
    // PathPlanningUtilities::Point2f current_position;
    // current_position.x_ = position_x;
    // current_position.y_ = position_y;
    // for (size_t i = 0; i < this->lane_coorination_.size(); i++) {
    //     PathPlanningUtilities::Point2f local_position = Tools::calcNewCoordinationPosition(this->lane_coorination_[i].worldpos_, current_position);
    //     if (!Tools::isLarge(local_position.x_, 0.0)) {
    //         index = i;
    //         break;
    //     }
    // }
    // return index;
    Eigen::Matrix<double, 2, 1> pos{position_x, position_y};
    return findCurrenPositionIndexInLane(pos);
}

void Lane::setTurn(int turn) {
    this->turn_ = turn;
}

int Lane::getTurn() const {
    return this->turn_;
}

// 点是否在道路内
bool Lane::judgePoint2fInLane(const PathPlanningUtilities::Point2f &position, size_t start_index, size_t end_index, double width) const {
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
void Lane::setLanePriority(double priority) {
    this->priority_ = priority;
}

// 获取道路优先级
double Lane::getLanePriority() const {
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
double Lane::calculateDistanceFromPosition(const Eigen::Matrix<double, 2, 1>& cur_pos) {
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
Eigen::Matrix<double, 2, 1> Lane::calculateTargetLanePosition(const Eigen::Matrix<double, 2, 1>& position, double distance) {
    // Calculate nearest lane index
    int lane_point_index = findCurrenPositionIndexInLane(position);
    
    // Calculate target point 
    int target_lane_point_index = lane_point_index + static_cast<int>(distance / 0.1);
    if (target_lane_point_index >= static_cast<int>(lane_coorination_.size())) {
        // printf("[Lane] target position responses to the last point in the lane, with the gap index %d.\n", target_lane_point_index - static_cast<int>(lane_coorination_.size()) + 1);
        target_lane_point_index = static_cast<int>(lane_coorination_.size()) - 1;
    }
    PathPlanningUtilities::CoordinationPoint target_path_point = lane_coorination_[target_lane_point_index];
    Eigen::Matrix<double, 2, 1> target_point{target_path_point.worldpos_.position_.x_, target_path_point.worldpos_.position_.y_};

    return target_point;
}

// Find nearest lane point index from a position
// Calculation of distances to find the local minimum
int Lane::findCurrenPositionIndexInLane(const Eigen::Matrix<double, 2, 1>& position) const {

    // int index = -1;
    // bool res = kb_tree_.findNearestIndex(position, &index);
    // if (res) {
    //     return index;
    // } else {
    //     std:cout << "Find nearest point failed." << std::endl;
    //     return 0;
    // }

     
    // Using simple distance to calculate the nearest point
    double pre_distance = MAX_VALUE;
    double cur_distance = 0.0;
    int nearest_point_index = -1;
    for (int i = 0; i < static_cast<int>(lane_coorination_.size()); i++) {
        Eigen::Matrix<double, 2, 1> cur_lane_point{lane_coorination_[i].worldpos_.position_.x_, lane_coorination_[i].worldpos_.position_.y_};
        cur_distance = (position - cur_lane_point).norm();
        if (i > 0 && cur_distance > pre_distance) {
            nearest_point_index = i - 1;
            break;
        }
        if (i == static_cast<int>(lane_coorination_.size()) - 1) {
            // printf("[Lane] current position responses to the last point in the lane.\n");
            nearest_point_index = i;
            break;
        } 
        pre_distance = cur_distance;
    }

    return nearest_point_index;

    // std::function <double (double, double)> dis = [&](const double x, const double y) {return sqrt((x - position(0)) * (x - position(0)) + (y - position(1)) * (y - position(1)));};
    
    // // Binary search the nearest index in the trajectory from the ego vehicle position
    // // TODO: check this logic
    // int left = 0;
    // int right = static_cast<int>(lane_coorination_.size()) - 1;
    // while (left < right) {
    //     int mid = left + (right - left) / 2;
    //     if (mid == 0 || mid == static_cast<int>(lane_coorination_.size()) - 1) {
    //         break;
    //     }
    //     if (dis(lane_coorination_[mid].worldpos_.position_.x_, lane_coorination_[mid].worldpos_.position_.y_) >= dis(lane_coorination_[mid - 1].worldpos_.position_.x_, lane_coorination_[mid - 1].worldpos_.position_.y_)) {
    //         right = mid;
    //     } else {
    //         left = mid + 1;
    //     }
    // }
    // return left;
}

// Judge position whether in lane
bool Lane::isInLane(const PathPlanningUtilities::Point2f& position) {
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

// Pre-process, judge whether a lane is occupied by static obstacle and virtual traffic rule obstacle
bool Lane::isLaneOccupiedByStaticObs(const Eigen::Matrix<double, 2, 1>& position, const std::vector<Obstacle>& all_obs, const std::vector<vec_map_cpp_msgs::VirtualObstacle> &traffic_virtual_obs) {
    // Get valid lane info
    int vehicle_index = findCurrenPositionIndexInLane(position);
    std::vector<PathPlanningUtilities::CurvePoint> valid_lane_curve(lane_curve_.begin() + vehicle_index, lane_curve_.end());
    // Construct occupation area of lane
    DecisionMaking::RSS::OccupationArea lane_occupation_area = DecisionMaking::RSS::OccupationArea(lane_curve_, 1.95, 5.0);

    // Construct occupation area for static obs
    for (const auto& obs : all_obs) {
        // Only static obs
        if (Tools::isEqual(obs.velocity_, 0.0)) {
            DecisionMaking::RSS::OccupationArea obs_occupation_area = DecisionMaking::RSS::OccupationArea(obs, 0, OBSTACLE_OCCUPANCY_AREA_SAMPLING_GAP_FOR_STATIC_OBSTACLE);
            size_t subvehicle_interact_index, obstacle_interact_index;
            if (DecisionMaking::RSS::occupationInteractionJudgement(lane_occupation_area, obs_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                return true;
            }
        }
    }

    // Construct occupation area for traffic ruel virtual obs
    for (const auto& traffic_obs : traffic_virtual_obs) {
        DecisionMaking::RSS::OccupationArea traffic_rule_obstacle_occupation_area = DecisionMaking::RSS::OccupationArea(traffic_obs);
        size_t subvehicle_interact_index, obstacle_interact_index;
        // 判断两占用区域是否相交
        if (occupationInteractionJudgement(lane_occupation_area, traffic_rule_obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {

            // // DEBUG
            // printf("Collision with traffic virtual obstacles.\n");
            // std::cout << "subvehicle_interact_index: " << subvehicle_interact_index << std::endl;
            // std::cout << "obstacle_interact_index: " << obstacle_interact_index << std::endl;
            // // END DEBUG

            return true;
        }
    }

    return false;

}





/**
 * @brief speed up calculation
 * @param {double&} remain_station
 * @param {double*} pos_x
 * @param {double*} pos_y
 * @param {double*} theta
 * @param {double*} curvature
 * @return {*}
 */    
void QuinticSpline::calculatePointInfo(const PathPlanningUtilities::CurvePoint& begin_state, const PathPlanningUtilities::CurvePoint& end_state, const double& real_dis, const double& remain_station, double* pos_x, double* pos_y, double* theta, double* curvature) {
    double l = sqrt((begin_state.position_.x_ - end_state.position_.x_) * (begin_state.position_.x_ - end_state.position_.x_) + (begin_state.position_.y_ - end_state.position_.y_) * (begin_state.position_.y_ - end_state.position_.y_));
    //Initialize parameters
    double ax, bx, cx, dx, ex, fx, ay, by, cy, dy, ey, fy;
    double p0x, p0y, t0x, t0y, k0x, k0y;
    p0x = begin_state.position_.x_;
    p0y = begin_state.position_.y_;
    t0x = cos(begin_state.theta_);
    t0y = sin(begin_state.theta_);
    k0x = -begin_state.kappa_*sin(begin_state.theta_);
    k0y = begin_state.kappa_*cos(begin_state.theta_);
    //Finish parameter initialization
    double p1x, p1y, t1x, t1y, k1x, k1y;
    p1x = end_state.position_.x_;
    p1y = end_state.position_.y_;
    t1x = cos(end_state.theta_);
    t1y = sin(end_state.theta_);
    k1x = -end_state.kappa_*sin(end_state.theta_);
    k1y = end_state.kappa_*cos(end_state.theta_);
    //Generate parameters
    fx = +(                   p0x );
    ex = +(                   t0x ) * l;
    dx = +(                   k0x ) * l * l / 2.0f;
    cx = +( + 10 * p1x - 10 * p0x )
            +( -  4 * t1x -  6 * t0x ) * l
            +( +      k1x -  3 * k0x ) * l * l / 2.0f;
    bx = +( - 15 * p1x + 15 * p0x )
            +( +  7 * t1x +  8 * t0x ) * l
            +( -  2 * k1x +  3 * k0x ) * l * l / 2.0f;
    ax = +( +  6 * p1x -  6 * p0x )
            +( -  3 * t1x -  3 * t0x ) * l
            +( +      k1x -      k0x ) * l * l / 2.0f;
    fy = +(                   p0y );
    ey = +(                   t0y ) * l;
    dy = +(                   k0y ) * l * l / 2.0f;
    cy = +( + 10 * p1y - 10 * p0y )
            +( -  4 * t1y -  6 * t0y ) * l
            +( +      k1y -  3 * k0y ) * l * l / 2.0f;
    by = +( - 15 * p1y + 15 * p0y )
            +( +  7 * t1y +  8 * t0y ) * l
            +( -  2 * k1y +  3 * k0y ) * l * l / 2.0f;
    ay = +( +  6 * p1y -  6 * p0y )
            +( -  3 * t1y -  3 * t0y ) * l
            +( +      k1y -      k0y ) * l * l / 2.0f;

    double cur_l = remain_station / real_dis * l;
    // Calculate parameters
    if (cur_l < 0.0) {
        cur_l = 0.0;
    } else if (cur_l > 1.0) {
        cur_l = 1.0;
    }
    std::vector<double> coefficients(5, 0.0);
    for (int i = 0; i < 5; i++) {
        coefficients[i] = powf(cur_l / l, i + 1.0f);
    }
    *pos_x = ax * coefficients[4] + bx * coefficients[3] + cx * coefficients[2] + dx * coefficients[1] + ex * coefficients[0] + fx;
    *pos_y = ay * coefficients[4] + by * coefficients[3] + cy * coefficients[2] + dy * coefficients[1] + ey * coefficients[0] + fy;
    double vx = ax * coefficients[3] * 5 + bx * coefficients[2] * 4 + cx * coefficients[1] * 3 + dx * coefficients[0] * 2 + ex;
    double vy = ay * coefficients[3] * 5 + by * coefficients[2] * 4 + cy * coefficients[1] * 3 + dy * coefficients[0] * 2 + ey;
    *theta = atan2(vy, vx);
    double acc_x = ax * coefficients[2] * 20 + bx * coefficients[1] * 12 + cx * coefficients[0] * 6 + dx * 2;
    double acc_y = ay * coefficients[2] * 20 + by * coefficients[1] * 12 + cy * coefficients[0] * 6 + dy * 2;
    *curvature = (acc_y*vx-acc_x*vy)/pow((vx*vx+vy*vy), 1.5);
}



/**
 * @brief calculate the point information given a s
 * @return the corresponding s
 */
PathPlanningUtilities::CurvePoint QuinticSpline::calculateCurvePoint(const Eigen::Matrix<double, 1, 12>& current_coefficients, const double& l, const double& s) {
    
    if (s < 0.0 || s > l) {
        printf("[QuinticSpline] s is out of range.\n");
    }
    
    // Parse parameters
    double a0, a1, a2, a3, a4, a5, b0, b1, b2, b3, b4, b5;
    a0 = current_coefficients(0);
    a1 = current_coefficients(1);
    a2 = current_coefficients(2);
    a3 = current_coefficients(3);
    a4 = current_coefficients(4);
    a5 = current_coefficients(5);
    b0 = current_coefficients(6);
    b1 = current_coefficients(7);
    b2 = current_coefficients(8);
    b3 = current_coefficients(9);
    b4 = current_coefficients(10);
    b5 = current_coefficients(11);

    std::array<double, 5> s_pows{s, pow(s,2), pow(s,3), pow(s,4), pow(s,5)};
    PathPlanningUtilities::CurvePoint target_point;
    target_point.position_.x_ = a0 + a1 * s + a2 * s_pows[1] + a3 * s_pows[2] + a4 * s_pows[3] + a5 * s_pows[4];
    target_point.position_.y_ = b0 + b1 * s + b2 * s_pows[1] + b3 * s_pows[2] + b4 * s_pows[3] + b5 * s_pows[4];
    double dx = a1 + 2 * a2 * s + 3 * a3 * s_pows[1] + 4 * a4 * s_pows[2] + 5 * a5 * s_pows[3];
    double dy = b1 + 2 * b2 * s + 3 * b3 * s_pows[1] + 4 * b4 * s_pows[2] + 5 * b5 * s_pows[3];
    double ddx = 2 * a2 + 6 * a3 * s + 12 * a4 * s_pows[1] + 20 * a5 * s_pows[2];
    double ddy = 2 * b2 + 6 * b3 * s + 12 * b4 * s_pows[1] + 20 * b5 * s_pows[2];
    target_point.theta_ = atan2(dy, dx);
    target_point.kappa_ = (ddy*dx-ddx*dy)/pow((dx*dx+dy*dy), 1.5);
    
    return target_point;
}

/**
 * @brief calculate the point information given a s
 * @return the corresponding s
 */
Eigen::Vector2d QuinticSpline::calculatePoint(const Eigen::Matrix<double, 1, 12>& current_coefficients, const double& l, const double& s) {
    if (s < 0.0 || s > l) {
        printf("[QuinticSpline] s is out of range.\n");
    }
    
    // Parse parameters
    double a0, a1, a2, a3, a4, a5, b0, b1, b2, b3, b4, b5;
    a0 = current_coefficients(0);
    a1 = current_coefficients(1);
    a2 = current_coefficients(2);
    a3 = current_coefficients(3);
    a4 = current_coefficients(4);
    a5 = current_coefficients(5);
    b0 = current_coefficients(6);
    b1 = current_coefficients(7);
    b2 = current_coefficients(8);
    b3 = current_coefficients(9);
    b4 = current_coefficients(10);
    b5 = current_coefficients(11);

    std::array<double, 5> s_pows{s, pow(s,2), pow(s,3), pow(s,4), pow(s,5)};
    Eigen::Vector2d target_point;
    target_point(0) = a0 + a1 * s + a2 * s_pows[1] + a3 * s_pows[2] + a4 * s_pows[3] + a5 * s_pows[4];
    target_point(1) = b0 + b1 * s + b2 * s_pows[1] + b3 * s_pows[2] + b4 * s_pows[3] + b5 * s_pows[4];
    
    return target_point;
}

/**
 * @brief calculate the corresponding arc length given a position
 * @param {double&} l: whole length
 * @param {Vector2d&} pos: query position
 * @return projected arc length
 */    
double QuinticSpline::calculateArcLength(const Eigen::Matrix<double, 1, 12>& current_coefficients, const double& l, const Eigen::Vector2d& pos) {
    // Parse parameters
    double a0, a1, a2, a3, a4, a5, b0, b1, b2, b3, b4, b5;
    a0 = current_coefficients(0);
    a1 = current_coefficients(1);
    a2 = current_coefficients(2);
    a3 = current_coefficients(3);
    a4 = current_coefficients(4);
    a5 = current_coefficients(5);
    b0 = current_coefficients(6);
    b1 = current_coefficients(7);
    b2 = current_coefficients(8);
    b3 = current_coefficients(9);
    b4 = current_coefficients(10);
    b5 = current_coefficients(11);

    double x0 = pos(0);
    double y0 = pos(1);

    double s = l;
    double f_s, f_d_s;
    static constexpr int max_iter_num = 5;
    for (int i = 0; i < max_iter_num; i++) {
        f_s = -a0*a1 - b0*b1 + (-3*pow(a3,2) - 6*a2*a4 - 6*a1*a5 - 3*pow(b2,2) - 6*b2*b3 - 3*pow(b3,2) - 6*b1*b5)*pow(s,5) + (-7*a3*a4 - 7*a2*a5 - 7*b2*b4 - 7*b3*b4)*pow(s,6) + (-4*pow(a4,2) - 8*a3*a5 - 4*pow(b4,2) - 8*b2*b5 - 8*b3*b5)*pow(s,7) + (-9*a4*a5 - 9*b4*b5)*pow(s,8) + (-5*pow(a5,2) - 5*pow(b5,2))*pow(s,9) + a1*x0 + s*(-pow(a1,2) - 2*a0*a2 - b1*2 + 2*a2*x0) + b1*y0 + pow(s,2)*(-3*a1*a2 - 3*a0*a3 - 3*b0*b2 - 3*b0*b3 + 3*a3*x0 + 3*b2*y0 + 3*b3*y0) + pow(s,3)*(-2*pow(a2,2) - 4*a1*a3 - 4*a0*a4 - 4*b1*b2 - 4*b1*b3 - 4*b0*b4 + 4*a4*x0 + 4*b4*y0) + pow(s,4)*(-5*a2*a3 - 5*a1*a4 - 5*a0*a5 - 5*b1*b4 - 5*b0*b5 + 5*a5*x0 + 5*b5*y0);
        f_d_s = -pow(a1,2) - 2*a0*a2 - pow(b1,2) + (-15*pow(a3,2) - 30*a2*a4 - 30*a1*a5 - 15*pow(b2,2) - 30*b2*b3 - 15*pow(b3,2) - 30*b1*b5)*pow(s,4) + (-42*a3*a4 - 42*a2*a5 - 42*b2*b4 - 42*b3*b4)*pow(s,5) + (-28*pow(a4,2) - 56*a3*a5 - 28*pow(b4,2) - 56*b2*b5 - 56*b3*b5)*pow(s,6) + (-72*a4*a5 - 72*b4*b5)*pow(s,7) + (-45*pow(a5,2) - 45*pow(b5,2))*pow(s,8) + 2*a2*x0 + s*(-6*a1*a2 - 6*a0*a3 - 6*b0*b2 - 6*b0*b3 + 6*a3*x0 + 6*b2*y0 + 6*b3*y0) + pow(s,2)*(-6*pow(a2,2) - 12*a1*a3 - 12*a0*a4 - 12*b1*b2 - 12*b1*b3 - 12*b0*b4 + 12*a4*x0 + 12*b4*y0) + pow(s,3)*(-20*a2*a3 - 20*a1*a4 - 20*a0*a5 - 20*b1*b4 - 20*b0*b5 + 20*a5*x0 + 20*b5*y0);
        s = s - f_s / f_d_s;
        if (s > l) s = l;
        if (s < 0.0) s = l;
    }

    return s;
}

// Constructor
ParametricLane::ParametricLane() = default;
// Destructor
ParametricLane::~ParametricLane() = default;

/**
 * @description: generate the whole information oif the lane
 * @param {BoundedCurve} geometry: key points of the lane
 * @param {vector<double>} stations: distances in the continuous points, TODO: replace this with a message from map 
 * @param {vector<std::array<double, 12>>} coefficients: coefficients for each segment of the whole piecewise quintic spline, TODO: replace this with a message from map
 */
void ParametricLane::generateLaneCenter(path_planning_msgs::BoundedCurve geometry, std::vector<double> gaps, std::vector<std::array<double, 12>> coefficients) {
    // Supply data for gaps and stations 
    n_ = gaps.size() + 1;
    gaps_ = gaps;
    stations_.resize(n_, 0.0);
    for (int i = 1; i < n_; i++) {
        stations_[i] = stations_[i - 1] + gaps[i - 1];
    }

    // Supply data for coefficients
    coefficients_.resize(n_ - 1, 12);
    for (int i = 0; i < n_ - 1; i++) {
        for (int j = 0; j < 12; j++) {
            coefficients_(i, j) = coefficients[i][j];
        }
    }

    // Supply sparse center line point information
    // TODO: check the necessity of this operation
    lane_coorination_.resize(n_);
    points_.resize(n_);
    for (int i = 0; i < n_; i++) {
        PathPlanningUtilities::CoordinationPoint coordinate_point;
        coordinate_point.worldpos_.position_.x_ = geometry.points[i].center_point.x;
        coordinate_point.worldpos_.position_.y_ = geometry.points[i].center_point.y;
        coordinate_point.worldpos_.theta_ = geometry.points[i].center_point.theta;
        coordinate_point.worldpos_.kappa_ = geometry.points[i].center_point.kappa;
        coordinate_point.station_ = stations_[i];
        coordinate_point.min_height_ = std::min(-0.1, -geometry.points[i].right_distance);
        coordinate_point.max_height_ = std::max(0.1, geometry.points[i].left_distance);
        lane_coorination_[i] = coordinate_point;
        points_[i] = {geometry.points[i].center_point.x, geometry.points[i].center_point.y};
    }
}

/**
 * @description: transform a point from world to frenet (only containing abscissa and ordinate)
 * @return point position in frenet frame
 */    
Eigen::Vector2d ParametricLane::calculateFrenetPoint(const Eigen::Vector2d& pos) {
    // Calculate the nearest key point roughly
    int target_index = calculateNearestScatterPointIndex(pos);
    // Transform position
    PathPlanningUtilities::Point2f pos_point;
    pos_point.x_ = pos(0);
    pos_point.y_ = pos(1);
    // Modify the index to find the previous point
    if (target_index > 0) {
        PathPlanningUtilities::Point2f transformed_point = Tools::calcNewCoordinationPosition(lane_coorination_[target_index].worldpos_, pos_point);
        if (transformed_point.x_ < 0.0) {
            target_index -= 1;
        }
    }

    // Get origin arc length 
    double arc_length = stations_[target_index];
    // Get extra arc length
    double extra_arc_length = QuinticSpline::calculateArcLength(coefficients_.row(target_index), gaps_[target_index], pos);
    arc_length += extra_arc_length;

    // Get the corresponding curve point
    PathPlanningUtilities::CurvePoint corresponding_point = QuinticSpline::calculateCurvePoint(coefficients_.row(target_index), gaps_[target_index], extra_arc_length);

    // Calculate ordinate in frenet frame
    PathPlanningUtilities::Point2f transformed_cur_pos = Tools::calcNewCoordinationPosition(corresponding_point, pos_point);
    double lateral_offset = transformed_cur_pos.y_;

    Eigen::Vector2d frenet_pos{arc_length, lateral_offset};

    return frenet_pos;
}

/**
 * @description: transform a point from frenet to world (only containing abscissa and ordinate)
 * @return point position in world frame
 */ 
Eigen::Vector2d ParametricLane::calculateWorldPoint(const Eigen::Vector2d& frenet_pos) {
    // // Get the target segment
    // int target_segment_index = std::lower_bound(stations_.begin(), stations_.end(), frenet_pos(0)) - stations_.begin();
    // if (target_segment_index == static_cast<int>(stations_.size()) - 1) {
    //     printf("[Lane] frenet position is out of the lane.\n");
    //     target_segment_index -= 1;
    // }

    // // Calculate the corresponding point in lane
    // double remain_station = frenet_pos(0) - stations_[target_segment_index];
    // PathPlanningUtilities::CurvePoint lane_curve_point = QuinticSpline::calculateCurvePoint(coefficients_.row(target_segment_index), gaps_[target_segment_index], remain_station);
    PathPlanningUtilities::CurvePoint tangent_point = calculateCurvePointFromArcLength(frenet_pos(0));

    Eigen::Vector2d lane_pos{tangent_point.position_.x_, tangent_point.position_.y_};

    // Get tangent vector and normal vector, the norm is set with 1.0
    double lane_orientation = tangent_point.theta_;
    double y = tan(lane_orientation);
    double x = 1.0;
    Eigen::Vector2d lane_tangent_vec{x, y};
    lane_tangent_vec /= lane_tangent_vec.norm();
    Eigen::Vector2d vec_normal{-lane_tangent_vec(1), lane_tangent_vec(0)};
    Eigen::Vector2d point = vec_normal * frenet_pos(1) + lane_pos;
    return point;
}

/**
 * @description: calculate the nearest in the lane given a position
 * @return {*}
 */    
PathPlanningUtilities::CurvePoint ParametricLane::findNearestPoint(const Eigen::Vector2d& pos) {
    // Calculate the nerest key point roughly
    int target_index = calculateNearestScatterPointIndex(pos);

    PathPlanningUtilities::CurvePoint nearest_point;
    // Get the nearest point for different situations
    if (target_index == 0) {
        double s = QuinticSpline::calculateArcLength(coefficients_.row(0), gaps_[0], pos);
        nearest_point = QuinticSpline::calculateCurvePoint(coefficients_.row(0), gaps_[0], s);
        return nearest_point;
    } else if (target_index == n_ - 1) {
        double s = QuinticSpline::calculateArcLength(coefficients_.row(n_ - 2), gaps_[n_ - 2], pos);
        nearest_point = QuinticSpline::calculateCurvePoint(coefficients_.row(n_ - 2), gaps_[n_ - 2], s);
        return nearest_point;
    } else {
        double pre_s = QuinticSpline::calculateArcLength(coefficients_.row(target_index - 1), gaps_[target_index - 1], pos);
        PathPlanningUtilities::CurvePoint pre_nearest_point = QuinticSpline::calculateCurvePoint(coefficients_.row(target_index - 1), gaps_[target_index - 1], pre_s);
        double next_s = QuinticSpline::calculateArcLength(coefficients_.row(target_index), gaps_[target_index], pos);
        PathPlanningUtilities::CurvePoint next_nearest_point = QuinticSpline::calculateCurvePoint(coefficients_.row(target_index), gaps_[target_index], next_s);
        if (sqrt(pow(pre_nearest_point.position_.x_ - pos(0), 2) + pow(pre_nearest_point.position_.y_ - pos(1), 2)) <= sqrt(pow(next_nearest_point.position_.x_ - pos(0), 2) + pow(next_nearest_point.position_.y_ - pos(1), 2))) {
            return pre_nearest_point;
        } else {
            return next_nearest_point;
        }
    }

}

/**
 * @description: calculate nearest key points index with binary search
 * @return the corresponding index
 */    
int ParametricLane::calculateNearestScatterPointIndex(const Eigen::Vector2d& pos) {
    int left = 0, right = n_ - 1;
    while (left <= right) {
        int mid = left + (right - left) / 2;
        if ((points_[mid] - pos).norm() >= (points_[mid - 1] - pos).norm()) {
            right = mid - 1;
        } else {
            left = mid + 1;
        }
    }
    return right;
}

/**
 * @description: find the distance given a position
 * @return nearest distance
 */ 
double ParametricLane::calculateDistanceFromPosition(const Eigen::Vector2d& cur_pos) {
    PathPlanningUtilities::CurvePoint nearest_point = findNearestPoint(cur_pos);
    double distance = sqrt(pow(nearest_point.position_.x_ - cur_pos(0), 2) + pow(nearest_point.position_.y_ - cur_pos(1), 2));

    return distance;
}

/**
 * @description: calculate the extend point in the lane given a position and a distance
 * @param {double} distance
 * @return {*}
 */    
Eigen::Vector2d ParametricLane::calculateTargetLanePosition(const Eigen::Vector2d& pos, double distance) {
    // Calculate the nearest key point roughly
    int target_index = calculateNearestScatterPointIndex(pos);
    // Transform position
    PathPlanningUtilities::Point2f pos_point;
    pos_point.x_ = pos(0);
    pos_point.y_ = pos(1);
    // Modify the index to find the previous point
    if (target_index > 0) {
        PathPlanningUtilities::Point2f transformed_point = Tools::calcNewCoordinationPosition(lane_coorination_[target_index].worldpos_, pos_point);
        if (transformed_point.x_ < 0.0) {
            target_index -= 1;
        }
    }

    // Get origin arc length 
    double arc_length = stations_[target_index];
    // Get extra arc length
    double extra_arc_length = QuinticSpline::calculateArcLength(coefficients_.row(target_index), gaps_[target_index], pos);
    arc_length += extra_arc_length;

    // Get arc length of target point
    double target_arc_length = arc_length + distance;
    // Get segment of target point
    int target_point_segment = std::lower_bound(stations_.begin(), stations_.end(), target_arc_length) - stations_.begin();
    if (target_point_segment >= n_ - 1) {
        printf("[Lane] target point is out of range.\n");
        return points_[n_ - 1];
    } 
    // Get truncate arc length
    double truncate_arc_length = target_arc_length - stations_[target_point_segment];
    // Get the final point
    Eigen::Vector2d target_point = QuinticSpline::calculatePoint(coefficients_.row(target_point_segment), gaps_[target_point_segment], truncate_arc_length);

    return target_point;
}

/**
 * @description: judge whether the given point is in the lane
 * @param {Point2f&} query point
 */    
bool ParametricLane::isInLane(const PathPlanningUtilities::Point2f& position) {
    // Parse input
    Eigen::Vector2d pos{position.x_, position.y_};
    // Get lane width from the nearest key point
    int nearest_key_point_index = calculateNearestScatterPointIndex(pos);
    PathPlanningUtilities::CoordinationPoint nearest_path_point = lane_coorination_[nearest_key_point_index];
    double threshold_dis = std::max(fabs(nearest_path_point.max_height_), fabs(nearest_path_point.min_height_));

    // Calculate distance 
    double distance = calculateDistanceFromPosition(pos);

    return distance > threshold_dis ? false : true;

}

/**
 * @description: pre-process, judge whether a lane is occupied by static obstacle and virtual traffic rule obstacle
 * @param {position} vehicle position
 * @param {all_obs} obstacles provided by perception
 * @param {traffic_vistual_obs} obstacles provided by map
 * @return whether the lane is occupied 
 */    
bool ParametricLane::isLaneOccupiedByStaticObs(const Eigen::Matrix<double, 2, 1>& position, const std::vector<Obstacle>& all_obs, const std::vector<vec_map_cpp_msgs::VirtualObstacle> &traffic_virtual_obs) {
    // Get the start arc length from the current position of the vehicle
    double start_arc_length = calculateArcLength(position);
    // Get the sampled curve points with a gap 5.0m
    std::vector<PathPlanningUtilities::CurvePoint> sampled_points;
    for (double cur_arc_length = start_arc_length; cur_arc_length <= stations_[n_ - 1]; cur_arc_length += 5.0) {
        sampled_points.emplace_back(calculateCurvePointFromArcLength(cur_arc_length));
    }
    // Construct occupation area of lane
    DecisionMaking::RSS::OccupationArea lane_occupation_area = DecisionMaking::RSS::OccupationArea(sampled_points, 3.0, 5.0, 1);
    // Construct occupation area for static obs
    for (const auto& obs : all_obs) {
        // Only static obs
        if (Tools::isEqual(obs.velocity_, 0.0)) {
            DecisionMaking::RSS::OccupationArea obs_occupation_area = DecisionMaking::RSS::OccupationArea(obs, 0, OBSTACLE_OCCUPANCY_AREA_SAMPLING_GAP_FOR_STATIC_OBSTACLE);
            size_t subvehicle_interact_index, obstacle_interact_index;
            if (DecisionMaking::RSS::occupationInteractionJudgement(lane_occupation_area, obs_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                return true;
            }
        }
    }

    // Construct occupation area for traffic ruel virtual obs
    for (const auto& traffic_obs : traffic_virtual_obs) {
        DecisionMaking::RSS::OccupationArea traffic_rule_obstacle_occupation_area = DecisionMaking::RSS::OccupationArea(traffic_obs);
        size_t subvehicle_interact_index, obstacle_interact_index;
        // 判断两占用区域是否相交
        if (occupationInteractionJudgement(lane_occupation_area, traffic_rule_obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {

            // // DEBUG
            // printf("Collision with traffic virtual obstacles.\n");
            // std::cout << "subvehicle_interact_index: " << subvehicle_interact_index << std::endl;
            // std::cout << "obstacle_interact_index: " << obstacle_interact_index << std::endl;
            // // END DEBUG

            return true;
        }
    }

    return false;

}

/**
 * @description: calculate the corresponding arc length given a position
 * @param {pos} query position
 * @return {*}
 */
double ParametricLane::calculateArcLength(const Eigen::Vector2d& pos) {
    // Calculate the nearest key point roughly
    int target_index = calculateNearestScatterPointIndex(pos);
    // Transform position
    PathPlanningUtilities::Point2f pos_point;
    pos_point.x_ = pos(0);
    pos_point.y_ = pos(1);
    // Modify the index to find the previous point
    if (target_index > 0) {
        PathPlanningUtilities::Point2f transformed_point = Tools::calcNewCoordinationPosition(lane_coorination_[target_index].worldpos_, pos_point);
        if (transformed_point.x_ < 0.0) {
            target_index -= 1;
        }
    }

    // Get origin arc length 
    double arc_length = stations_[target_index];
    // Get extra arc length
    double extra_arc_length = QuinticSpline::calculateArcLength(coefficients_.row(target_index), gaps_[target_index], pos);
    arc_length += extra_arc_length;

    return arc_length;
}

/**
 * @description: get the corresponding curve point from arc length
 * @param {arc_length} query arc length
 * @return {*}
 */
PathPlanningUtilities::CurvePoint ParametricLane::calculateCurvePointFromArcLength(const double& arc_length) {
    // Get the target segment
    int target_segment_index = std::lower_bound(stations_.begin(), stations_.end(), arc_length) - stations_.begin();
    if (target_segment_index == static_cast<int>(stations_.size()) - 1) {
        printf("[Lane] frenet position is out of the lane.\n");
        target_segment_index -= 1;
    }

    // Calculate the corresponding point in lane
    double remain_station = arc_length - stations_[target_segment_index];
    PathPlanningUtilities::CurvePoint lane_curve_point = QuinticSpline::calculateCurvePoint(coefficients_.row(target_segment_index), gaps_[target_segment_index], remain_station);
    return lane_curve_point;
}
