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
    // TODO: check this logic, maybe has problem
    // Note that although this method may has some problem, in the new tyrajectory planning, this calculation method is not used
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
        PathPlanningUtilities::CoordinationPoint target_path_point = lane_coorination_[target_lane_point_index];
        Eigen::Matrix<double, 2, 1> target_point{target_path_point.worldpos_.position_.x_, target_path_point.worldpos_.position_.y_};

        return target_point;
    }

    // Find nearest lane point index from a position
    // Calculation of distances to find the local minimum
    int findCurrenPositionIndexInLane(const Eigen::Matrix<double, 2, 1>& position) const {
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
                // TODO: add logic to handle this situation 
                printf("[Lane] current position responses to the last point in the lane.\n");
            } 
            pre_distance = cur_distance;
        }

        return nearest_point_index;
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

// A segment of quintic spline
class QuinticSpline {
 public:
    QuinticSpline() = default;
    // TODO: check if the arc length could bev add here to replace the points distance
    QuinticSpline(const PathPlanningUtilities::CurvePoint& begin_state, const PathPlanningUtilities::CurvePoint& end_state, const double& real_dis) {
        double l = sqrt((begin_state.position_.x_ - end_state.position_.x_) * (begin_state.position_.x_ - end_state.position_.x_) + (begin_state.position_.y_ - end_state.position_.y_) * (begin_state.position_.y_ - end_state.position_.y_));
        l_ = l;
        real_dis_ = real_dis;
        //Initialize parameters
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
		fx_ = +(                   p0x );
		ex_ = +(                   t0x ) * l;
		dx_ = +(                   k0x ) * l * l / 2.0f;
        cx_ = +( + 10 * p1x - 10 * p0x )
              +( -  4 * t1x -  6 * t0x ) * l
              +( +      k1x -  3 * k0x ) * l * l / 2.0f;
        bx_ = +( - 15 * p1x + 15 * p0x )
              +( +  7 * t1x +  8 * t0x ) * l
              +( -  2 * k1x +  3 * k0x ) * l * l / 2.0f;
        ax_ = +( +  6 * p1x -  6 * p0x )
              +( -  3 * t1x -  3 * t0x ) * l
              +( +      k1x -      k0x ) * l * l / 2.0f;
        fy_ = +(                   p0y );
        ey_ = +(                   t0y ) * l;
        dy_ = +(                   k0y ) * l * l / 2.0f;
        cy_ = +( + 10 * p1y - 10 * p0y )
              +( -  4 * t1y -  6 * t0y ) * l
              +( +      k1y -  3 * k0y ) * l * l / 2.0f;
        by_ = +( - 15 * p1y + 15 * p0y )
              +( +  7 * t1y +  8 * t0y ) * l
              +( -  2 * k1y +  3 * k0y ) * l * l / 2.0f;
        ay_ = +( +  6 * p1y -  6 * p0y )
              +( -  3 * t1y -  3 * t0y ) * l
              +( +      k1y -      k0y ) * l * l / 2.0f;
    }
    ~QuinticSpline() = default;

    /**
     * @brief calculate nearest point information
     * @param {*}
     * @return {*}
     */ 
    void calculateNearestInfo() {

    }  

    /**
     * @brief calculate point information given parameter
     * @param {*}
     * @return {*}
     * TODO: check this transformation logic, add curvature slope transform here
     */
    void calculatePointInfo(const double& remain_station, double* pos_x, double* pos_y, double* theta, double* curvature) {
        double cur_l = remain_station / real_dis_ * l_;
        // Calculate parameters
        assert(cur_l >= 0 && cur_l <= l_);
        std::vector<double> coefficients(5, 0.0);
        for (int i = 0; i < 5; i++) {
            coefficients[i] = powf(cur_l / l_, i + 1.0f);
        }
        *pos_x = ax_ * coefficients[4] + bx_ * coefficients[3] + cx_ * coefficients[2] + dx_ * coefficients[1] + ex_ * coefficients[0] + fx_;
        *pos_y = ay_ * coefficients[4] + by_ * coefficients[3] + cy_ * coefficients[2] + dy_ * coefficients[1] + ey_ * coefficients[0] + fy_;
        double vx = ax_ * coefficients[3] * 5 + bx_ * coefficients[2] * 4 + cx_ * coefficients[1] * 3 + dx_ * coefficients[0] * 2 + ex_;
        double vy = ay_ * coefficients[3] * 5 + by_ * coefficients[2] * 4 + cy_ * coefficients[1] * 3 + dy_ * coefficients[0] * 2 + ey_;
        *theta = atan2(vy, vx);
        double ax = ax_ * coefficients[2] * 20 + bx_ * coefficients[1] * 12 + cx_ * coefficients[0] * 6 + dx_ * 2;
        double ay = ay_ * coefficients[2] * 20 + by_ * coefficients[1] * 12 + cy_ * coefficients[0] * 6 + dy_ * 2;
        *curvature = (ay*vx-ax*vy)/pow((vx*vx+vy*vy), 1.5);
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
    static void calculatePointInfo(const PathPlanningUtilities::CurvePoint& begin_state, const PathPlanningUtilities::CurvePoint& end_state, const double& real_dis, const double& remain_station, double* pos_x, double* pos_y, double* theta, double* curvature) {
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
    


    double real_dis_{0.0};
    double l_{0.0};
    double ax_{0.0}, bx_{0.0}, cx_{0.0}, dx_{0.0}, ex_{0.0}, fx_{0.0};
    double ay_{0.0}, by_{0.0}, cy_{0.0}, dy_{0.0}, ey_{0.0}, fy_{0.0};
};

class QuinticSplinePiecewise {
 public: 
    QuinticSplinePiecewise(const std::vector<PathPlanningUtilities::CurvePoint>& curve_points, const double& real_dis) {
        segment_num_ = static_cast<int>(curve_points.size()) - 1;
        quintic_segments_.resize(segment_num_);
        arc_length_domain_.resize(segment_num_ + 1);
        for (int i = 0; i < segment_num_; i++) {
            quintic_segments_[i] = QuinticSpline(curve_points[i], curve_points[i + 1], real_dis);
            arc_length_domain_[i] = i * real_dis;
        }
        arc_length_domain_[segment_num_] = segment_num_ * real_dis;
    }
    ~QuinticSplinePiecewise() = default;

    /**
     * @brief calculate the nearest point in the spline from a specific point
     * @param {*}
     * @return {*}
     */    
    void calculateNearestInfo(const Eigen::Matrix<double, 2, 1>& location, const std::pair<int, int>& initial_guess) {

    }

    /**
     * @brief calculate information from a given arc length
     * @param {*}
     * @return {*}
     */   


    int segment_num_{0};
    std::vector<double> arc_length_domain_{};
    std::vector<QuinticSpline> quintic_segments_{};
};

#endif
