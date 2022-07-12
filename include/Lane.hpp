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
#include "Obstacle.hpp"
#include "Tools.hpp"
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/kdtree/kdtree_flann.h>

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

    Lane();

    ~Lane();

    // 将道路设置为可用状态
    void enable();

    // 将道路设置为不可用状态
    void disable();

    bool getLaneExistance() const;

    // 生成道路中线信息(包括坐标系、中线在world系、中线在frenet系)(每一点的坐标系以前向为正，后向为负，左向为正，右向为负)
    void generateLaneCenter(path_planning_msgs::BoundedCurve geometry);

    // 获取道路中线信息(包括坐标系、中线在world系、中线在frenet系)
    const std::vector<PathPlanningUtilities::CoordinationPoint> &getLaneCoordnation() const;

    const PathPlanningUtilities::Path &getLaneCenterPathInWorld() const;

    const PathPlanningUtilities::Path &getLaneCenterPathInFrenet() const;

    // 确定道路中每一个点的变换矩阵
    void generateLaneTransMatrix();

    const std::vector<TransMatrix> &getLaneTransMatrix() const;

    // 确定道路限速
    // void setLaneVelocityLimitaion(double lane_velocity_limitation) {
    //     this->lane_velocity_limitation_ = lane_velocity_limitation;
    // }

    // double getLaneVelocityLimitation() {
    //     return this->lane_velocity_limitation_;
    // }

    // 确定道路限速
    void setLaneVelocityLimitation(const std::vector<double> &lane_velocity_limitation);

    const std::vector<double> &getLaneVelocityLimitation() const;

    // 确定道路最低速度
    void setLowVelocity(const std::vector<double> &lane_lowest_velocity);

    const std::vector<double> &getLowVelocity() const;

    // 确定道路是否是被引导道路
    void setGuidedFlag(bool is_guided);

    bool getGuidedFlag() const;

    // 找出当前位置在道路中对应的下标
    // TODO: check this logic, maybe has problem
    // Note that although this method may has some problem, in the new tyrajectory planning, this calculation method is not used
    int findCurrenPositionIndexInLane(double position_x, double position_y) const;

    void setTurn(int turn);

    int getTurn() const;

    // 点是否在道路内
    bool judgePoint2fInLane(const PathPlanningUtilities::Point2f &position, size_t start_index, size_t end_index, double width) const;

    // 设置道路的优先级
    void setLanePriority(double priority);

    // 获取道路优先级
    double getLanePriority() const;

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
    double calculateDistanceFromPosition(const Eigen::Matrix<double, 2, 1>& cur_pos);

    // Calculate target lane point from a position and a specified distance
    Eigen::Matrix<double, 2, 1> calculateTargetLanePosition(const Eigen::Matrix<double, 2, 1>& position, double distance);

    // Find nearest lane point index from a position
    // Calculation of distances to find the local minimum
    int findCurrenPositionIndexInLane(const Eigen::Matrix<double, 2, 1>& position) const;

    // Judge position whether in lane
    bool isInLane(const PathPlanningUtilities::Point2f& position);

    // Pre-process, judge whether a lane is occupied by static obstacle and virtual traffic rule obstacle
    bool isLaneOccupiedByStaticObs(const Eigen::Matrix<double, 2, 1>& position, const std::vector<DecisionMaking::Obstacle>& all_obs, const std::vector<vec_map_cpp_msgs::VirtualObstacle> &traffic_virtual_obs);

    bool lane_existance_ = false;   // 道路是否可用
    std::vector<PathPlanningUtilities::CoordinationPoint> lane_coorination_;  // 道路完整信息
    std::vector<PathPlanningUtilities::CurvePoint> lane_curve_; // Lane center point information
    std::vector<TransMatrix> lane_transmatrix_;  // 道路上每一点形成的坐标系，用于world与frenet转化
    PathPlanningUtilities::Path lane_center_path_in_world_;  // 道路中线在world系下的坐标
    PathPlanningUtilities::Path lane_center_path_in_frenet_;  // 道路中线在frenet系下的坐标
    bool is_guided_;  // 道路是否是被引导道路
    std::vector<double> lane_velocity_limitation_;  // 道路上每一点的速度上限
    std::vector<double> lane_lowest_velocity_;  // 道路上每一点的速度下限
    int turn_;  // 转向情况
    double priority_;  // 道路的优先级
    // KDTree kb_tree_;
    // size_t lane_type_; // 道路的类型
    // bool stop_line_existance_ = false; // 道路中是否存在停止线
    // double lane_type_changing_distance_; // 道路类型变换位置，也是停止线位置
    // int traffic_light_id_; // 道路中是否存在交通灯
    // int turn_direction_; // 道路的转向
};


// Piecewise quintic spline based lane
class ParametricLane {
 public:
    // Constructor
    ParametricLane() = default;
    // Destructor
    ~ParametricLane() = default;

    /**
     * @description: generate the whole information oif the lane
     * @param {BoundedCurve} geometry: key points of the lane
     * @param {vector<double>} stations: distances in the continuous points, TODO: replace this with a message from map 
     * @param {vector<std::array<double, 12>>} coefficients: coefficients for each segment of the whole piecewise quintic spline, TODO: replace this with a message from map
     */
    void generateLaneCenter(path_planning_msgs::BoundedCurve geometry, std::vector<double> gaps, std::vector<std::array<double, 12>> coefficients) {
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
    Eigen::Vector2d calculateFrenetPoint(const Eigen::Vector2d& pos) {
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
     * @description: calculate the nearest in the lane given a position
     * @return {*}
     */    
    PathPlanningUtilities::CurvePoint findNearestPoint(const Eigen::Vector2d& pos) {
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
    int calculateNearestScatterPointIndex(const Eigen::Vector2d& pos) {
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


    int n_{0}; // Scatter points' number
    std::vector<double> stations_; // Specify the accumulated distance from the first point of lane
    std::vector<double> gaps_; // Specify the distance between each two continuous points
    std::vector<PathPlanningUtilities::CoordinationPoint> lane_coorination_;  // Complete information for each given points
    std::vector<Eigen::Vector2d> points_; // Scatter key points
    Eigen::Matrix<double, Eigen::Dynamic, 12> coefficients_; // Coefficients for piecewise quintic spline



    
};


// A segment of quintic spline
class QuinticSpline {
 public:
    /**
     * @brief speed up calculation
     * @param {double&} remain_station
     * @param {double*} pos_x
     * @param {double*} pos_y
     * @param {double*} theta
     * @param {double*} curvature
     * @return {*}
     */    
    static void calculatePointInfo(const PathPlanningUtilities::CurvePoint& begin_state, const PathPlanningUtilities::CurvePoint& end_state, const double& real_dis, const double& remain_station, double* pos_x, double* pos_y, double* theta, double* curvature);

    /**
     * @brief calculate the point information given a s
     * @return the corresponding s
     */
    static PathPlanningUtilities::CurvePoint calculateCurvePoint(const Eigen::Matrix<double, 1, 12>& current_coefficients, const double& l, const double& s) {
        
        // DEBUG
        if (s < 0.0 || s > l) {
            std::cout << "[StateTransformer] s is out of range." << std::endl;
        }
        // END DEBUG
        
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
     * @brief calculate the corresponding arc length given a position
     * @param {double&} l: whole length
     * @param {Vector2d&} pos: query position
     * @return projected arc length
     */    
    static double calculateArcLength(const Eigen::Matrix<double, 1, 12>& current_coefficients, const double& l, const Eigen::Vector2d& pos) {
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



};






#endif
