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
    ParametricLane();
    // Destructor
    ~ParametricLane();

    /**
     * @description: generate the whole information oif the lane
     * @param {BoundedCurve} geometry: key points of the lane
     * @param {vector<double>} stations: distances in the continuous points, TODO: replace this with a message from map 
     * @param {vector<std::array<double, 12>>} coefficients: coefficients for each segment of the whole piecewise quintic spline, TODO: replace this with a message from map
     */
    void generateLaneCenter(path_planning_msgs::BoundedCurve geometry, std::vector<double> gaps, std::vector<std::array<double, 12>> coefficients);

    /**
     * @description: transform a point from world to frenet (only containing abscissa and ordinate)
     * @return point position in frenet frame
     */    
    Eigen::Vector2d calculateFrenetPoint(const Eigen::Vector2d& pos);

    /**
     * @description: transform a point from frenet to world (only containing abscissa and ordinate)
     * @return point position in world frame
     */ 
    Eigen::Vector2d calculateWorldPoint(const Eigen::Vector2d& frenet_pos);

    /**
     * @description: calculate the nearest in the lane given a position
     * @return {*}
     */    
    PathPlanningUtilities::CurvePoint findNearestPoint(const Eigen::Vector2d& pos);

    /**
     * @description: calculate nearest key points index with binary search
     * @return the corresponding index
     */    
    int calculateNearestScatterPointIndex(const Eigen::Vector2d& pos);


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
    static PathPlanningUtilities::CurvePoint calculateCurvePoint(const Eigen::Matrix<double, 1, 12>& current_coefficients, const double& l, const double& s);

    /**
     * @brief calculate the corresponding arc length given a position
     * @param {double&} l: whole length
     * @param {Vector2d&} pos: query position
     * @return projected arc length
     */    
    static double calculateArcLength(const Eigen::Matrix<double, 1, 12>& current_coefficients, const double& l, const Eigen::Vector2d& pos);



};






#endif
