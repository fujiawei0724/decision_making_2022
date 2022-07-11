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

// class KDTree {
//  public:

//     // 构造函数
//     KDTree(const std::vector<PathPlanningUtilities::Point2f> &points) {
//         pcl::PointCloud<pcl::PointXY>::Ptr obstacles (new pcl::PointCloud<pcl::PointXY>());
//         for (int i = 0; i < static_cast<int>(points.size()); i++) {
//             pcl::PointXY obstacle = {static_cast<float>(points[i].x_), static_cast<float>(points[i].y_)};
//             obstacles->points.push_back(obstacle);
//             pointToIndex_[std::make_pair(static_cast<float>(points[i].x_), static_cast<float>(points[i].y_))] = i;
//         }
//         obstacles->width = obstacles->points.size();
//         obstacles->height = 1;
//         obstacles_ = obstacles;
//         // 构造kdtree
//         kdtree_.setInputCloud(this->obstacles_);
//     }

//     // 构造函数
//     KDTree() {}

//     // 析构函数
//     ~KDTree() {};

//     // 搜索k最近邻
//     int findKNeighbor(float x, float y, std::vector<std::pair<float, float>> *results, std::vector<float> *sq_distances, int k = 1) const {
//         std::vector<std::pair<float, float>> points;
//         // 构造搜索点
//         pcl::PointXY search_point = {x, y};
//         // 进行搜索
//         std::vector<int> pointIdxNKNSearch(k);
//         std::vector<float> pointNKNSquaredDistance(k);
//         if (this->kdtree_.nearestKSearch (search_point, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
//             for (size_t i = 0; i < pointIdxNKNSearch.size (); i++) {
//                 points.push_back(std::make_pair(this->obstacles_->points[pointIdxNKNSearch[i]].x, this->obstacles_->points[pointIdxNKNSearch[i]].y));
//             }
//             *results = points;
//             *sq_distances = pointNKNSquaredDistance;
//             return 0;
//         } else {
//             return -1;
//         }
//     }

//     bool findNearestIndex(const Eigen::Matrix<double, 2, 1>& position, int* index) const {
//         std::vector<std::pair<float, float>> nearest_points;
//         std::vector<float> nearest_dis;
//         int res = findKNeighbor(static_cast<float>(position(0, 0)), static_cast<float>(position(1, 0)), &nearest_points, &nearest_dis);
//         if (res == 0) {
//             *index = pointToIndex_.at(nearest_points[0]);
//             return true;
//         } else {
//             *index = 0;
//             return false;
//         }

//     }

//     // // 搜索范围近邻
//     // int findRangeNeighbor(float x, float y, std::vector<std::pair<float, float>> *results, std::vector<float> *sq_distances, double range) const {
//     //     std::vector<std::pair<float, float>> points;
//     //     // 构造搜索点
//     //     pcl::PointXY search_point = {x, y};
//     //     // 进行搜索
//     //     std::vector<int> pointIdxRadiusSearch;
//     //     std::vector<float> pointRadiusSquaredDistance;
//     //     if (this->kdtree_.radiusSearch (search_point, range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
//     //         for (size_t i = 0; i < pointIdxRadiusSearch.size (); i++) {
//     //             points.push_back(std::make_pair(this->obstacles_->points[pointIdxRadiusSearch[i]].x, this->obstacles_->points[pointIdxRadiusSearch[i]].y));
//     //         }
//     //         *results = points;
//     //         *sq_distances = pointRadiusSquaredDistance;
//     //         return 0;
//     //     } else {
//     //         return -1;
//     //     }
//     // }

 
//  private:
//     pcl::PointCloud<pcl::PointXY>::Ptr obstacles_;  // 障碍物信息
//     pcl::KdTreeFLANN<pcl::PointXY> kdtree_;  // kdtree
//     std::map<std::pair<float, float>, int> pointToIndex_;
// };

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

};






#endif
