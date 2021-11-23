/*
 * @Author: fujiawei0724
 * @Date: 2021-11-22 16:30:19
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-11-23 21:16:20
 * @Descripttion: Ssc trajectory planning.
 */


#pragma once

#include "Const.hpp"
#include "VehicleState.hpp"

namespace SscPlanner {

// Ssc map
class SscPlanning3DMap {
 public:
    using GridMap3D = GridMapND<uint8_t, 3>;
    struct Config {
        std::array<int, 3> map_size = {{1000, 100, 81}};
        std::array<double, 3> map_resolution = {{0.25, 0.2, 0.1}};
        std::array<std::string, 3> axis_name = {{"s", "d", "t"}};

        double s_back_len = 0.0;
        double MaxLongitudinalVel = 50.0;
        double MinLongitudinalVel = 0.0;
        double MaxLongitudinalAcc = 3.0;
        double MaxLongitudinalDecel = -8.0;
        double MaxLateralVel = 3.0;
        double MaxLateralAcc = 2.5;

        int MaxNumOfGridAlongTime = 2;

        // TODO: check this logic, the time inflate steps number may be too small to construct continuous cubes with large overlap to ensure the feasibility of optimization space
        std::array<int, 6> inflate_steps = {{20, 5, 10, 10, 1, 1}};
    };

    SscPlanning3DMap() = default;
    SscPlanning3DMap(const Config& config) {
        config_ = config;
        p_3d_grid_ = new GridMap3D(config_.map_size, config_.map_resolution, config_.axis_name);
    }
    ~SscPlanning3DMap() = default;
    

    // Construct 3D map using dynamic obstacles and static obstacles information
    void contruct3DMap(const std::unordered_map<int, std::vector<FsVehicle>>& surround_trajs) {
        fillDynamicObstacles(surround_trajs);
        fillStaticObstacles();
    }

    // Fill the dynamic obstacles information
    void fillDynamicObstacles(const std::unordered_map<int, std::vector<FsVehicle>>& surround_trajs) {
        for (auto iter = surround_trajs.begin(); iter != surround_trajs.end(); iter++) {
            fillSingleDynamicObstacle(iter->second);
        }
    }

    void fillSingleDynamicObstacle(const std::vector<FsVehicle>& fs_traj) {
        if (fs_traj.size() == 0) {
            printf("[TrajPlanning3DMap] trajectory is empty.");
            return;
        }

        // TODO: check the density of the trajectory, i.e., if the trajectory is sparse in t dimension, th 3D grid map would not be occupied completely according to the gap of t
        for (int i = 0; i < static_cast<int>(fs_traj.size()); i++) {

            // Judge the effectness of frenet state vehicle's vertex
            bool is_valid = true;
            for (const auto v: fs_traj[i].vertex_) {
                if (v(0) <= 0) {
                    is_valid = false;
                    break;
                }
            }
            if (!is_valid) {
                continue;
            }

            double z = fs_traj[i].fs_.time_stamp_;
            int t_idx = 0;
            std::vector<Point2i> v_coord;
            std::array<double, 3> p_w;
            for (const auto v: fs_traj[i].vertex_) {
                p_w = {v(0), v(1), z};
                auto coord = p_3d_grid_->getCoordUsingGlobalPosition(p_w);
                t_idx = coord[2];
                if (!p_3d_grid_->checkCoordInRange(coord)) {
                    is_valid = false;
                    break;
                }
                v_coord.push_back(Point2i(coord[0], coord[1]));
            }
            if (!is_valid) {
                continue;
            }
            std::vector<std::vector<cv::Point2i>> vv_coord_cv;
            std::vector<cv::Point2i> v_coord_cv;
            ShapeUtils::getCvPoint2iVecUsingCommonPoint2iVec(v_coord, v_coord_cv);
            vv_coord_cv.emplace_back(v_coord_cv);
            int w = p_3d_grid_->dims_size()[0];
            int h = p_3d_grid_->dims_size()[1];
            int layer_offset = t_idx * w * h;
            cv::Mat layer_mat = cv::Mat(h, w, CV_MAKE_TYPE(cv::DataType<uint8_t>::type, 1), p_3d_grid_->get_data_ptr() + layer_offset);
            cv::fillPoly(layer_mat, vv_coord_cv, 100);
        }
    }

    // Fill the static obstacles information
    // Note that in the first version, the static obstacles are not considered here, this is just a empty interface
    // TODO: add the static obstacles' occupied grid here
    void fillStaticObstacles() {

    }


    GridMap3D* p_3d_grid_{nullptr};
    Config config_;

};

} // End of namespace ssc planner