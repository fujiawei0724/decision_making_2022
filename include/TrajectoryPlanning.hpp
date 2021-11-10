/*
 * @Author: fujiawei0724
 * @Date: 2021-11-04 15:05:54
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-11-10 15:09:30
 * @Descripttion: The components for trajectory planning. 
 */

#pragma once

#include "Const.hpp"
#include "VehicleState.hpp"

namespace TrajectoryPlanner {

using namespace Common;

class TrajPlanning3DMap {
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

    TrajPlanning3DMap() = default;
    TrajPlanning3DMap(const Config& config) {
        config_ = config;
        p_3d_grid_ = new GridMap3D(config_.map_size, config_.map_resolution, config_.axis_name);
    }
    ~TrajPlanning3DMap() = default;


    /**
     * @brief Generate semantic cubes using multi thread
     */
    void generateSemanticCubes() {
        // Initialize containers
        int seeds_num = static_cast<int>(seeds_.size());
        // Note that the last seed is valid to generate semantic cube
        semantic_coord_cubes_.resize(seeds_num - 1);
        semantic_coord_cubes_valid_.resize(seeds_num - 1);
        std::vector<std::thread> thread_set(seeds_num - 1);

        // Generate semantic cubes using multi thread
        for (int i = 0; i < static_cast<int>(semantic_coord_cubes_.size()); i++) {
            thread_set[i] = std::thread();
        }
        for (int i = 0; i < static_cast<int>(semantic_coord_cubes_.size()); i++) {
            thread_set[i].join();
        }
    }

    /**
     * @brief Generate semantic cube from a seed
     */    
    void generateSingleSemanticCube(int index) {
        // Generate initial semantic cube
        Point3i cur_seed = seeds_[index];
        Point3i next_seed = seeds_[index];
        SemanticCube<int> initial_coord_semantic_cube = ShapeUtils::generateInitialCoordSemanticCube(cur_seed, next_seed, index);

        // Judge whether the initial semantic cube is free
        if (!checkIfSemanticCubeIsFree(initial_coord_semantic_cube)) {
            semantic_coord_cubes_valid_[index] = true;
            return;
        }

        // Initial cube

        
        
    }

    /**
     * @brief Inflate the cube in 3D grid map until reached to the occupied space
     * @param init_seman_cube the initial scale of semantic cube
     */    
    void inflateCube(SemanticCube<int>* cube) {
        // Determine the gap of each step
        int x_p_step = config_.inflate_steps[0];
        int x_n_step = config_.inflate_steps[1];
        int y_p_step = config_.inflate_steps[2];
        int y_n_step = config_.inflate_steps[3];
        int z_p_step = config_.inflate_steps[4];

        // Calculate rough constraint condition
        // TODO: check these parameters
        int t_max_grids = cube->t_start_ + config_.MaxNumOfGridAlongTime;

        double t = t_max_grids * p_3d_grid_->dims_resolution(2);
        double a_max = config_.MaxLongitudinalAcc;
        double a_min = config_.MaxLongitudinalDecel;
        double d_comp = initial_fs_.vec_s_[1] * 1.0;

        double s_u = initial_fs_.vec_s_[0] + initial_fs_.vec_s_[1] * t + 0.5 * a_max * t * t + d_comp;
        double s_l = initial_fs_.vec_s_[0] + initial_fs_.vec_s_[1] * t + 0.5 * a_min * t * t - d_comp;

        int s_idx_u = p_3d_grid_->getCoordUsingGlobalMetricOnSingleDim(s_u, 0);
        int s_idx_l = p_3d_grid_->getCoordUsingGlobalMetricOnSingleDim(s_l, 0);
        s_idx_l = std::max(s_idx_l, static_cast<int>((config_.s_back_len / 2.0) / config_.map_resolution[0]));

        // Inflate in 5 directions
        // TODO: check this logic, whether inflate in 6 directions is available and better?
        bool x_p_finish = false;
        bool x_n_finish = false;
        bool y_p_finish = false;
        bool y_n_finish = false;
        bool z_p_finish = false;

        while (!(x_p_finish && x_n_finish && y_p_finish && y_n_finish)) {

        }

        while (!z_p_finish) {
            
        }
    }

    /**
     * @brief Judge whether the semantic cube is free 
     * @param semantic_coord_cube cube needs to judge
     * @return is free
     */    
    bool checkIfSemanticCubeIsFree(const SemanticCube<int>& semantic_coord_cube) {
        int coord_s_min = semantic_coord_cube.s_start_;
        int coord_s_max = semantic_coord_cube.s_end_;
        int coord_d_min = semantic_coord_cube.d_start_;
        int coord_d_max = semantic_coord_cube.d_end_;
        int coord_t_min = semantic_coord_cube.t_start_;
        int coord_t_max = semantic_coord_cube.t_end_;

        int i, j, k;
        std::array<int, 3> coord;
        bool is_free;
        for (i = coord_s_min; i <= coord_s_max; i++) {
            for (j = coord_d_min; j <= coord_d_max; j++) {
                for (k = coord_t_min; k <= coord_t_max; k++) {
                    coord = {i, j, k};
                    is_free = p_3d_grid_->checkIfEqualUsingCoordinate(coord, 0);
                    if (!is_free) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    /**
     * @brief Inflate cube in x positive direction
     * @param n_step max number of movement step
     * @param cube cube needs to be inflated
     * @return is finished
     */    
    bool inflateCubeOnXPosAxis(const int& n_step, SemanticCube<int>* cube) {
        for (int i = 0; i < n_step; i++) {
            int x = cube->s_end_ + 1;
            if (!p_3d_grid_->checkCoordInRangeOnSingleDim(x, 0)) {
                return true;
            } else {

            }
        }

    }

    /**
     * @brief Check if plane is free on X axis
     * @param x coord in x
     * @param cube cube needs to be judge
     * @return is free
     */    
    bool checkIfPlaneIsFreeOnXAxis(const SemanticCube<int>& cube, const int& x) {
        int f0_min = cube.d_start_;
        int f0_max = cube.d_end_;
        int f1_min = cube.t_start_;
        int f1_max = cube.t_end_;
        std::array<int, 3> coord;
        bool is_free;
        for (int i = f0_min; i <= f0_max; i++) {
            for (int j = f1_min; j <= f1_max; j++) {
                coord = {x, i, j};
                is_free = p_3d_grid_->checkIfEqualUsingCoordinate(coord, 0);
                if (!is_free) {
                    return false;
                }
            }
        }
        return true;
    }



    /**
     * @brief Construct seeds information using ego trajectory information
     * @param ego_traj ego vehicle's trajectory in frenet coordination
     * @return is success
     */
    bool constructSeeds(const std::vector<FsVehicle>& ego_traj) {
        int num_states = static_cast<int>(ego_traj.size());
        if (num_states < 2) {
            return false;
        }

        std::vector<Point3i> tmp_seeds_;
        bool first_seed_determined = false;

        // Note that the last trajectory is not responsible for generate semantic cube
        for (int k = 0; k < num_states; k++) {
            if (!first_seed_determined) {
                double s_0 = initial_fs_.vec_s_[0];
                double d_0 = initial_fs_.vec_dt_[0];
                double t_0 = initial_fs_.time_stamp_;
                std::array<double, 3> p_w_0 = {s_0, d_0, t_0};
                auto coord_0 = p_3d_grid_->getCoordUsingGlobalPosition(p_w_0);

                double s_1 = ego_traj[k].fs_.vec_s_[0];
                double d_1 = ego_traj[k].fs_.vec_dt_[0];
                double t_1 = ego_traj[k].fs_.time_stamp_;
                std::array<double, 3> p_w_1 = {s_1, d_1, t_1};
                auto coord_1 = p_3d_grid_->getCoordUsingGlobalPosition(p_w_1);

                // Delete the point out of range
                if (!p_3d_grid_->checkCoordInRange(coord_1)) {
                    continue;
                }
                if (coord_1[2] <= 0) {
                    continue;
                }

                first_seed_determined = true;
                tmp_seeds_.emplace_back(Point3i(coord_0[0], coord_0[1], coord_0[2]));
                tmp_seeds_.emplace_back(Point3i(coord_1[0], coord_1[1], coord_1[2]));
            } else {
                double s = ego_traj[k].fs_.vec_s_[0];
                double d = ego_traj[k].fs_.vec_dt_[0];
                double t = ego_traj[k].fs_.time_stamp_;
                std::array<double, 3> p_w = {s, d, t};
                auto coord = p_3d_grid_->getCoordUsingGlobalPosition(p_w);

                if (!p_3d_grid_->checkCoordInRange(coord)) {
                    continue;
                }
                tmp_seeds_.emplace_back(Point3i(coord[0], coord[1], coord[2]));
            }
        }

        // Cache 
        seeds_ = tmp_seeds_;

    }
    
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


    GridMap3D* p_3d_grid_;
    Config config_;
    std::unordered_map<int, std::array<bool, 6>> inters_for_cube_;
    double start_time_;
    FrenetState initial_fs_;

    std::vector<Point3i> seeds_; // Seeds information
    std::vector<bool> semantic_coord_cubes_valid_; // Store whether the semantic generated is valid
    std::vector<SemanticCube<int>> semantic_coord_cubes_; // Cubes information


};


// The interface between trajectory planner and behavior planner
class StateInterface {

};

// Semantic cubes generator
class SemanticCubeGenerator {
public:
    using FrenetStateSequence = std::vector<FrenetState>;
    // Constructor
    SemanticCubeGenerator(const FrenetStateSequence& cur_fr_states_seq) {

    }
    // Destructor
    ~SemanticCubeGenerator() = default;

    

};




} // End of namespace trajectory planner

