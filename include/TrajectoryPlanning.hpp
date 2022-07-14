/*
 * @Author: fujiawei0724
 * @Date: 2021-11-04 15:05:54
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-12-20 15:04:52
 * @Descripttion: The components for trajectory planning. 
 */

#pragma once

#include "Const.hpp"
#include "Tools.hpp"
#include "VehicleState.hpp"

// Trajectory planning based on quintic B-spline
namespace TrajectoryPlanner {

using namespace Common;

// 3d grid map for semantic cudes generation
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
     * @brief Run once to generate the semantic cubes sequence
     * @return The success of the generation process
     */    
    bool runOnce(const std::vector<FsVehicle>& ego_traj, const std::unordered_map<int, std::vector<FsVehicle>>& surround_laned_trajs, const std::unordered_map<int, std::vector<FsVehicle>>& surround_unlaned_obs_trajs, std::vector<SemanticCube<double>>* semantic_cubes_sequence) {
        
        // ~Stage I: construct 3D grid map
        contruct3DMap(surround_laned_trajs);
        contruct3DMap(surround_unlaned_obs_trajs);

        // ~Stage II: generate seeds 
        bool is_success = true;
        is_success = constructSeeds(ego_traj);

        // ~Stage III: generate semantic cubes
        is_success = generateSemanticCubes();

        if (!is_success) {
            return false;
        }
        *semantic_cubes_sequence = semantic_cubes_;
        return true;
    }

    /**
     * @brief Generate semantic cubes using multi thread
     * @return The result of preliminary judgement of semantic cubes validity
     */
    bool generateSemanticCubes() {
        // Initialize containers
        int seeds_num = static_cast<int>(seeds_.size());
        // Note that the last seed is valid to generate semantic cube
        semantic_coord_cubes_.resize(seeds_num - 1);
        semantic_cubes_.resize(seeds_num - 1);
        semantic_coord_cubes_valid_.resize(seeds_num - 1);
        std::vector<std::thread> thread_set(seeds_num - 1);

        // // Generate semantic cubes using multi thread
        // for (int i = 0; i < static_cast<int>(semantic_coord_cubes_.size()); i++) {
        //     thread_set[i] = std::thread(&TrajPlanning3DMap::generateSingleSemanticCube, this, i);
        // }
        // for (int i = 0; i < static_cast<int>(semantic_coord_cubes_.size()); i++) {
        //     thread_set[i].join();
        // }

        // DEBUG
        for (int i = 0; i < static_cast<int>(semantic_coord_cubes_.size()); i++) {
            generateSingleSemanticCube(i);
        }
        // END DEBUG

        // Preliminary judgement of validity
        bool is_valid = true;
        for (const bool& cur_valid: semantic_coord_cubes_valid_) {
            if (!cur_valid) {
                is_valid = false;
            }
        }
        if (!is_valid) {
            return false;
        }
        return true;
    }

    /**
     * @brief Generate semantic cube from a seed
     */    
    void generateSingleSemanticCube(int index) {
        // Generate initial semantic cube
        Point3i cur_seed = seeds_[index];
        Point3i next_seed = seeds_[index];
        SemanticCube<int> coord_semantic_cube = ShapeUtils::generateInitialCoordSemanticCube(cur_seed, next_seed, index);

        // Judge whether the initial semantic cube is free
        if (!checkIfSemanticCubeIsFree(coord_semantic_cube)) {
            semantic_coord_cubes_valid_[index] = false;
            return;
        }

        // Initial cube
        inflateCube(&coord_semantic_cube);

        // Transform coord information to global metric inforation 
        SemanticCube<double> semantic_cube;
        transformCoordSemCubeToSemCube(coord_semantic_cube, &semantic_cube);

        // Cache
        semantic_coord_cubes_[index] = coord_semantic_cube;
        semantic_cubes_[index] = semantic_cube;
        semantic_coord_cubes_valid_[index] = true;
    }

    /**
     * @brief Transform coord semantic cube to semantic cube
     * @param coord_semantic_cube semantic cube in the formation of coordinate
     * @param semantic_cube semantic cube in the formation of value
     */    
    void transformCoordSemCubeToSemCube(const SemanticCube<int>& coord_semantic_cube, SemanticCube<double>* semantic_cube) {
        semantic_cube->s_start_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_semantic_cube.s_start_, 0);
        semantic_cube->s_end_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_semantic_cube.s_end_, 0);
        semantic_cube->d_start_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_semantic_cube.d_start_, 1);
        semantic_cube->d_end_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_semantic_cube.d_end_, 1);
        semantic_cube->t_start_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_semantic_cube.t_start_, 2);
        semantic_cube->t_end_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_semantic_cube.t_end_, 2);
        semantic_cube->id_ = coord_semantic_cube.id_;
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

        // Inflate on s and d dimension
        while (!(x_p_finish && x_n_finish && y_p_finish && y_n_finish)) {
            if (!x_p_finish) {
                x_p_finish = inflateCubeOnXPosAxis(x_p_step, cube);
            }
            if (!x_n_finish) {
                x_n_finish = inflateCubeOnXNegAxis(x_n_step, cube);
            }
            if (!y_p_finish) {
                y_p_finish = inflateCubeOnYPosAxis(y_p_step,cube);
            }
            if (!y_n_finish) {
                y_n_finish = inflateCubeOnYNegAxis(y_n_step, cube);
            }

            // Rough constraints
            if (cube->s_end_ >= s_idx_u) {
                x_p_finish = true;
            }
            if (cube->s_start_ <= s_idx_l) {
                x_n_finish = true;
            }
        }

        // Inflate on t dimension
        while (!z_p_finish) {
            if (!z_p_finish) {
                z_p_finish = inflateCubeOnZPosAxis(z_p_step, cube);
            }

            // // TODO: check this t dimension inflate constraint condition, if it is too harsh
            // if (cube->t_end_ - cube->t_start_ >= config_.MaxNumOfGridAlongTime) {
            //     z_p_finish = true;
            // }
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
                if (checkIfPlaneIsFreeOnXAxis(*cube, x)) {
                    cube->s_end_ = x;
                } else {
                    return true;
                }
            }
        }
        return false;
    }



    /**
     * @brief Inflate cube in x negative direction
     * @param n_step max number of movement step
     * @param cube cube needs to be inflated
     * @return is finished
     */    
    bool inflateCubeOnXNegAxis(const int& n_step, SemanticCube<int>* cube) {
        for (int i = 0; i < n_step; i++) {
            int x = cube->s_start_ - 1;
            if (!p_3d_grid_->checkCoordInRangeOnSingleDim(x, 0)) {
                return true;
            } else {
                if (checkIfPlaneIsFreeOnXAxis(*cube, x)) {
                    cube->s_start_ = x;
                } else {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief Inflate cube in Y positive direction
     * @param n_step max number of movement step
     * @param cube cube needs to be inflated
     * @return is finished
     */    
    bool inflateCubeOnYPosAxis(const int& n_step, SemanticCube<int>* cube) {
        for (int i = 0; i < n_step; i++) {
            int y = cube->d_end_ + 1;
            if (!p_3d_grid_->checkCoordInRangeOnSingleDim(y, 1)) {
                return true;
            } else {
                if (checkIfPlaneIsFreeOnYAxis(*cube, y)) {
                    cube->d_end_ = y;
                } else {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief Inflate cube in Y negative direction
     * @param n_step max number of movement step
     * @param cube cube needs to be inflated
     * @return is finished
     */    
    bool inflateCubeOnYNegAxis(const int& n_step, SemanticCube<int>* cube) {
        for (int i = 0; i < n_step; i++) {
            int y = cube->d_start_ - 1;
            if (!p_3d_grid_->checkCoordInRangeOnSingleDim(y, 1)) {
                return true;
            } else {
                if (checkIfPlaneIsFreeOnYAxis(*cube, y)) {
                    cube->d_start_ = y;
                } else {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief Inflate cube in Z positive direction
     * @param n_step max number of movement step
     * @param cube cube needs to be inflated
     * @return is finished
     */    
    bool inflateCubeOnZPosAxis(const int& n_step, SemanticCube<int>* cube) {
        for (int i = 0; i < n_step; i++) {
            int z = cube->t_end_ + 1;
            if (!p_3d_grid_->checkCoordInRangeOnSingleDim(z, 2)) {
                return true;
            } else {
                if (checkIfPlaneIsFreeOnZAxis(*cube, z)) {
                    cube->t_end_ = z;
                } else {
                    return true;
                }
            }
        }
        return false;
    }


    /**
     * @brief Check if plane is free on X axis
     * @param x coord in x
     * @param cube cube needs to be judge
     * @return is free
     */    
    bool checkIfPlaneIsFreeOnXAxis(const SemanticCube<int>& cube, const int& x) const {
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
     * @brief Check if plane is free on Y axis
     * @param y coord in y
     * @param cube cube needs to be judge
     * @return is free
     */     
    bool checkIfPlaneIsFreeOnYAxis(const SemanticCube<int>& cube, const int& y) const {
        int f0_min = cube.s_start_;
        int f0_max = cube.s_end_;
        int f1_min = cube.t_start_;
        int f1_max = cube.t_end_;
        std::array<int, 3> coord;
        bool is_free;
        for (int i = f0_min; i <= f0_max; i++) {
            for (int j = f1_min; j <= f1_max; j++) {
                coord = {i, y, j};
                is_free = p_3d_grid_->checkIfEqualUsingCoordinate(coord, 0);
                if (!is_free) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * @brief Check if plane is free on Z axis
     * @param z coord in z
     * @param cube cube needs to be judge
     * @return is free
     */     
    bool checkIfPlaneIsFreeOnZAxis(const SemanticCube<int>& cube, const int& z) const {
        int f0_min = cube.s_start_;
        int f0_max = cube.s_end_;
        int f1_min = cube.d_start_;
        int f1_max = cube.d_end_;
        std::array<int, 3> coord;
        bool is_free;
        for (int i = f0_min; i <= f0_max; i++) {
            for (int j = f1_min; j <= f1_max; j++) {
                coord = {i, j, z};
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

        std::vector<Point3i> tmp_seeds;
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
                tmp_seeds.emplace_back(Point3i(coord_0[0], coord_0[1], coord_0[2]));
                tmp_seeds.emplace_back(Point3i(coord_1[0], coord_1[1], coord_1[2]));
            } else {
                double s = ego_traj[k].fs_.vec_s_[0];
                double d = ego_traj[k].fs_.vec_dt_[0];
                double t = ego_traj[k].fs_.time_stamp_;
                std::array<double, 3> p_w = {s, d, t};
                auto coord = p_3d_grid_->getCoordUsingGlobalPosition(p_w);

                if (!p_3d_grid_->checkCoordInRange(coord)) {
                    continue;
                }
                tmp_seeds.emplace_back(Point3i(coord[0], coord[1], coord[2]));
            }
        }

        // Cache 
        seeds_ = tmp_seeds;
        return true;

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
            printf("[TrajPlanning3DMap] trajectory is empty.\n");
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
    double start_time_;
    FrenetState initial_fs_;

    std::vector<Point3i> seeds_; // Seeds information
    std::vector<bool> semantic_coord_cubes_valid_; // Store whether the semantic generated is valid
    std::vector<SemanticCube<int>> semantic_coord_cubes_; // Cubes coord information
    std::vector<SemanticCube<double>> semantic_cubes_; // Cubes information


};

// Optimization tools 
class OptimizationUtils {
 public:

    static double calculateStartTime(const std::vector<double>& segment_ref_stamps) {
        return (1.0 / 120.0) * segment_ref_stamps[0] + (26.0 / 120.0) * segment_ref_stamps[1] + (33.0 / 60.0) * segment_ref_stamps[2] + (13.0 / 60.0) * segment_ref_stamps[3] + (1.0 / 120.0) * segment_ref_stamps[4];
    }

    static double calculateEndTime(const std::vector<double>& segment_ref_stamps) {
        return (1.0 / 120.0) * segment_ref_stamps[1] + (13.0 / 60.0) * segment_ref_stamps[2] + (33.0 / 60.0) * segment_ref_stamps[3] + (26.0 / 120.0) * segment_ref_stamps[4] + (1.0 / 120.0) * segment_ref_stamps[5];
    }

    static double calculateTimeSpan(const std::vector<double>& segment_ref_stamps) {
        return calculateEndTime(segment_ref_stamps) - calculateStartTime(segment_ref_stamps);
    }
};


// Interface with optimization library
class OptimizerInterface {
 public:
    using ET = CGAL::Gmpz;
    using Program = CGAL::Quadratic_program_from_iterators<
                        std::vector<double*>::iterator,                            // For A
                        double*,                                                   // For b
                        CGAL::Const_oneset_iterator<CGAL::Comparison_result>,      // For r
                        bool*,                                                     // For fl
                        double*,                                                   // For l
                        bool*,                                                     // For fu
                        double*,                                                   // For u
                        std::vector<double*>::iterator,                            // For D
                        double*>;                                                  // For c
    using Solution = CGAL::Quadratic_program_solution<ET>;

    OptimizerInterface() = default;
    ~OptimizerInterface() = default;

    // Load data 
    void load(const std::vector<double>& all_ref_stamps, const EqualConstraint& start_constraint, const EqualConstraint& end_constraint, const std::array<std::vector<double>, 4>& unequal_constraints) {
        all_ref_stamps_ = all_ref_stamps;
        start_constraint_ = start_constraint;
        end_constraint_ = end_constraint;
        unequal_constraints_ = unequal_constraints;
    }

    /**
     * @brief Multi thread optimize trajectory scatter points in s and d dimensions synchronous
     */    
    void runOnce(std::vector<double>* optimized_s, std::vector<double>* optimized_d) {
        // Prepare data for s and d dimensions
        std::array<double, 3> s_start_constraints = start_constraint_.toDimensionS();
        std::array<double, 3> s_end_constraints = end_constraint_.toDimensionS();
        std::array<std::vector<double>, 2> s_unequal_constraints = {unequal_constraints_[0], unequal_constraints_[1]};
        std::array<double, 3> d_start_constraints = start_constraint_.toDimensionD();
        std::array<double, 3> d_end_constraints = end_constraint_.toDimensionD();
        std::array<std::vector<double>, 2> d_unequal_constraints = {unequal_constraints_[2], unequal_constraints_[3]};

        // Multi thread calculation
        // TODO: add logic to handle the situation where the optimization process is failed
        std::thread s_thread(&OptimizerInterface::optimizeSingleDim, this, s_start_constraints, s_end_constraints, s_unequal_constraints[0], s_unequal_constraints[1], "s");
        std::thread d_thread(&OptimizerInterface::optimizeSingleDim, this, d_start_constraints, d_end_constraints, d_unequal_constraints[0], d_unequal_constraints[1], "d");
        s_thread.join();
        d_thread.join();

        // Cache information
        *optimized_s = optimized_data_["s"];
        *optimized_d = optimized_data_["d"];
    }

    /**
     * @brief Calculate the matrices related to objective function, for both s and d dimensions, D and c has the same value
     * @param D 
     * @param c
     */
    void calculateDcMatrix(std::vector<double*>* D, double** c) {
        
        // Initialize D matrix
        int points_num = static_cast<int>(all_ref_stamps_.size());
        Eigen::MatrixXd D_matrix = Eigen::MatrixXd::Zero(points_num, points_num);
        int segment_number = points_num - 5;

        // Calculate D matrix
        for (int i = 0; i < segment_number; i++) {
            // Calculate time span
            std::vector<double> cur_segment_ref_stamps{all_ref_stamps_.begin() + i, all_ref_stamps_.begin() + i + 6};
            double time_span = OptimizationUtils::calculateTimeSpan(cur_segment_ref_stamps);
            // TODO: check this logic, the exponent is "-3" or "-5"?
            double time_coefficient = pow(time_span, -3);

            // Intergrate to objective function
            D_matrix.block(i, i, 6, 6) += BSplineHessianMatrix * time_coefficient;
        }

        // Convert the eigen data to double**
        std::vector<double*> tmp_D(points_num);
        for (int i = 0; i < points_num; i++) {
            double* d_col = new double[points_num];
            for (int j = 0; j <= i; j++) {
                *(d_col + j) = D_matrix(i, j);
            }
            tmp_D[i] = d_col;
        }

        // Generate b information, all zeros
        double* tmp_c = new double[points_num];

        // TODO: check this parameters transformation process
        *D = tmp_D;
        *c = tmp_c;
    }

    /**
     * @brief Caclculate equal constraints in the start point and end point 
     * @param A
     * @param b
     * @param single_start_constraints start constraints in one dimension (s/d)
     * @param single_end_constraints end constraints in one dimension (s/d) 
     */  
    void calculateAbMatrix(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, std::vector<double*>* A, double** b) {
        
        // Construct matrix A and b to load information
        int points_num = static_cast<int>(all_ref_stamps_.size());
        Eigen::MatrixXd A_matrix = Eigen::MatrixXd::Zero(8, points_num);
        Eigen::MatrixXd b_matrix = Eigen::MatrixXd::Zero(8, 1);

        // Added points constraint conditions
        A_matrix(0, 0) = 1.0, A_matrix(0, 2) = -2.0, A_matrix(0, 4) = 1.0;
        A_matrix(1, 1) = 1.0, A_matrix(1, 2) = -2.0, A_matrix(1, 3) = 1.0;
        A_matrix(2, points_num - 1) = 1.0, A_matrix(2, points_num - 3) = -2.0, A_matrix(2, points_num - 5) = 1.0;
        A_matrix(3, points_num - 2) = 1.0, A_matrix(3, points_num - 3) = -2.0, A_matrix(3, points_num - 4) = 1.0;

        // Start point and end point position constraint conditions
        A_matrix(4, 2) = 1.0, A_matrix(5, points_num - 3) = 1.0;
        b_matrix(4, 0) = single_start_constraints[0], b_matrix(5, 0) = single_end_constraints[0];

        // Start point and end point velocity constraint conditions
        std::vector<double> start_segment_ref_stamps{all_ref_stamps_.begin(), all_ref_stamps_.begin() + 6};
        double start_segment_time_span = OptimizationUtils::calculateTimeSpan(start_segment_ref_stamps);
        A_matrix(6, 0) = -1.0 / 24.0, A_matrix(6, 1) = -5.0 / 12.0, A_matrix(6, 3) = 5.0 / 12.0, A_matrix(6, 4) = 1.0 / 24.0;
        b_matrix(6, 0) = single_start_constraints[1] * start_segment_time_span;
        std::vector<double> end_segment_ref_stamps{all_ref_stamps_.begin() + points_num - 6, all_ref_stamps_.begin() + points_num};
        double end_segment_time_span = OptimizationUtils::calculateTimeSpan(end_segment_ref_stamps);
        A_matrix(7, points_num - 5) = -1.0 / 24.0, A_matrix(7, points_num - 4) = -5.0 / 12.0, A_matrix(7, points_num - 2) = 5.0 / 12.0, A_matrix(7, points_num - 1) = -1.0 / 24.0;
        b_matrix(7, 0) = single_end_constraints[1] * end_segment_time_span;

        // TODO: for quintic B-spline, the acceleration of the start point and point must be set to zero, add an algorithm to handle this problem

        // Transform the structure of data
        std::vector<double*> tmp_A(points_num);
        for (int i = 0; i < points_num; i++) {
            double* a_col = new double[8];
            for (int j = 0; j < 8; j++) {
                *(a_col + j) = A_matrix(j, i);
            } 
            tmp_A[i] = a_col;
        }
        double* tmp_b = new double[8];
        for (int i = 0; i < 8; i++) {
            *(tmp_b + i) = b_matrix(i, 0);
        }


        // TODO: check this parameters transformation process
        *A = tmp_A;
        *b = tmp_b;
    }

    /**
     * @brief Calculate lower and upper boundaries (provided by semantic cubes) for intermedite points 
     * @param single_lower_boundaries lower boundaries for intermediate points in single dimension
     * @param single_upper_boundaries upper boundaries for intermediate points in single dimension
     * @param fl for each variable, is lower boundary valid
     * @param l lower bounds for each variable 
     * @param fu for each variable, is upper boundary valid
     * @param u upper bounds for each variable
     */    
    void calculateBoundariesForIntermediatePoints(const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries, bool** fl, double** l, bool** fu, double** u) {

        
        int points_num = static_cast<int>(all_ref_stamps_.size());
        bool* tmp_fl = new bool[points_num];
        double* tmp_l = new double[points_num];
        bool* tmp_fu = new bool[points_num];
        double* tmp_u = new double[points_num];

        for (int i = 0; i < points_num; i++) {
            if (i <= 2 || i >= points_num - 3) {
                // For the first three points and last three points, the unequal constraints are invalid
                *(tmp_fl + i) = false;
                *(tmp_fu + i) = false;
            } else {
                *(tmp_fl + i) = true;
                *(tmp_fu + i) = true;
                *(tmp_l + i) = single_lower_boundaries[i - 2];
                *(tmp_u + i) = single_upper_boundaries[i - 2];
            }
        }

        // TODO: check this parameters transformation process
        *fl = tmp_fl;
        *l = tmp_l;
        *fu = tmp_fu;
        *u = tmp_u;
    }

    /**
     * @brief Optimize trajectory scatter points in 2d 
     * @param single_start_constraints start constraints' disintegration in single dimension (s/d)
     * @param single_end_constraints end constraints' disintegration in single dimension (s/d)
     * @param single_lower_boundaries lower boundaries in single dimension (s/d)
     * @param single_upper_bounaries upper boundaries in single dimension (s/d)
     * @param dimension_name "s" or "d"
     */
    void optimizeSingleDim(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries, std::string dimension_name) {
        // ~Stage I: calculate D and c matrices (objective function)
        std::vector<double*> D;
        double* c = nullptr;
        calculateDcMatrix(&D, &c);
        double c0 = 0.0;

        // ~Stage II: calculate start and end points equal constraints
        CGAL::Const_oneset_iterator<CGAL::Comparison_result> r(CGAL::EQUAL);
        std::vector<double*> A;
        double* b = nullptr;
        calculateAbMatrix(single_start_constraints, single_end_constraints, &A, &b);

        // ~Stage III: calculate low and up boundaries for intermediate points
        bool* fl = nullptr;
        double* l = nullptr;
        bool* fu = nullptr;
        double* u = nullptr;
        calculateBoundariesForIntermediatePoints(single_lower_boundaries, single_upper_boundaries, &fl, &l, &fu, &u);

        // ~Stage IV: optimization and transform the formation of optimization result
        int variables_num = static_cast<int>(all_ref_stamps_.size());
        int constraints_num = 8;
        Program qp(variables_num, constraints_num, A.begin(), b, r, fl, l, fu, u, D.begin(), c, c0);
        Solution s = CGAL::solve_quadratic_program(qp, ET());
        // Convert data
        std::vector<double> optimized_values;
        for (auto iter = s.variable_values_begin(); iter != s.variable_values_end(); iter++) {
            double value = CGAL::to_double(*iter);
            optimized_values.emplace_back(value);
        }
        // Truncate data to delete the addtional points
        std::vector<double> truncated_optimized_data{optimized_values.begin() + 2, optimized_values.end() - 2};
        
        // ~Stage V: store information
        optimized_data_[dimension_name] = truncated_optimized_data;
    }


    
    std::vector<double> all_ref_stamps_;
    EqualConstraint start_constraint_;
    EqualConstraint end_constraint_;
    std::array<std::vector<double>, 4> unequal_constraints_;
    std::unordered_map<std::string, std::vector<double>> optimized_data_;
};


// Trajectory optimizer
class TrajectoryOptimizer {
 public:
    using DrivingCorridor = std::vector<SemanticCube<double>>;
    TrajectoryOptimizer() {
        opt_itf_ = new OptimizerInterface();
    }
    ~TrajectoryOptimizer() = default;

    // Load data to optimizer
    void load(const EqualConstraint& start_constraint, const EqualConstraint& end_constraint, const DrivingCorridor& driving_corridor, const std::vector<double>& ref_stamps) {
        start_constraint_ = start_constraint;
        end_constraint_ = end_constraint;
        driving_corridor_ = driving_corridor;
        ref_stamps_ = ref_stamps;
    }

    // Run optimizer
    void runOnce(std::vector<double>* s, std::vector<double>* d, std::vector<double>* t, bool* res) {
        
        // ~Stage I: check, prepare, and supply data 
        assert(ref_stamps_.size() - 1 == driving_corridor_.size() && static_cast<int>(ref_stamps_.size()) >= 3);
        // Add addtional time stamps to approximate start point and end point 
        std::vector<double> all_ref_stamps = calculateAllRefStamps();
        // Generate unequal constraints for the intermediate points
        std::array<std::vector<double>, 4> unequal_constraints;
        bool is_unequal_constraints_generation_success = generateUnequalConstraints(&unequal_constraints);
        if (!is_unequal_constraints_generation_success) {
            *res = false;
            return;
        }

        // ~Stage II: load data and optimization
        // TODO: handle the optimization failed situation
        std::vector<double> optimized_s;
        std::vector<double> optimized_d;
        opt_itf_->load(all_ref_stamps, start_constraint_, end_constraint_, unequal_constraints);
        opt_itf_->runOnce(&optimized_s, &optimized_d);

        // ~Stage III: data return
        *s = optimized_s;
        *d = optimized_d;
        *t = ref_stamps_;
        *res = true;
        
    }

    /**
     * @brief Add additional points' reference time stamps to construct the whole reference time stamps
     * @return all referencetime stamps
     */    
    std::vector<double> calculateAllRefStamps() {
        int init_stamps_num = static_cast<int>(ref_stamps_.size());
        std::vector<double> all_ref_stamps = std::vector<double>(init_stamps_num + 4, 0.0);
        double add_stamp_1 = 2.0 * ref_stamps_[0] - ref_stamps_[2];
        double add_stamp_2 = 2.0 * ref_stamps_[0] - ref_stamps_[1];
        double add_stamp_3 = 2.0 * ref_stamps_[init_stamps_num - 1] - ref_stamps_[init_stamps_num - 2];
        double add_stamp_4 = 2.0 * ref_stamps_[init_stamps_num - 1] - ref_stamps_[init_stamps_num - 3];
        for (int i = 0; i < static_cast<int>(all_ref_stamps.size()); i++) {
            if (i == 0) {
                all_ref_stamps[i] = add_stamp_1;
            } else if (i == 1) {
                all_ref_stamps[i] = add_stamp_2;
            } else if (i == static_cast<int>(all_ref_stamps.size()) - 2) {
                all_ref_stamps[i] = add_stamp_3;
            } else if (i == static_cast<int>(all_ref_stamps.size()) - 1) {
                all_ref_stamps[i] = add_stamp_4;
            } else {
                all_ref_stamps[i] = ref_stamps_[i - 2];
            }
        }
        return all_ref_stamps;
    }

    /**
     * @brief Generate unequal constraints for intermediate points
     * @param unequal_constraints four vectors in the array represent the lower bounds of s, upper bounds of s, lower bounds of d, and upper bounds of d in order
     * @return is success
     */    
    bool generateUnequalConstraints(std::array<std::vector<double>, 4>* unequal_constraints) {
        // Initialize
        std::array<std::vector<double>, 4> tmp_unequal_constraints = {};
        for (int i = 0; i < 4; i++) {
            tmp_unequal_constraints[i].resize(static_cast<int>(ref_stamps_.size()) - 2);
        }

        for (int i = 0; i < static_cast<int>(ref_stamps_.size()); i++) {
            if (i == 0 || i == static_cast<int>(ref_stamps_.size()) - 1) {
                // In the no added status, start point and end point only have equal constraints 
                continue;
            }
            double ref_stamp = ref_stamps_[i];
            
            // Calculate the first semantic cube affects the point in reference stamp
            int first_index = std::max(i - 5, 0);
            // Traverse the affected semantic cubes to generate unequal constraint
            double s_low = MIN_VALUE;
            double s_up = MAX_VALUE;
            double d_low = MIN_VALUE;
            double d_up = MAX_VALUE;
            for (int j = first_index; j < i + 1; j++) {
                if (ref_stamp < driving_corridor_[j].t_start_ || ref_stamp > driving_corridor_[j].t_end_) {
                    printf("[TrajectoryOptimizer] unequal constraint generation failed.");
                    return false;
                }
                s_low = std::max(s_low, driving_corridor_[j].s_start_);
                s_up = std::min(s_up, driving_corridor_[j].s_end_);
                d_low = std::max(d_low, driving_corridor_[j].d_start_);
                d_up = std::min(d_up, driving_corridor_[j].d_end_);
            }

            tmp_unequal_constraints[0][i - 1] = s_low;
            tmp_unequal_constraints[1][i - 1] = s_up;
            tmp_unequal_constraints[2][i - 1] = d_low;
            tmp_unequal_constraints[3][i - 1] = d_up; 
            
        }

        *unequal_constraints = tmp_unequal_constraints;
        return true;
    }

    OptimizerInterface* opt_itf_{nullptr};
    EqualConstraint start_constraint_;
    EqualConstraint end_constraint_;
    DrivingCorridor driving_corridor_;
    std::vector<double> ref_stamps_;
};

class QuinticBSplineTrajectory {
 public:
    QuinticBSplineTrajectory(const std::vector<double>& s, const std::vector<double>& d, const std::vector<double>& t) {
        assert(s.size() == t.size() && d.size() == t.size());
        
        // Add additional points information
        int all_points_num = static_cast<int>(t.size()) + 4;
        assert(all_points_num > 5);
        std::vector<double> s_all(all_points_num, 0.0);
        std::vector<double> d_all(all_points_num, 0.0);
        std::vector<double> t_all(all_points_num, 0.0);
        for (int i = 0; i < static_cast<int>(t.size()); i++) {
            s_all[i + 2] = s[i];
            d_all[i + 2] = d[i];
            t_all[i + 2] = t[i];
        }
        s_all[0] = 2.0 * s_all[2] - s_all[4];
        d_all[0] = 2.0 * d_all[2] - d_all[4];
        t_all[0] = 2.0 * t_all[2] - t_all[4];
        s_all[1] = 2.0 * s_all[2] - s_all[3];
        d_all[1] = 2.0 * d_all[2] - d_all[3];
        t_all[1] = 2.0 * t_all[2] - t_all[3];
        s_all[all_points_num - 1] = 2.0 * s_all[all_points_num - 3] - s_all[all_points_num - 5];
        d_all[all_points_num - 1] = 2.0 * d_all[all_points_num - 3] - d_all[all_points_num - 5];
        t_all[all_points_num - 1] = 2.0 * t_all[all_points_num - 3] - t_all[all_points_num - 5];
        s_all[all_points_num - 2] = 2.0 * s_all[all_points_num - 3] - s_all[all_points_num - 4];
        d_all[all_points_num - 2] = 2.0 * d_all[all_points_num - 3] - d_all[all_points_num - 4];
        t_all[all_points_num - 2] = 2.0 * t_all[all_points_num - 3] - t_all[all_points_num - 4];

        // Determine segment number
        segment_num_ = all_points_num - 5;
        s_coefficients_.resize(segment_num_);
        d_coefficients_.resize(segment_num_);
        t_coefficients_.resize(segment_num_);

        // Calculate coefficients of segment 
        for (int i = 0; i < all_points_num - 5; i++) {
            // Calculate s coefficients
            s_coefficients_[i].resize(6);
            s_coefficients_[i][0] = (1.0 / 120.0) * s_all[i] + (26.0 / 120.0) * s_all[i + 1] + (33.0 / 60.0) * s_all[i + 2] + (13.0 / 60.0) * s_all[i + 3] + (1.0 / 120.0) * s_all[i + 4];
            s_coefficients_[i][1] = (-5.0 / 120.0) * s_all[i] + (-50.0 / 120.0) * s_all[i + 1] + (25.0 / 60.0) * s_all[i + 3] + (5.0 / 120.0) * s_all[i + 4];
            s_coefficients_[i][2] = (10.0 / 120.0) * s_all[i] + (20.0 / 120.0) * s_all[i + 1] + (-30.0 / 60.0) * s_all[i + 2] + (10.0 / 60.0) * s_all[i + 3] + (10.0 / 120.0) * s_all[i + 4];
            s_coefficients_[i][3] = (-10.0 / 120.0) * s_all[i] + (20.0 / 120.0) * s_all[i + 1] + (-10.0 / 60.0) * s_all[i + 3] + (10.0 / 120.0) * s_all[i + 4];
            s_coefficients_[i][4] = (5.0 / 120.0) * s_all[i] + (-20.0 / 120.0) * s_all[i + 1] + (15.0 / 60.0) * s_all[i + 2] + (-10.0 / 60.0) * s_all[i + 3] + (5.0 / 120.0) * s_all[i + 4];
            s_coefficients_[i][5] = (-1.0 / 120.0) * s_all[i] + (5.0 / 120.0) * s_all[i + 1] + (-5.0 / 60.0) * s_all[i + 2] + (5.0 / 60.0) * s_all[i + 3] + (-5.0 / 120.0) * s_all[i + 4] + (1.0 / 120.0) * s_all[i + 5];

            // Calculate d coefficients
            d_coefficients_[i].resize(6);
            d_coefficients_[i][0] = (1.0 / 120.0) * d_all[i] + (26.0 / 120.0) * d_all[i + 1] + (33.0 / 60.0) * d_all[i + 2] + (13.0 / 60.0) * d_all[i + 3] + (1.0 / 120.0) * d_all[i + 4];
            d_coefficients_[i][1] = (-5.0 / 120.0) * d_all[i] + (-50.0 / 120.0) * d_all[i + 1] + (25.0 / 60.0) * d_all[i + 3] + (5.0 / 120.0) * d_all[i + 4];
            d_coefficients_[i][2] = (10.0 / 120.0) * d_all[i] + (20.0 / 120.0) * d_all[i + 1] + (-30.0 / 60.0) * d_all[i + 2] + (10.0 / 60.0) * d_all[i + 3] + (10.0 / 120.0) * d_all[i + 4];
            d_coefficients_[i][3] = (-10.0 / 120.0) * d_all[i] + (20.0 / 120.0) * d_all[i + 1] + (-10.0 / 60.0) * d_all[i + 3] + (10.0 / 120.0) * d_all[i + 4];
            d_coefficients_[i][4] = (5.0 / 120.0) * d_all[i] + (-20.0 / 120.0) * d_all[i + 1] + (15.0 / 60.0) * d_all[i + 2] + (-10.0 / 60.0) * d_all[i + 3] + (5.0 / 120.0) * d_all[i + 4];
            d_coefficients_[i][5] = (-1.0 / 120.0) * d_all[i] + (5.0 / 120.0) * d_all[i + 1] + (-5.0 / 60.0) * d_all[i + 2] + (5.0 / 60.0) * d_all[i + 3] + (-5.0 / 120.0) * d_all[i + 4] + (1.0 / 120.0) * d_all[i + 5];

            // Calculte t coefficients
            t_coefficients_[i].resize(6);
            t_coefficients_[i][0] = (1.0 / 120.0) * t_all[i] + (26.0 / 120.0) * t_all[i + 1] + (33.0 / 60.0) * t_all[i + 2] + (13.0 / 60.0) * t_all[i + 3] + (1.0 / 120.0) * t_all[i + 4];
            t_coefficients_[i][1] = (-5.0 / 120.0) * t_all[i] + (-50.0 / 120.0) * t_all[i + 1] + (25.0 / 60.0) * t_all[i + 3] + (5.0 / 120.0) * t_all[i + 4];
            t_coefficients_[i][2] = (10.0 / 120.0) * t_all[i] + (20.0 / 120.0) * t_all[i + 1] + (-30.0 / 60.0) * t_all[i + 2] + (10.0 / 60.0) * t_all[i + 3] + (10.0 / 120.0) * t_all[i + 4];
            t_coefficients_[i][3] = (-10.0 / 120.0) * t_all[i] + (20.0 / 120.0) * t_all[i + 1] + (-10.0 / 60.0) * t_all[i + 3] + (10.0 / 120.0) * t_all[i + 4];
            t_coefficients_[i][4] = (5.0 / 120.0) * t_all[i] + (-20.0 / 120.0) * t_all[i + 1] + (15.0 / 60.0) * t_all[i + 2] + (-10.0 / 60.0) * t_all[i + 3] + (5.0 / 120.0) * t_all[i + 4];
            t_coefficients_[i][5] = (-1.0 / 120.0) * t_all[i] + (5.0 / 120.0) * t_all[i + 1] + (-5.0 / 60.0) * t_all[i + 2] + (5.0 / 60.0) * t_all[i + 3] + (-5.0 / 120.0) * t_all[i + 4] + (1.0 / 120.0) * t_all[i + 5];
        }

    }
    ~QuinticBSplineTrajectory() = default;

    // Generate scatter point
    Point3f generateTrajPoint(double u) const {
        double s = sValue(u);
        double d = dValue(u);
        double t = tValue(u);
        Point3f cur_traj_point = Point3f(s, d, t);
        return cur_traj_point;
    }

    /**
     * @brief Generate the whole trajectory
     * @param sample_gap the gap in coefficient sampling, not the gap in world frame or frenet frame
     * @return the final trajectory
     */    
    std::vector<Point3f> generateTraj(double sample_gap) {
        int sample_num = static_cast<int>(static_cast<double>(segment_num_) / sample_gap);
        std::vector<double> samples = Tools::linspace(0.0, static_cast<double>(segment_num_), sample_num);
        std::vector<Point3f> trajectory;

        for (const auto sample : samples) {
            trajectory.emplace_back(generateTrajPoint(sample));
        }

        return trajectory;
    }



 private:
    // Verify the input
    double inputVerify(double u) const  {
        if (Tools::isSmall(u, 0.0)) {
            return 0.0;
        } else if (Tools::isLarge(u, static_cast<double>(segment_num_))){
            return static_cast<double>(segment_num_);
        } else {
            return u;
        }
    }

    // Generate segment information, i.e., the index of segment, the parameter in this specified segment
    std::pair<int, double> getSegmentInfo(double u) const  {
        u = this->inputVerify(u);
        for (int i = 0; i < this->segment_num_; i++) {
            if (Tools::isSmall(u, static_cast<double>(i + 1))) {
                double remain = u - i;
                std::pair<int, double> result{i, remain};
                return result;
            }
        }
        std::pair<int, double> result{segment_num_ - 1, 1.0};
        return result;
    }

    // Calculate s
    double sValue(double u) const  {
        u = inputVerify(u);
        // 得到分段和参数
        auto segment_info = getSegmentInfo(u);
        u = segment_info.second;
        int index = segment_info.first;
        return s_coefficients_[index][0] + s_coefficients_[index][1] * u + s_coefficients_[index][2] * u * u + s_coefficients_[index][3] * u * u * u + s_coefficients_[index][4] * u * u * u * u + s_coefficients_[index][5] * u * u * u * u * u;
    }
    
    // Calculate d
    double dValue(double u) const  {
        u = inputVerify(u);
        // 得到分段和参数
        auto segment_info = getSegmentInfo(u);
        u = segment_info.second;
        int index = segment_info.first;
        return d_coefficients_[index][0] + d_coefficients_[index][1] * u + d_coefficients_[index][2] * u * u + d_coefficients_[index][3] * u * u * u + d_coefficients_[index][4] * u * u * u * u + d_coefficients_[index][5] * u * u * u * u * u;
    }

    // Calculate t
    double tValue(double u) const  {
        u = inputVerify(u);
        // 得到分段和参数
        auto segment_info = getSegmentInfo(u);
        u = segment_info.second;
        int index = segment_info.first;
        return t_coefficients_[index][0] + t_coefficients_[index][1] * u + t_coefficients_[index][2] * u * u + t_coefficients_[index][3] * u * u * u + t_coefficients_[index][4] * u * u * u * u + t_coefficients_[index][5] * u * u * u * u * u;
    }


    int segment_num_;
    std::vector<std::vector<double>> s_coefficients_;
    std::vector<std::vector<double>> d_coefficients_;
    std::vector<std::vector<double>> t_coefficients_;

};



// Trajectory planning core
class TrajectoryPlanningCore {
 public:
    TrajectoryPlanningCore() = default;
    ~TrajectoryPlanningCore() = default;

    // Load data
    void load(const Vehicle& cur_vehicle_state, const Lane& reference_lane, const std::vector<Vehicle>& ego_traj, const std::unordered_map<int, std::vector<Vehicle>>& sur_laned_veh_trajs, const std::vector<Obstacle>& sur_unlaned_obs) {
        current_vehicle_state_ = cur_vehicle_state;
        reference_lane_ = reference_lane;
        ego_traj_ = ego_traj;
        sur_laned_veh_trajs_ = sur_laned_veh_trajs;
        sur_unlaned_obs_ = sur_unlaned_obs;
    }

    // Run trajectory planner
    void runOnce(bool* result, std::vector<Point3f>* trajectory) {
        // ~Stage I: construct bridge and transform information
        bridge_itf_ = new BpTpBridge(reference_lane_);
        FsVehicle current_vehicle_state_fs = bridge_itf_->getFsVehicle(current_vehicle_state_);
        std::vector<FsVehicle> ego_traj_fs = bridge_itf_->getEgoFrenetTrajectory(ego_traj_);

        // // DEBUG 
        // for (auto& veh_fs : ego_traj_fs) {
        //     veh_fs.fs_.print();
        // }
        // // END DEBUG

        std::unordered_map<int, std::vector<FsVehicle>> sur_laned_trajs_fs = bridge_itf_->getSurFrenetTrajectories(sur_laned_veh_trajs_);
        std::unordered_map<int, std::vector<FsVehicle>> sur_unlaned_trajs_fs = bridge_itf_->getUnlanedSurFrenetTrajectories(sur_unlaned_obs_);

        // ~Stage II: contruct traj planning 3d grid map and generate semantic cubes
        TrajPlanning3DMap::Config config;
        traj_planning_3d_map_itf_ = new TrajPlanning3DMap(config);
        std::vector<SemanticCube<double>> semantic_cubes_sequence;
        bool is_map_constructed_success = traj_planning_3d_map_itf_->runOnce(ego_traj_fs, sur_laned_trajs_fs, sur_unlaned_trajs_fs, &semantic_cubes_sequence);
        if (!is_map_constructed_success) {
            printf("[TrajectoryPlanningCore] traj planning 3d grid map constructed failed.\n");
            *result = false;
            return;
        }

        // DEBUG
        for (auto& sem_cube : semantic_cubes_sequence) {
            sem_cube.print();
        }
        // END DEBUG

        // ~Stage III: determine constraints conditions and do optimization
        EqualConstraint start_constraints, end_constraints;
        start_constraints.load(current_vehicle_state_fs);
        end_constraints.load(ego_traj_fs.back());
        std::vector<double> ref_stamps;
        for (const auto& ego_state : ego_traj_fs) {
            ref_stamps.emplace_back(ego_state.fs_.time_stamp_);
        }
        traj_opt_itf_ = new TrajectoryOptimizer();
        traj_opt_itf_->load(start_constraints, end_constraints, semantic_cubes_sequence, ref_stamps);
        std::vector<double> optimized_s, optimized_d, t;
        bool is_optimization_success = false;
        traj_opt_itf_->runOnce(&optimized_s, &optimized_d, &t, &is_optimization_success);
        if (!is_optimization_success) {
            printf("[TrajectoryPlanningCore] traj optimization failed.\n");
            *result = false;
            return;
        }

        // ~Stage IV: calculate quintic B-spline trajectory in frenet frame
        B_spline_traj_itf_ = new QuinticBSplineTrajectory(optimized_s, optimized_d, t);
        std::vector<Point3f> traj_fs = B_spline_traj_itf_->generateTraj(0.01);

        // ~Stage V: transform the trajectory from frenet to world
        std::vector<Point3f> traj = bridge_itf_->getTrajFromTrajFs(traj_fs);

        *trajectory = traj;
        *result = true;
    }

    QuinticBSplineTrajectory* B_spline_traj_itf_{nullptr};
    TrajPlanning3DMap* traj_planning_3d_map_itf_{nullptr};
    BpTpBridge* bridge_itf_{nullptr};
    TrajectoryOptimizer* traj_opt_itf_{nullptr};

    Vehicle current_vehicle_state_;
    Lane reference_lane_;
    std::vector<Vehicle> ego_traj_;
    std::unordered_map<int, std::vector<Vehicle>> sur_laned_veh_trajs_;
    std::vector<Obstacle> sur_unlaned_obs_;
    
};





} // End of namespace trajectory planner

