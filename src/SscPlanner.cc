/*
 * @Author: fujiawei0724
 * @Date: 2021-12-09 19:59:05
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-12-22 20:14:58
 * @Description: Components for ssc planner.
 */

#include "Common.hpp"

namespace SscPlanner {

    SscPlanning3DMap::SscPlanning3DMap() = default;
    SscPlanning3DMap::SscPlanning3DMap(const Config& config) {
        config_ = config;
        p_3d_grid_ = new GridMap3D(config_.map_size, config_.map_resolution, config_.axis_name);
    }
    SscPlanning3DMap::~SscPlanning3DMap() = default;

    /**
     * @brief Load data, set initial fs and origin
     * @param {*}
     * @return {*}
     */    
    void SscPlanning3DMap::load(const FrenetState& initial_fs) {
        initial_fs_ = initial_fs;
        std::array<double, 3> map_origin;
        map_origin[0] = initial_fs.vec_s_[0] - config_.s_back_len;
        map_origin[1] = -1.0 * (config_.map_size[1] - 1) * config_.map_resolution[1] / 2.0;
        map_origin[2] = initial_fs.time_stamp_;
        p_3d_grid_->set_origin(map_origin);
    }

    /**
     * @brief Run once to generate the semantic cubes sequence
     * @param {*}
     * @return {*}
     */
    bool SscPlanning3DMap::runOnce(const std::vector<FsVehicle>& ego_traj, const std::unordered_map<int, std::vector<FsVehicle>>& surround_laned_trajs, const std::unordered_map<int, std::vector<FsVehicle>>& surround_unlaned_obs_trajs, DrivingCorridorWorldMetric* semantic_cubes_sequence) {
        // ~Stage I: construct 3D grid map
        clock_t constructed_3dmap_start_time = clock();
        contruct3DMap(surround_laned_trajs);
        contruct3DMap(surround_unlaned_obs_trajs);
        clock_t constructed_3dmap_end_time = clock();
        printf("[SscPlanner] construct 3d map time consumption: %lf.\n", static_cast<double>((constructed_3dmap_end_time - constructed_3dmap_start_time)) / CLOCKS_PER_SEC);

        // ~Stage II: generate corridor in coord frame
        DrivingCorridor driving_corridor;
        clock_t generate_corridor_start_time = clock();
        bool is_success = generateCorridor(ego_traj, &driving_corridor);
        clock_t generate_corridor_end_time = clock();
        printf("[SscPlanner] generate corridor time consumption: %lf.\n", static_cast<double>((generate_corridor_end_time - generate_corridor_start_time)) / CLOCKS_PER_SEC);
        if (!is_success) {
            return false;
        }

        // ~Stage III: transform to world metric
        DrivingCorridorWorldMetric semantic_cubes;
        getFinalGlobalMetricCubesList(driving_corridor, &semantic_cubes);
        *semantic_cubes_sequence = semantic_cubes;

        return true;
    }    

    /**
     * @brief Get final global metric cube list
     * @param driving_corridor description of semantic cubes in coord index
     * @param semantic_cubes final cubes list
     */    
    void SscPlanning3DMap::getFinalGlobalMetricCubesList(const DrivingCorridor& driving_corridor, DrivingCorridorWorldMetric* driving_corridor_world_metric) {
        driving_corridor_world_metric->is_valid = driving_corridor.is_valid;
        for (int i = 0; i < static_cast<int>(driving_corridor.cubes.size()); i++) {
            DrivingCubeWorldMetric current_driving_cube_world_metric;
            transformCoordCubeToWorldMetric(driving_corridor.cubes[i], &current_driving_cube_world_metric);
            driving_corridor_world_metric->cubes.emplace_back(current_driving_cube_world_metric);
        }
    }

    /**
     * @brief Transform cube from coordination to world metric
     * @param {*}
     * @return {*}
     */    
    void SscPlanning3DMap::transformCoordCubeToWorldMetric(const DrivingCube& driving_cube, DrivingCubeWorldMetric* cube_world_metric) {
        // Transform cube information
        SemanticCube<double> semantic_cube_world;
        transformCoordSemCubeToSemCube(driving_cube.cube, &semantic_cube_world);
        
        // Transform point information
        std::vector<Point3f> world_points;
        for (int i = 0; i < static_cast<int>(driving_cube.seeds.size()); i++) {
            Point3f current_point;
            transformCoordToWorldMetric(driving_cube.seeds[i], &current_point);
            world_points.emplace_back(current_point);
        }

        cube_world_metric->cube = semantic_cube_world;
        cube_world_metric->seeds = world_points;
    }

    /**
     * @brief Transform coord semantic cube to semantic cube
     * @param coord_semantic_cube semantic cube in the formation of coordinate
     * @param semantic_cube semantic cube in the formation of value
     */    
    void SscPlanning3DMap::transformCoordSemCubeToSemCube(const SemanticCube<int>& coord_semantic_cube, SemanticCube<double>* semantic_cube) {
        semantic_cube->s_start_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_semantic_cube.s_start_, 0);
        semantic_cube->s_end_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_semantic_cube.s_end_, 0);
        semantic_cube->d_start_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_semantic_cube.d_start_, 1);
        semantic_cube->d_end_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_semantic_cube.d_end_, 1);
        semantic_cube->t_start_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_semantic_cube.t_start_, 2);
        semantic_cube->t_end_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_semantic_cube.t_end_, 2);
        semantic_cube->id_ = coord_semantic_cube.id_;
    }

    /**
     * @brief Transform coord point3i to world metric point3f
     * @param {*}
     * @return {*}
     */
    void SscPlanning3DMap::transformCoordToWorldMetric(const Point3i& coord_point, Point3f* world_point) {
        world_point->x_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_point.x_, 0);
        world_point->y_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_point.y_, 1);
        world_point->z_ = p_3d_grid_->getGloalMetricUsingCoordOnSingleDim(coord_point.z_, 2);
    } 


    /**
     * @brief Construct corridors using initial trajectory 
     * @param ego_traj ego vehicle's scatter trajectory state
     * @param corridor generated corridor 
     * @return is success
     */    
    bool SscPlanning3DMap::generateCorridor(const std::vector<FsVehicle>& ego_traj, DrivingCorridor* corridor) {
        // Generate seeds
        std::vector<Point3i> traj_seeds;
        if (!constructSeeds(ego_traj, &traj_seeds)) {
            printf("[Ssc planning] fail to construct seeds.\n");
            return false;
        }

        // Inflate cubes
        DrivingCorridor driving_corridor;
        bool is_valid = true;
        auto seed_num = static_cast<int>(traj_seeds.size());
        if (seed_num < 2) {
            driving_corridor.is_valid = false;
            is_valid = false;
            printf("[Ssc planning] trajectory seeds' number is not suitable.\n");
            return false;
        }
        for (int i = 0; i < seed_num; i++) {
            if (i == 0) {
                SemanticCube<int> coord_semantic_cube = ShapeUtils::generateInitialCoordSemanticCube(traj_seeds[i], traj_seeds[i + 1], i);
                if (!checkIfSemanticCubeIsFree(coord_semantic_cube)) {
                    printf("[Ssc planning] initial cube is not free, seed id: %d.\n", i);

                    DrivingCube driving_cube;
                    driving_cube.cube = coord_semantic_cube;
                    driving_cube.seeds.emplace_back(traj_seeds[i]);
                    driving_cube.seeds.emplace_back(traj_seeds[i + 1]);
                    driving_corridor.cubes.emplace_back(driving_cube);

                    driving_corridor.is_valid = false;
                    is_valid = false;
                    break;
                }

                inflateCube(&coord_semantic_cube);
                DrivingCube driving_cube;
                driving_cube.cube = coord_semantic_cube;
                driving_cube.seeds.emplace_back(traj_seeds[i]);
                driving_corridor.cubes.emplace_back(driving_cube);
            } else {
                if (checkIfCubeContainsSeed(driving_corridor.cubes.back().cube, traj_seeds[i])) {
                    driving_corridor.cubes.back().seeds.emplace_back(traj_seeds[i]);
                    continue;
                } else {
                    // Get the last seed in cube 
                    Point3i seed_r = driving_corridor.cubes.back().seeds.back();
                    driving_corridor.cubes.back().seeds.pop_back();
                    // Cut cube on time axis
                    driving_corridor.cubes.back().cube.t_end_ = seed_r.z_;
                    i = i - 1;

                    // TODO: check the id order of cube in ssc planning
                    SemanticCube<int> cube = ShapeUtils::generateInitialCoordSemanticCube(traj_seeds[i], traj_seeds[i + 1]);

                    if (!checkIfSemanticCubeIsFree(cube)) {
                        printf("[Ssc planning] initial cube is not free, seed id: %d.\n", i);
                        
                        DrivingCube driving_cube;
                        driving_cube.cube = cube;
                        driving_cube.seeds.emplace_back(traj_seeds[i]);
                        driving_cube.seeds.emplace_back(traj_seeds[i + 1]);
                        driving_corridor.cubes.emplace_back(driving_cube);

                        driving_corridor.is_valid = false;
                        is_valid = false;
                        break;
                    }

                    inflateCube(&cube);
                    DrivingCube driving_cube;
                    driving_cube.cube = cube;
                    driving_cube.seeds.emplace_back(traj_seeds[i]);
                    driving_corridor.cubes.emplace_back(driving_cube);
                }
            }
        }

        if (is_valid) {
            // Limit the last cube's time upper bound
            driving_corridor.cubes.back().cube.t_end_ = traj_seeds.back().z_;
            driving_corridor.is_valid = true;
        }

        *corridor = driving_corridor;
        return true;
    }

    /**
     * @brief Judge whether semantic cube contains seed
     * @param cube semantic cube need to judge
     * @param seed trajectory seed need to judge
     * @return result
     */    
    bool SscPlanning3DMap::checkIfCubeContainsSeed(const SemanticCube<int>& cube, const Point3i& seed) {
        if (cube.s_start_ > seed.x_ || cube.s_end_ < seed.x_) {
            return false;
        }
        if (cube.d_start_ > seed.y_ || cube.d_end_ < seed.y_) {
            return false;
        }
        if (cube.t_start_ > seed.z_ || cube.t_end_ < seed.z_) {
            return false;
        }
        return true;
    }



    /**
     * @brief Inflate the cube in 3D grid map until reached to the occupied space
     * @param init_seman_cube the initial scale of semantic cube
     */    
    void SscPlanning3DMap::inflateCube(SemanticCube<int>* cube) {
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
        // s_idx_l = std::max(s_idx_l, cube->s_start_ - static_cast<int>(10.0 / config_.map_resolution[0]));

        // Inflate in 5 directions
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

            // TODO: check this t dimension inflate constraint condition, if it is too harsh
            if (cube->t_end_ - cube->t_start_ >= config_.MaxNumOfGridAlongTime) {
                z_p_finish = true;
            }
        }
    }

    /**
     * @brief Judge whether the semantic cube is free 
     * @param semantic_coord_cube cube needs to judge
     * @return is free
     */    
    bool SscPlanning3DMap::checkIfSemanticCubeIsFree(const SemanticCube<int>& semantic_coord_cube) {
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
    bool SscPlanning3DMap::inflateCubeOnXPosAxis(const int& n_step, SemanticCube<int>* cube) {
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
    bool SscPlanning3DMap::inflateCubeOnXNegAxis(const int& n_step, SemanticCube<int>* cube) {
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
    bool SscPlanning3DMap::inflateCubeOnYPosAxis(const int& n_step, SemanticCube<int>* cube) {
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
    bool SscPlanning3DMap::inflateCubeOnYNegAxis(const int& n_step, SemanticCube<int>* cube) {
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
    bool SscPlanning3DMap::inflateCubeOnZPosAxis(const int& n_step, SemanticCube<int>* cube) {
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
    bool SscPlanning3DMap::checkIfPlaneIsFreeOnXAxis(const SemanticCube<int>& cube, const int& x) const {
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
    bool SscPlanning3DMap::checkIfPlaneIsFreeOnYAxis(const SemanticCube<int>& cube, const int& y) const {
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
    bool SscPlanning3DMap::checkIfPlaneIsFreeOnZAxis(const SemanticCube<int>& cube, const int& z) const {
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
    bool SscPlanning3DMap::constructSeeds(const std::vector<FsVehicle>& ego_traj, std::vector<Point3i>* seeds) {
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

        // // DEBUG
        // for (int i = 0; i < static_cast<int>(tmp_seeds.size()); i++) {
        //     printf("seed index: %d\n", i);
        //     printf("x: %d, y: %d, z: %d\n", tmp_seeds[i].x_, tmp_seeds[i].y_, tmp_seeds[i].z_);
        // }
        // // END DEBUG

        // Cache 
        *seeds = tmp_seeds;
        
        return true;
    }

    // Construct 3D map using dynamic obstacles and static obstacles information
    void SscPlanning3DMap::contruct3DMap(const std::unordered_map<int, std::vector<FsVehicle>>& surround_trajs) {
        fillDynamicObstacles(surround_trajs);
        fillStaticObstacles();
    }

    // Fill the dynamic obstacles information
    void SscPlanning3DMap::fillDynamicObstacles(const std::unordered_map<int, std::vector<FsVehicle>>& surround_trajs) {
        for (auto iter = surround_trajs.begin(); iter != surround_trajs.end(); iter++) {
            fillSingleDynamicObstacle(iter->second);
        }
    }

    void SscPlanning3DMap::fillSingleDynamicObstacle(const std::vector<FsVehicle>& fs_traj) {
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
    void SscPlanning3DMap::fillStaticObstacles() {

    }

    SscOptimizationInterface::SscOptimizationInterface() = default;
    SscOptimizationInterface::~SscOptimizationInterface() = default;

    /**
     * @brief load data
     * @param ref_stamps time stamps of the point in in the intersection of two cubes
     * @param start_constraints start points' constraints
     * @param end_constraints end points' constraints
     * @param unequal_constraints position limit of each point
     * @param equal_constraints ensure the continuity of the connections between each two cubes
     */    
    void SscOptimizationInterface::load(const std::vector<double>& ref_stamps, const EqualConstraint& start_constraints, const EqualConstraint& end_constraints, std::array<std::vector<double>, 4>& unequal_constraints, std::vector<std::vector<double>>& equal_constraints) {
        ref_stamps_ = ref_stamps;
        start_constraints_ = start_constraints;
        end_constraints_ = end_constraints;
        unequal_constraints_ = unequal_constraints;
        equal_constraints_ = equal_constraints;
    }

    /**
     * @brief Run optimization
     * @param {*}
     * @return {*}
     */    
    void SscOptimizationInterface::runOnce(std::vector<double>* optimized_s, std::vector<double>* optimized_d) {
        // Prepare data for s and d dimensions
        std::array<double, 3> s_start_constraints = start_constraints_.toDimensionS();
        std::array<double, 3> s_end_constraints = end_constraints_.toDimensionS();
        std::array<std::vector<double>, 2> s_unequal_constraints = {unequal_constraints_[0], unequal_constraints_[1]};
        std::array<double, 3> d_start_constraints = start_constraints_.toDimensionD();
        std::array<double, 3> d_end_constraints = end_constraints_.toDimensionD();
        std::array<std::vector<double>, 2> d_unequal_constraints = {unequal_constraints_[2], unequal_constraints_[3]};

        // Multi thread calculation
        // TODO: add logic to handle the situation where the optimization process is failed
        clock_t single_dim_optimization_start_time = clock();
        std::thread s_thread(&SscOptimizationInterface::optimizeSingleDim, this, s_start_constraints, s_end_constraints, s_unequal_constraints[0], s_unequal_constraints[1], "s");
        std::thread d_thread(&SscOptimizationInterface::optimizeSingleDim, this, d_start_constraints, d_end_constraints, d_unequal_constraints[0], d_unequal_constraints[1], "d");
        s_thread.join();
        d_thread.join();
        clock_t single_dim_optimization_end_time = clock();
        printf("[SscPlanner] single dimension optimization time consumption: %lf.\n", static_cast<double>((single_dim_optimization_end_time - single_dim_optimization_start_time)) / CLOCKS_PER_SEC);

        // // DEBUG
        // optimizeSingleDim(s_start_constraints, s_end_constraints, s_unequal_constraints[0], s_unequal_constraints[1], "s");
        // optimizeSingleDim(d_start_constraints, d_end_constraints, d_unequal_constraints[0], d_unequal_constraints[1], "d");
        // // END DEBUG

        // Cache
        *optimized_s = optimized_data_["s"];
        *optimized_d = optimized_data_["d"];
    }

    /**
     * @brief Optimize in single dimension
     * @param {*}
     */
    void SscOptimizationInterface::optimizeSingleDim(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries, std::string dimension_name) {
        // ~Stage I: calculate D and c matrices (objective function)
        std::vector<double*> D;
        double* c = nullptr;
        calculateDcMatrix(&D, &c);
        double c0 = 0.0;

        // // DEBUG
        // for (int i = 0; i < 18; i++) {
        //     for (int j = 0; j < 18; j++) {
        //         std::cout << *(D[i] + j) << ", ";
        //     }
        // }
        // std::cout << "--------------------------" << std::endl;;

        // for (int i = 0; i < 18; i++) {
        //     std::cout << *(c + i) << ", ";
        // }
        // std::cout << "--------------------------" << std::endl;
        // // END DEBUG


        // ~Stage II: calculate equal constraints, includes start point constraints, end point constraints and continuity constraints
        CGAL::Const_oneset_iterator<CGAL::Comparison_result> r(CGAL::EQUAL);
        std::vector<double*> A;
        double* b = nullptr;
        calculateAbMatrix(single_start_constraints, single_end_constraints, equal_constraints_, &A, &b);

        // // DEBUG
        // for (int i = 0; i < 18; i++) {
        //     for (int j = 0; j < 12; j++) {
        //         std::cout << *(A[i] + j) << ", ";
        //     }
        // }
        // std::cout << "---------------------------" << std::endl;
        // for (int i = 0; i < 12; i++) {
        //     std::cout << *(b + i) << ", ";
        // }
        // std::cout << "---------------------------" << std::endl;
        // // END DEBUG

        // ~Stage III: calculate low and up boundaries for intermediate points
        bool* fl = nullptr;
        double* l = nullptr;
        bool* fu = nullptr;
        double* u = nullptr;
        calculateBoundariesForIntermediatePoints(single_lower_boundaries, single_upper_boundaries, &fl, &l, &fu, &u);

        // // DEBUG
        // std::cout << "fl" << std::endl;
        // for (int i = 0; i < 18; i++) {
        //     std::cout << *(fl + i) << ", ";
        // }
        // std::cout << std::endl;
        // std::cout << "l" << std::endl;
        // for (int i = 0; i < 18; i++) {
        //     std::cout << *(l + i) << ", ";
        // }
        // std::cout << std::endl;
        // std::cout << "fu" << std::endl;
        // for (int i = 0; i < 18; i++) {
        //     std::cout << *(fu + i) << ", ";
        // }
        // std::cout << std::endl;
        // std::cout << "u" << std::endl;
        // for (int i = 0; i < 18; i++) {
        //     std::cout << *(u + i) << ", ";
        // }
        // std::cout << std::endl;
        // std::cout << "-------------------" << std::endl;
        // // END DEBUG
        
        // ~Stage IV: optimization and transform the formation of optimization result
        int variables_num = (static_cast<int>(ref_stamps_.size()) - 1) * 6;
        int constraints_num = 6 + (static_cast<int>(ref_stamps_.size()) - 2) * 3;
        Program qp(variables_num, constraints_num, A.begin(), b, r, fl, l, fu, u, D.begin(), c, c0);
        Solution s = CGAL::solve_quadratic_program(qp, ET());
        // Convert data
        std::vector<double> optimized_values;
        for (auto iter = s.variable_values_begin(); iter != s.variable_values_end(); iter++) {
            double value = CGAL::to_double(*iter);
            optimized_values.emplace_back(value);
        }
        
        // ~Stage V: store information
        optimized_data_[dimension_name] = optimized_values;

    }

    /**
     * @brief Calculate boundaries for intermediate points
     * @param {*}
     */    
    void SscOptimizationInterface::calculateBoundariesForIntermediatePoints(const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries, bool** fl, double** l, bool** fu, double** u) {
        int variables_num = (static_cast<int>(ref_stamps_.size()) - 1) * 6;
        bool* tmp_fl = new bool[variables_num];
        double* tmp_l = new double[variables_num];
        bool* tmp_fu = new bool[variables_num];
        double* tmp_u = new double[variables_num];

        for (int i = 0; i < variables_num; i++) {
            if (i == 0 || i == variables_num - 1) {
                // For the first point and last point, the unequal constraints are invalid
                *(tmp_fl + i) = false;
                *(tmp_fu + i) = false;
            } else {
                *(tmp_fl + i) = true;
                *(tmp_fu + i) = true;
                *(tmp_l + i) = single_lower_boundaries[i];
                *(tmp_u + i) = single_upper_boundaries[i];
            }
        }

        // TODO: check this parameters transformation process
        *fl = tmp_fl;
        *l = tmp_l;
        *fu = tmp_fu;
        *u = tmp_u;
    }

    /**
     * @brief Calculate equal constraints, note that position constraints in the connection don't need to be considered
     * @param {*}
     */
    void SscOptimizationInterface::calculateAbMatrix(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, const std::vector<std::vector<double>>& equal_constraints, std::vector<double*>* A, double** b) {
        
        // Calculate dimensions and initialize
        int variables_num = (static_cast<int>(ref_stamps_.size()) - 1) * 6;
        int equal_constraints_num = 6 + (static_cast<int>(ref_stamps_.size()) - 2) * 3;
        double start_cube_time_span = ref_stamps_[1] - ref_stamps_[0];
        double end_cube_time_span = ref_stamps_[ref_stamps_.size() - 1] - ref_stamps_[ref_stamps_.size() - 2];
        Eigen::MatrixXd A_matrix = Eigen::MatrixXd::Zero(equal_constraints_num, variables_num);
        Eigen::MatrixXd b_matrix = Eigen::MatrixXd::Zero(equal_constraints_num, 1);

        // Supple start point and end point position constraints 
        A_matrix(0, 0) = 1.0, A_matrix(1, variables_num - 1) = 1.0;
        b_matrix(0, 0) = single_start_constraints[0], b_matrix(1, 0) = single_end_constraints[0];

        // Supple start point and end point velocity constraints
        A_matrix(2, 0) = -5.0, A_matrix(2, 1) = 5.0;
        b_matrix(2, 0) = single_start_constraints[1] * start_cube_time_span;
        A_matrix(3, variables_num - 2) = -5.0, A_matrix(3, variables_num - 1) = 5.0;
        b_matrix(3, 0) = single_end_constraints[1] * end_cube_time_span;

        // Supple start point and end point acceleration constraints
        A_matrix(4, 0) = 20.0, A_matrix(4, 1) = -40.0, A_matrix(4, 2) = 20.0;
        b_matrix(4, 0) = single_start_constraints[2] * start_cube_time_span;
        A_matrix(5, variables_num - 3) = 20.0, A_matrix(5, variables_num - 2) = -40.0, A_matrix(5, variables_num - 1) = 20.0;
        b_matrix(5, 0) = single_end_constraints[2] * end_cube_time_span;

        // Supple continuity ensurance constraints
        for (int i = 0; i < static_cast<int>(equal_constraints.size()); i++) {
            int constraint_index = i + 6;
            for (int j = 0; j < variables_num; j++) {
                
                // DEBUG: check this logic
                assert(static_cast<int>(equal_constraints[i].size()) == variables_num);
                // END DEBUG

                A_matrix(constraint_index, j) = equal_constraints[i][j];
            }
        }

        // // DEBUG
        // for (int i = 0; i < static_cast<int>(A_matrix.rows()); i++) {
        //     for (int j = 0; j < static_cast<int>(A_matrix.cols()); j++) {
        //         printf("%lf, ", A_matrix(i, j));
        //     }
        // }
        // std::cout << "---------------------------";
        // std::cout << std::endl;
        // // END DEBUG

        // // DEBUG
        // std::cout << b_matrix << std::endl;
        // // EDN DEBUG
        

        // Transform data structure 
        std::vector<double*> tmp_A(variables_num);
        for (int i = 0; i < variables_num; i++) {
            double* a_col = new double[equal_constraints_num];
            for (int j = 0; j < equal_constraints_num; j++) {
                *(a_col + j) = A_matrix(j, i);
            }
            tmp_A[i] = a_col;
        }
        double* tmp_b = new double[equal_constraints_num];
        for (int i = 0; i < equal_constraints_num; i++) {
            *(tmp_b + i) = b_matrix(i, 0);
        }

        *A = tmp_A;
        *b = tmp_b;

    }

    /**
     * @brief Calculate the matrices related to objective function, for both s and d dimensions, D and c has the same value
     * @param D 
     * @param c
     */
    void SscOptimizationInterface::calculateDcMatrix(std::vector<double*>* D, double** c) {
        
        // Initialize D matrix
        int variables_num = (static_cast<int>(ref_stamps_.size()) - 1) * 6;
        Eigen::MatrixXd D_matrix = Eigen::MatrixXd::Zero(variables_num, variables_num);

        // Calculate D matrix
        for (int i = 0; i < static_cast<int>(ref_stamps_.size()) - 1; i++) {
            // Calculate time span
            double time_span = ref_stamps_[i + 1] - ref_stamps_[i];
            double time_coefficient = pow(time_span, -3);

            // Intergrate to objective function
            int influenced_variable_index = i * 6;
            D_matrix.block(influenced_variable_index, influenced_variable_index, 6, 6) += BezierCurveHessianMatrix * time_coefficient;
        }

        // // DEBUG
        // for (int i = 0; i < static_cast<int>(D_matrix.rows()); i++) {
        //     for (int j = 0; j < static_cast<int>(D_matrix.cols()); j++) {
        //         printf("%lf, ", D_matrix(i, j));
        //     }
        // }
        // printf("----------------------------------\n");
        // // END DEBUG

        // Convert the eigen data to double**
        std::vector<double*> tmp_D(variables_num);
        for (int i = 0; i < variables_num; i++) {
            double* d_col = new double[variables_num];
            for (int j = 0; j < variables_num; j++) {
                *(d_col + j) = D_matrix(i, j);
            }
            tmp_D[i] = d_col;
        }

        // // DEBUG
        // for (int i = 0; i < variables_num; i++) {
        //     for (int j = 0; j <= i; j++) {
        //         printf("%lf ", *(tmp_D[i] + j));
        //     }
        //     printf("-----------------\n");
        // }
        // // END DEBUG

        // Generate c information, all zeros
        double* tmp_c = new double[variables_num];
        for (int i = 0; i < variables_num; i++) {
            *(tmp_c + i) = 0.0;
        }

        // TODO: check this parameters transformation process
        *D = tmp_D;
        *c = tmp_c;
    }

    OoqpOptimizationInterface::OoqpOptimizationInterface() = default;
    OoqpOptimizationInterface::~OoqpOptimizationInterface() = default;

    /**
     * @brief load data
     * @param ref_stamps time stamps of the point in in the intersection of two cubes
     * @param start_constraints start points' constraints
     * @param end_constraints end points' constraints
     * @param unequal_constraints position limit of each point
     * @param equal_constraints ensure the continuity of the connections between each two cubes
     */    
    void OoqpOptimizationInterface::load(const std::vector<double>& ref_stamps, const EqualConstraint& start_constraints, const EqualConstraint& end_constraints, std::array<std::vector<double>, 4>& unequal_constraints, std::vector<std::vector<double>>& equal_constraints) {
        ref_stamps_ = ref_stamps;
        start_constraints_ = start_constraints;
        end_constraints_ = end_constraints;
        unequal_constraints_ = unequal_constraints;
        equal_constraints_ = equal_constraints;
    }

    /**
     * @brief Run optimization
     * @param {*}
     * @return {*}
     */    
    bool OoqpOptimizationInterface::runOnce(std::vector<double>* optimized_s, std::vector<double>* optimized_d) {
        // Prepare data for s and d dimensions
        std::array<double, 3> s_start_constraints = start_constraints_.toDimensionS();
        std::array<double, 3> s_end_constraints = end_constraints_.toDimensionS();
        std::array<std::vector<double>, 2> s_unequal_constraints = {unequal_constraints_[0], unequal_constraints_[1]};
        std::array<double, 3> d_start_constraints = start_constraints_.toDimensionD();
        std::array<double, 3> d_end_constraints = end_constraints_.toDimensionD();
        std::array<std::vector<double>, 2> d_unequal_constraints = {unequal_constraints_[2], unequal_constraints_[3]};

        // Multi thread calculation
        // TODO: add logic to handle the situation where the optimization process is failed
        clock_t single_dim_optimization_start_time = clock();
        std::thread s_thread(&OoqpOptimizationInterface::optimizeSingleDim, this, s_start_constraints, s_end_constraints, s_unequal_constraints[0], s_unequal_constraints[1], "s");
        std::thread d_thread(&OoqpOptimizationInterface::optimizeSingleDim, this, d_start_constraints, d_end_constraints, d_unequal_constraints[0], d_unequal_constraints[1], "d");
        s_thread.join();
        d_thread.join();
        clock_t single_dim_optimization_end_time = clock();
        printf("[SscPlanner] single dimension optimization time consumption: %lf.\n", static_cast<double>((single_dim_optimization_end_time - single_dim_optimization_start_time)) / CLOCKS_PER_SEC);

        // // DEBUG
        // optimizeSingleDim(s_start_constraints, s_end_constraints, s_unequal_constraints[0], s_unequal_constraints[1], "s");
        // optimizeSingleDim(d_start_constraints, d_end_constraints, d_unequal_constraints[0], d_unequal_constraints[1], "d");
        // // END DEBUG

        // Cache
        *optimized_s = optimized_data_["s"];
        *optimized_d = optimized_data_["d"];

        bool final_res = optimization_res_["s"] && optimization_res_["d"];

        return final_res;
    }

    /**
     * @brief Optimize in single dimension
     * @param {*}
     */
    void OoqpOptimizationInterface::optimizeSingleDim(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries, std::string dimension_name) {
        // ~Stage I: calculate objective function Q and c
        Eigen::SparseMatrix<double, Eigen::RowMajor> Q;
        Eigen::VectorXd c;
        calculateQcMatrix(Q, c);

        // // DEBUG 
        // std::cout << "Q: " << Q << std::endl;
        // std::cout << "c: " << c << std::endl; 
        // // END DEBUG

        // ~Stage II: calculate equal constraints, includes start point constraints, end point constraints and continuity constraints
        Eigen::SparseMatrix<double, Eigen::RowMajor> A;
        Eigen::VectorXd b;
        calculateAbMatrix(single_start_constraints, single_end_constraints, equal_constraints_, A, b);

        // // DEBUG
        // std::cout << "A: " << A << std::endl;
        // std::cout << "b: " << b << std::endl;
        // // END DEBUG

        // ~Stage III: calculate low and up boundaries for intermediate points
        Eigen::Matrix<char, Eigen::Dynamic, 1> useLowerLimitForX;
        Eigen::Matrix<char, Eigen::Dynamic, 1> useUpperLimitForX;
        Eigen::VectorXd lowerLimitForX;
        Eigen::VectorXd upperLimitForX;
        calculateBoundariesForIntermediatePoints(single_lower_boundaries, single_upper_boundaries, useLowerLimitForX, useUpperLimitForX, lowerLimitForX, upperLimitForX);

        // // DEBUG
        // std::cout << "lowerLimitForX: " << lowerLimitForX << std::endl;
        // std::cout << "upperLimitForX: " << upperLimitForX << std::endl;
        // for (int i = 0; i < useLowerLimitForX.rows(); i++) {
        //     printf("%d\n", useLowerLimitForX(i, 0));
        //     printf("%d\n", useUpperLimitForX(i, 0));
        // }
        // std::cout << "------------------" << std::endl;
        // // END DEBUG

        // ~Stage IV: optimization
        std::vector<double> optimized_values;
        bool optimization_status = solve(Q, c, A, b, useLowerLimitForX, useUpperLimitForX, lowerLimitForX, upperLimitForX, &optimized_values);

        // ~Stage V: store information
        optimized_data_[dimension_name] = optimized_values;
        optimization_res_[dimension_name] = optimization_status;
    }

    /**
     * @brief calculate objective function
     * @param {*}
     * @return {*}
     */    
    void OoqpOptimizationInterface::calculateQcMatrix(Eigen::SparseMatrix<double, Eigen::RowMajor>& Q, Eigen::VectorXd& c) {
        // Initialize Q matrix
        int variables_num = (static_cast<int>(ref_stamps_.size()) - 1) * 6;
        Eigen::MatrixXd Q_matrix = Eigen::MatrixXd::Zero(variables_num, variables_num);

        // Calculate D matrix
        for (int i = 0; i < static_cast<int>(ref_stamps_.size()) - 1; i++) {
            // Calculate time span
            double time_span = ref_stamps_[i + 1] - ref_stamps_[i];
            double time_coefficient = pow(time_span, -3);

            // Intergrate to objective function
            int influenced_variable_index = i * 6;
            Q_matrix.block(influenced_variable_index, influenced_variable_index, 6, 6) += BezierCurveHessianMatrix * time_coefficient;
        }

        // // DEBUG
        // std::cout << "Q_matrix: " << Q_matrix << std::endl;
        // // END DEBUG

        // // DEBUG
        // for (int i = 0; i < Q_matrix.rows(); i++) {
        //     for (int j = 0; j < Q_matrix.cols(); j++) {
        //         std::cout << Q_matrix(i, j) << ", ";
        //     }
        //     std::cout << std::endl;
        // }
        // std::cout << "-----------------" << std::endl;
        // // END DEBUG

        Q = Q_matrix.sparseView();
        c.resize(variables_num);
        c.setZero();
    }

    /**
     * @brief Calculate equal constraints, note that position constraints in the connection don't need to be considered
     * @param {*}
     */
    void OoqpOptimizationInterface::calculateAbMatrix(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, const std::vector<std::vector<double>>& equal_constraints, Eigen::SparseMatrix<double, Eigen::RowMajor>& A, Eigen::VectorXd& b) {
                
        // Calculate dimensions and initialize
        int variables_num = (static_cast<int>(ref_stamps_.size()) - 1) * 6;
        int equal_constraints_num = 6 + (static_cast<int>(ref_stamps_.size()) - 2) * 3;
        double start_cube_time_span = ref_stamps_[1] - ref_stamps_[0];
        double end_cube_time_span = ref_stamps_[ref_stamps_.size() - 1] - ref_stamps_[ref_stamps_.size() - 2];
        Eigen::MatrixXd A_matrix = Eigen::MatrixXd::Zero(equal_constraints_num, variables_num);
        Eigen::MatrixXd b_matrix = Eigen::MatrixXd::Zero(equal_constraints_num, 1);

        // Supple start point and end point position constraints 
        A_matrix(0, 0) = 1.0, A_matrix(1, variables_num - 1) = 1.0;
        b_matrix(0, 0) = single_start_constraints[0], b_matrix(1, 0) = single_end_constraints[0];

        // Supple start point and end point velocity constraints
        A_matrix(2, 0) = -5.0, A_matrix(2, 1) = 5.0;
        b_matrix(2, 0) = single_start_constraints[1] * start_cube_time_span;
        A_matrix(3, variables_num - 2) = -5.0, A_matrix(3, variables_num - 1) = 5.0;
        b_matrix(3, 0) = single_end_constraints[1] * end_cube_time_span;

        // Supple start point and end point acceleration constraints
        A_matrix(4, 0) = 20.0, A_matrix(4, 1) = -40.0, A_matrix(4, 2) = 20.0;
        b_matrix(4, 0) = single_start_constraints[2] * start_cube_time_span;
        A_matrix(5, variables_num - 3) = 20.0, A_matrix(5, variables_num - 2) = -40.0, A_matrix(5, variables_num - 1) = 20.0;
        b_matrix(5, 0) = single_end_constraints[2] * end_cube_time_span;

        // Supple continuity ensurance constraints
        for (int i = 0; i < static_cast<int>(equal_constraints.size()); i++) {
            int constraint_index = i + 6;
            for (int j = 0; j < variables_num; j++) {
                
                // DEBUG: check this logic
                assert(static_cast<int>(equal_constraints[i].size()) == variables_num);
                // END DEBUG

                A_matrix(constraint_index, j) = equal_constraints[i][j];
            }
        }
        A = A_matrix.sparseView();
        b = b_matrix;

        // // DEBUG
        // std::cout << "A_matrix: " << std::endl;
        // for (int i = 0; i < A_matrix.rows(); i++) {
        //     for (int j = 0; j < A_matrix.cols(); j++) {
        //         std::cout << A_matrix(i, j) << ", ";
        //     }
        // }
        // std::cout << "-------------------" << std::endl;
        // std::cout << b_matrix << std::endl;
        // std::cout << "-------------------" << std::endl;
        // // END DEBUG
    }

    /**
     * @brief Calculate boundaries for intermediate points
     * @param {*}
     */    
    void OoqpOptimizationInterface::calculateBoundariesForIntermediatePoints(const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries, Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimitForX, Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimitForX, Eigen::VectorXd& lowerLimitForX, Eigen::VectorXd& upperLimitForX) {
        int variables_num = (static_cast<int>(ref_stamps_.size()) - 1) * 6;
        useLowerLimitForX.setConstant(variables_num, 1);
        useUpperLimitForX.setConstant(variables_num, 1);
        lowerLimitForX.resize(variables_num);
        upperLimitForX.resize(variables_num);

        for (int i = 0; i < variables_num; i++) {
            if (i == 0 || i == variables_num - 1) {
                // For the first point and last point, the unequal constraints are invalid
                useLowerLimitForX(i) = 0;
                useUpperLimitForX(i) = 0;
                lowerLimitForX(i) = 0.0;
                upperLimitForX(i) = 0.0;
            } else {
                useLowerLimitForX(i) = 1;
                useUpperLimitForX(i) = 1;
                lowerLimitForX(i) = single_lower_boundaries[i];
                upperLimitForX(i) = single_upper_boundaries[i];
            }
        }
    }

    /**
     * @brief solve quartic programming
     * @param {*}
     * @return {*}
     */
    bool OoqpOptimizationInterface::solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::VectorXd& b,
                    Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimitForX, Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimitForX, 
                    Eigen::VectorXd& lowerLimitForX, Eigen::VectorXd& upperLimitForX, std::vector<double>* optimized_values) {
        
        // // DEBUG
        // std::cout << "Q: " << Q << std::endl;
        // std::cout << "c: " << c << std::endl;
        // std::cout << "A: " << A << std::endl;
        // std::cout << "b: " << b << std::endl;
        // std::cout << "useLowerLimitForX: " << useLowerLimitForX << std::endl;
        // std::cout << "useUpperLimitForX: " << useUpperLimitForX << std::endl;
        // std::cout << "lowerLimitForX: " << lowerLimitForX << std::endl;
        // std::cout << "upperLimitForX: " << upperLimitForX << std::endl;
        // // END DEBUG

        Eigen::VectorXd x;
        int nx = Q.rows();
        x.setZero(nx);
        // Make copies of variables that are changed
        auto ccopy(c);
        auto Acopy(A);
        auto bcopy(b);

        Eigen::SparseMatrix<double, Eigen::RowMajor> Q_triangular = Q.triangularView<Eigen::Lower>();
        
        // Compress sparse Eigen matrices (refer to Eigen Sparse Matrix user manual)
        Q_triangular.makeCompressed();
        Acopy.makeCompressed();

        // Initialize new problem formulation.
        int my = bcopy.size();
        int mz = 0; // TODO: check this parameter, unequal constraint , set with 0
        int nnzQ = Q_triangular.nonZeros();
        // int nnzA = Acopy.nonZeros();
        int nnzA = Acopy.nonZeros();
        int nnzC = 0; // Unequal constraint , set with 0

        QpGenSparseMa27* qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);
        
        // Fill in problem data
        double* cp = &ccopy.coeffRef(0);
        std::vector<int> irowQ_vec, jcolQ_vec;
        Tools::calculateParam(Q_triangular, &irowQ_vec, &jcolQ_vec);
        int irowQ[irowQ_vec.size()], jcolQ[jcolQ_vec.size()];
        memcpy(irowQ, &irowQ_vec[0], irowQ_vec.size() * sizeof(irowQ_vec[0]));
        memcpy(jcolQ, &jcolQ_vec[0], jcolQ_vec.size() * sizeof(jcolQ_vec[0]));
        double* dQ = Q_triangular.valuePtr();

        double* xlow = &lowerLimitForX.coeffRef(0);
        char* ixlow = &useLowerLimitForX.coeffRef(0);
        double* xupp = &upperLimitForX.coeffRef(0);
        char* ixupp = &useUpperLimitForX.coeffRef(0);
        
        std::vector<int> irowA_vec, jcolA_vec;
        Tools::calculateParam(Acopy, &irowA_vec, &jcolA_vec);
        int irowA[irowA_vec.size()], jcolA[jcolA_vec.size()];
        memcpy(irowA, &irowA_vec[0], irowA_vec.size() * sizeof(irowA_vec[0]));
        memcpy(jcolA, &jcolA_vec[0], jcolA_vec.size() * sizeof(jcolA_vec[0]));
        double* dA = Acopy.valuePtr();
        double* bA = &bcopy.coeffRef(0);
        
        int krowC[] = {};
        int jcolC[] = {};
        double dC[] = {};
        double clow[] = {};
        char iclow[] = {};
        double cupp[] = {};
        char icupp[] = {};

        QpGenData *prob = (QpGenData *)qp->copyDataFromSparseTriple(cp, irowQ, nnzQ, jcolQ, dQ, xlow, ixlow, xupp, ixupp, irowA, nnzA, jcolA, dA, bA, krowC, nnzC, jcolC, dC, clow, iclow, cupp, icupp);
        
        // Create object to store problem variables
        QpGenVars* vars = (QpGenVars*)qp->makeVariables(prob);

        // // DEBUG
        // prob->print();
        // // END DEBUG

        // Create object to store problem residual data
        QpGenResiduals* resid = (QpGenResiduals*)qp->makeResiduals(prob);

        // Create solver object
        GondzioSolver* s = new GondzioSolver(qp, prob);

        // // DEBUG
        // s->monitorSelf();
        // // END DEBUG

        // Solve
        int status = s->solve(prob, vars, resid);

        if (status == TerminationCode::SUCCESSFUL_TERMINATION) {
            vars->x->copyIntoArray(&x.coeffRef(0));
            *optimized_values = std::vector<double>(x.data(), x.data() + x.rows() * x.cols());
        }

        delete s;
        delete resid;
        delete vars;
        delete prob;
        delete qp;

        return status == TerminationCode::SUCCESSFUL_TERMINATION;

    }

    SscOptimizer::SscOptimizer() {
        ssc_opt_itf_ = new OoqpOptimizationInterface();
    }
    SscOptimizer::~SscOptimizer() = default;

    /**
     * @brief Load data
     * @param start_constraint
     * @param end_constraint
     * @param driving_corridor
     */    
    void SscOptimizer::load(const EqualConstraint& start_constraint, const EqualConstraint& end_constraint, const DrivingCorridorWorldMetric& driving_corridor) {
        start_constraint_ = start_constraint;
        end_constraint_ = end_constraint;
        driving_corridor_ = driving_corridor;
    }

    /**
     * @brief Run optimizer to generate optimized variables 
     * @param {*}
     * @return {*}
     */
    bool SscOptimizer::runOnce(std::vector<double>* s, std::vector<double>* d, std::vector<double>* t) {
        // ~Stage I: calculate time stamps of split point (include start point and end point)
        std::vector<double> ref_stamps;
        for (int i = 0; i < static_cast<int>(driving_corridor_.cubes.size()); i++) {
            if (i == 0) {
                ref_stamps.emplace_back(driving_corridor_.cubes[i].cube.t_start_);
            }
            ref_stamps.emplace_back(driving_corridor_.cubes[i].cube.t_end_);
        }
        
        // ~Stage II: calculate unequal constraints for variables (except start point and end point)
        std::array<std::vector<double>, 4> unequal_constraints;
        generateUnequalConstraints(&unequal_constraints);

        // // DEBUG
        // for (int i = 0; i < 4; i++) {
        //     for (int j = 0; j < static_cast<int>(unequal_constraints[i].size()); j++) {
        //         printf("i: %d, j: %d, %lf\n", i, j, unequal_constraints[i][j]);
        //     }
        // }
        // // END DEBUG

        // ~Stage III: calculate equal constraints to ensure the continuity
        std::vector<std::vector<double>> equal_constraints;
        generateEqualConstraints(&equal_constraints);

        // ~Stage IV: optimization
        // TODO: add logic to judge optimization failed
        std::vector<double> optimized_s;
        std::vector<double> optimized_d;
        ssc_opt_itf_->load(ref_stamps, start_constraint_, end_constraint_, unequal_constraints, equal_constraints);
        bool optimization_res = ssc_opt_itf_->runOnce(&optimized_s, &optimized_d);

        *s = optimized_s;
        *d = optimized_d;
        *t = ref_stamps;

        return optimization_res;
    }

    /**
     * @brief Generate unequal constraints for intermediate variables 
     * @param unequal_constraints constraints generated 
     */    
    void SscOptimizer::generateUnequalConstraints(std::array<std::vector<double>, 4>* unequal_constraints) {
        // Initialize
        int variables_num = static_cast<int>(driving_corridor_.cubes.size()) * 6;
        std::array<std::vector<double>, 4> tmp_unequal_constraints = {};
        for (int i = 0; i < 4; i++) {
            tmp_unequal_constraints[i].resize(static_cast<int>(variables_num));
        } 

        // Calculate unequal constraints
        for (int i = 0; i < variables_num; i++) {
            if (i == 0 || i == variables_num - 1) {
                // Delete unequal constraints for start point and end point
                continue;
            }

            // if (i % 5 == 0) {
            //     // Points in intersection which have two constraint cubes 
            //     int next_corridor_index = i / 5;
            //     DrivingCubeWorldMetric current_cube = driving_corridor_.cubes[next_corridor_index - 1];
            //     DrivingCubeWorldMetric next_cube = driving_corridor_.cubes[next_corridor_index];
            //     tmp_unequal_constraints[0][i] = std::max(current_cube.cube.s_start_, next_cube.cube.s_start_); // s_low
            //     tmp_unequal_constraints[1][i] = std::min(current_cube.cube.s_end_, next_cube.cube.s_end_); // s_up
            //     tmp_unequal_constraints[2][i] = std::max(current_cube.cube.d_start_, next_cube.cube.d_start_); // d_start
            //     tmp_unequal_constraints[3][i] = std::min(current_cube.cube.d_end_, next_cube.cube.d_end_); // d_end
            // } else {
            //     // Normal points which only have one constraint cube
            //     int current_corridor_index = i / 5;
            //     DrivingCubeWorldMetric current_cube = driving_corridor_.cubes[current_corridor_index];
            //     tmp_unequal_constraints[0][i] = current_cube.cube.s_start_;
            //     tmp_unequal_constraints[1][i] = current_cube.cube.s_end_;
            //     tmp_unequal_constraints[2][i] = current_cube.cube.d_start_;
            //     tmp_unequal_constraints[3][i] = current_cube.cube.d_end_;
            // }

            int current_corridor_index = i / 6;
            DrivingCubeWorldMetric current_cube = driving_corridor_.cubes[current_corridor_index];
            tmp_unequal_constraints[0][i] = current_cube.cube.s_start_;
            tmp_unequal_constraints[1][i] = current_cube.cube.s_end_;
            tmp_unequal_constraints[2][i] = current_cube.cube.d_start_;
            tmp_unequal_constraints[3][i] = current_cube.cube.d_end_;

        }

        *unequal_constraints = tmp_unequal_constraints;
    }

    /**
     * @brief Generate equal constraints that were deployed to ensure the continuity in the intersection in each two cubes
     * @param equal_constraints equal constraints generated
     */  
    void SscOptimizer::generateEqualConstraints(std::vector<std::vector<double>>* equal_constraints) {
        // Initialize 
        std::vector<std::vector<double>> tmp_equal_constraints;
        int variables_num = static_cast<int>(driving_corridor_.cubes.size()) * 6;

        // Calculate equal constraints
        for (int i = 0; i < static_cast<int>(driving_corridor_.cubes.size()) - 1; i++) {
            // Calculate two related cubes and their time span
            int current_cube_index = i;
            int next_cube_index = i + 1;
            DrivingCubeWorldMetric current_cube = driving_corridor_.cubes[current_cube_index];
            DrivingCubeWorldMetric next_cube = driving_corridor_.cubes[next_cube_index];
            double current_cube_time_span = current_cube.cube.t_end_ - current_cube.cube.t_start_;
            double next_cube_time_span = next_cube.cube.t_end_ - next_cube.cube.t_start_;

            // Initialize equal constraints
            int current_cube_start_index = i * 6;
            int next_cube_start_index = (i + 1) * 6;
            std::vector<double> current_position_equal_constraints(variables_num, 0.0);
            std::vector<double> current_velocity_equal_constraints(variables_num, 0.0);
            std::vector<double> current_acceleration_equal_constraints(variables_num, 0.0);

            // Supple position constraints
            current_position_equal_constraints[current_cube_start_index + 5] = 1.0;
            current_position_equal_constraints[next_cube_start_index] = -1.0;

            // Supple velocity constraints 
            current_velocity_equal_constraints[current_cube_start_index + 4] = -5.0 / current_cube_time_span;
            current_velocity_equal_constraints[current_cube_start_index + 5] = 5.0 / current_cube_time_span;
            current_velocity_equal_constraints[next_cube_start_index] = -1.0 * -5.0 / next_cube_time_span;
            current_velocity_equal_constraints[next_cube_start_index + 1] = -1.0 * 5.0 / next_cube_time_span;

            // Supple acceleration constraints 
            current_acceleration_equal_constraints[current_cube_start_index + 3] = 20.0 / current_cube_time_span;
            current_acceleration_equal_constraints[current_cube_start_index + 4] = -40.0 / current_cube_time_span;
            current_acceleration_equal_constraints[current_cube_start_index + 5] = 20.0 / current_cube_time_span;
            current_acceleration_equal_constraints[next_cube_start_index] = -1.0 * 20.0 / next_cube_time_span;
            current_acceleration_equal_constraints[next_cube_start_index + 1] = -1.0 * -40.0 / next_cube_time_span;
            current_acceleration_equal_constraints[next_cube_start_index + 2] = -1.0 * 20.0 / next_cube_time_span;

            // Cache
            tmp_equal_constraints.emplace_back(current_position_equal_constraints);
            tmp_equal_constraints.emplace_back(current_velocity_equal_constraints);
            tmp_equal_constraints.emplace_back(current_acceleration_equal_constraints);

        }

        *equal_constraints = tmp_equal_constraints;
    } 

    BezierPiecewiseCurve::BezierPiecewiseCurve(const std::vector<double>& s, const std::vector<double>& d, std::vector<double>& ref_stamps) {
        // Check data
        assert(s.size() == d.size());
        assert((static_cast<int>(ref_stamps.size()) - 1) * 6 == static_cast<int>(s.size()));

        // Calculate segments number
        ref_stamps_ = ref_stamps;
        segment_num_ = static_cast<int>(ref_stamps.size()) - 1;

        // Calculate coefficient for both s dimension and d dimension
        s_coefficients_.resize(segment_num_);
        d_coefficients_.resize(segment_num_);

        for (int i = 0; i < segment_num_; i++) {
            s_coefficients_[i].resize(6);
            d_coefficients_[i].resize(6);
            
            // Supple coefficients
            int start_influenced_index = i * 6;
            s_coefficients_[i][0] = s[start_influenced_index];
            s_coefficients_[i][1] = s[start_influenced_index + 1];
            s_coefficients_[i][2] = s[start_influenced_index + 2];
            s_coefficients_[i][3] = s[start_influenced_index + 3];
            s_coefficients_[i][4] = s[start_influenced_index + 4];
            s_coefficients_[i][5] = s[start_influenced_index + 5];
            d_coefficients_[i][0] = d[start_influenced_index];
            d_coefficients_[i][1] = d[start_influenced_index + 1];
            d_coefficients_[i][2] = d[start_influenced_index + 2];
            d_coefficients_[i][3] = d[start_influenced_index + 3];
            d_coefficients_[i][4] = d[start_influenced_index + 4];
            d_coefficients_[i][5] = d[start_influenced_index + 5];
        }

    }
    BezierPiecewiseCurve::~BezierPiecewiseCurve() = default;

    /**
     * @brief Calculate curve
     * @param {*}
     * @return {*}
     */    
    std::vector<Point3f> BezierPiecewiseCurve::generateTraj(double sample_gap) {
        // Initialize
        std::vector<Point3f> traj;

        // Calculate point for each segment
        // Note that for each segment, the sample gap is the same, which means the sample points' number is different according to the segment's time span
        for (int segment_index = 0; segment_index < segment_num_; segment_index++) {
            double time_span = ref_stamps_[segment_index + 1] - ref_stamps_[segment_index];
            int sample_num = static_cast<int>(time_span / sample_gap);
            // Calculate seeds in this segment
            std::vector<double> segment_seeds;
            if (segment_index == segment_num_ - 1) {
                segment_seeds = Tools::linspace(0.0, 1.0, sample_num);
            } else {
                segment_seeds = Tools::linspace(0.0, 1.0, sample_num, false);
            }

            // For each seed, generate a point
            for (const auto& current_seed : segment_seeds) {
                double time_stamp = ref_stamps_[segment_index] + (time_span * current_seed);
                traj.emplace_back(generatePoint(segment_index, current_seed, time_stamp));
            }
        }

        return traj;
        
    }

    /**
     * @brief Calculate single point 
     * @param {*}
     * @return {*}
     */    
    Point3f BezierPiecewiseCurve::generatePoint(int segment_index, double remain, double time_stamp) {
        // Calculate s and d value
        double s_value = s_coefficients_[segment_index][0] * pow(1.0 - remain, 5) + 5.0 * s_coefficients_[segment_index][1] * remain * pow(1.0 - remain, 4) + 10.0 * s_coefficients_[segment_index][2] * pow(remain, 2) * pow(1.0 - remain, 3) + 10.0 * s_coefficients_[segment_index][3] * pow(remain, 3) * pow(1.0 - remain, 2) + 5.0 * s_coefficients_[segment_index][4] * pow(remain, 4) * (1.0 - remain) + s_coefficients_[segment_index][5] * pow(remain, 5);
        double d_value = d_coefficients_[segment_index][0] * pow(1.0 - remain, 5) + 5.0 * d_coefficients_[segment_index][1] * remain * pow(1.0 - remain, 4) + 10.0 * d_coefficients_[segment_index][2] * pow(remain, 2) * pow(1.0 - remain, 3) + 10.0 * d_coefficients_[segment_index][3] * pow(remain, 3) * pow(1.0 - remain, 2) + 5.0 * d_coefficients_[segment_index][4] * pow(remain, 4) * (1.0 - remain) + d_coefficients_[segment_index][5] * pow(remain, 5);

        return Point3f(s_value, d_value, time_stamp);
    }

    SscTrajectoryPlanningCore::SscTrajectoryPlanningCore() = default;
    SscTrajectoryPlanningCore::~SscTrajectoryPlanningCore() = default;

    // Load data
    void SscTrajectoryPlanningCore::load(const Vehicle& cur_vehicle_state, const Lane& reference_lane, const std::vector<Vehicle>& ego_traj, const std::unordered_map<int, std::vector<Vehicle>>& sur_laned_veh_trajs, const std::vector<DecisionMaking::Obstacle>& sur_unlaned_obs) {
        current_vehicle_state_ = cur_vehicle_state;
        reference_lane_ = reference_lane;
        ego_traj_ = ego_traj;
        sur_laned_veh_trajs_ = sur_laned_veh_trajs;
        sur_unlaned_obs_ = Tools::filtUnlanedObstaclesForSimulation(sur_unlaned_obs, cur_vehicle_state.state_.position_);
    }

    // Generate trajectory
    void SscTrajectoryPlanningCore::runOnce(bool* result, std::vector<Point3f>* trajectory) {
        // ~Stage I: construct bridge and transform information
        bridge_itf_ = new BpTpBridge(reference_lane_);
        FsVehicle current_vehicle_state_fs = bridge_itf_->getFsVehicle(current_vehicle_state_);
        std::vector<FsVehicle> ego_traj_fs = bridge_itf_->getEgoFrenetTrajectory(ego_traj_);
        std::unordered_map<int, std::vector<FsVehicle>> sur_laned_trajs_fs = bridge_itf_->getSurFrenetTrajectories(sur_laned_veh_trajs_);
        std::unordered_map<int, std::vector<FsVehicle>> sur_unlaned_trajs_fs = bridge_itf_->getUnlanedSurFrenetTrajectories(sur_unlaned_obs_);

        // // DEBUG 
        // for (auto& veh_fs : ego_traj_fs) {
        //     veh_fs.fs_.print();
        // }
        // // END DEBUG

        // ~Stage II: contruct traj planning 3d grid map and generate semantic cubes
        SscPlanning3DMap::Config config;
        ssc_3d_map_itf_ = new SscPlanning3DMap(config);
        ssc_3d_map_itf_->load(current_vehicle_state_fs.fs_);
        DrivingCorridorWorldMetric driving_corridor;
        bool is_map_constructed_success = ssc_3d_map_itf_->runOnce(ego_traj_fs, sur_laned_trajs_fs, sur_unlaned_trajs_fs, &driving_corridor);
        if (!is_map_constructed_success) {
            printf("[SscTrajectoryPlanningCore] ssc planning 3d grid map constructed failed.\n");
            *result = false;
            return;
        }

        // // DEBUG
        // driving_corridor.print();
        // // END DEBUG

        // ~Stage III: determine constraints conditions and do optimization
        EqualConstraint start_constraints, end_constraints;
        start_constraints.load(current_vehicle_state_fs);
        end_constraints.load(ego_traj_fs.back());
        ssc_opt_itf_ = new SscOptimizer();
        ssc_opt_itf_->load(start_constraints, end_constraints, driving_corridor);
        std::vector<double> s, d, t;
        // TODO: add logic to handle the situation where the optimization is failed
        bool optimization_res = ssc_opt_itf_->runOnce(&s, &d, &t);
        if (!optimization_res || (s.size() != d.size())) {
            printf("[SscTrajectoryPlanningCore] ssc optimization failed.\n");
            *result = false;
            return;
        }

        // // DEBUG
        // for (const auto& it_s : s) {
        //     printf("%lf, ", it_s);
        // }
        // printf("--------------------\n");
        // for (const auto& it_d : d) {
        //     printf("%lf, ", it_d);
        // }
        // printf("--------------------\n");
        // for (const auto& it_t : t) {
        //     printf("%lf, ", it_t);
        // }
        // printf("--------------------\n");
        // // END DEBUG
        
        // ~Stage IV: calculate piecewise bezier curve in frenet frame
        bezier_curve_traj_itf_ = new BezierPiecewiseCurve(s, d, t);
        std::vector<Point3f> traj_fs = bezier_curve_traj_itf_->generateTraj(0.02);

        // // DEBUG
        // std::vector<double> interpolated_s, interpolated_d, interpolated_t;
        // for (const auto& point_fs : traj_fs) {
        //     interpolated_s.emplace_back(point_fs.x_);
        //     interpolated_d.emplace_back(point_fs.y_);
        //     interpolated_t.emplace_back(point_fs.z_);
        // }
        // for (const auto& s : interpolated_s) {
        //     printf("%lf, ", s);
        // }
        // printf("--------------------\n");
        // for (const auto& d : interpolated_d) {
        //     printf("%lf, ", d);
        // }
        // printf("--------------------\n");
        // for (const auto& t : interpolated_t) {
        //     printf("%lf, ", t);
        // }
        // printf("--------------------\n");

        // // END DEBUG

        // ~Stage V: transform the trajectory from frenet to world
        // TODO: fatal error, add logic to improve coordination transformation precision
        std::vector<Point3f> traj = bridge_itf_->getTrajFromTrajFs(traj_fs);

        // // DEBUG
        // std::vector<double> interpolated_s, interpolated_d, interpolated_t;
        // for (const auto& point_fs : traj) {
        //     interpolated_s.emplace_back(point_fs.x_);
        //     interpolated_d.emplace_back(point_fs.y_);
        //     interpolated_t.emplace_back(point_fs.z_);
        // }
        // for (const auto& s : interpolated_s) {
        //     printf("%lf, ", s);
        // }
        // printf("--------------------\n");
        // for (const auto& d : interpolated_d) {
        //     printf("%lf, ", d);
        // }
        // printf("--------------------\n");
        // for (const auto& t : interpolated_t) {
        //     printf("%lf, ", t);
        // }
        // printf("--------------------\n");

        // // END DEBUG

        // // Test 
        // // Only transform the control points position
        // std::vector<Point2f> control_points_2d_fs;
        // assert(s.size() == d.size());
        // for (int i = 0; i < static_cast<int>(s.size()); i++) {
        //     control_points_2d_fs.emplace_back(Point2f(s[i], d[i]));
        // }
        // std::vector<Point2f> control_points_2d = bridge_itf_->get2dTrajFrom2dTrajFs(control_points_2d_fs);
        // std::vector<double> s_world, d_world;
        // for (const auto& point : control_points_2d) {
        //     s_world.emplace_back(point.x_);
        //     d_world.emplace_back(point.y_);
        // }
        // bezier_curve_traj_itf_ = new BezierPiecewiseCurve(s_world, d_world, t);
        // std::vector<Point3f> traj = bezier_curve_traj_itf_->generateTraj(0.01);
        // // End test

        // // DEBUG
        // std::vector<double> interpolated_s, interpolated_d, interpolated_t;
        // for (const auto& point_fs : traj) {
        //     interpolated_s.emplace_back(point_fs.x_);
        //     interpolated_d.emplace_back(point_fs.y_);
        //     interpolated_t.emplace_back(point_fs.z_);
        // }
        // for (const auto& s : interpolated_s) {
        //     printf("%lf, ", s);
        // }
        // printf("--------------------\n");
        // for (const auto& d : interpolated_d) {
        //     printf("%lf, ", d);
        // }
        // printf("--------------------\n");
        // for (const auto& t : interpolated_t) {
        //     printf("%lf, ", t);
        // }
        // printf("--------------------\n");

        // // END DEBUG

        *result = true;
        *trajectory = traj;
    
    }

} // End of namespace SscPlanner