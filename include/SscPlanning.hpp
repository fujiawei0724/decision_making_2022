/*
 * @Author: fujiawei0724
 * @Date: 2021-11-22 16:30:19
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-12-09 20:22:30
 * @Descripttion: Ssc trajectory planning.
 */


#pragma once

#include "Const.hpp"
#include "VehicleState.hpp"

namespace SscPlanner {

using namespace Common;

struct DrivingCube {
    std::vector<Point3i> seeds;
    SemanticCube<int> cube;
};

struct DrivingCubeWorldMetric {
    std::vector<Point3f> seeds;
    SemanticCube<double> cube;
};

struct DrivingCorridor {
    bool is_valid;
    std::vector<DrivingCube> cubes;

    // DEBUG
    void print() {
        for (int i = 0; i < static_cast<int>(cubes.size()); i++) {
            printf("Cube index: %d\n", i);
            cubes[i].cube.print();
        }
    }
};

struct DrivingCorridorWorldMetric {
    bool is_valid;
    std::vector<DrivingCubeWorldMetric> cubes;

    // DEBUG
    void print() {
        for (int i = 0; i < static_cast<int>(cubes.size()); i++) {
            printf("Cube index: %d\n", i);
            cubes[i].cube.print();
        }
    }
};

// Ssc map
class SscPlanning3DMap {
 public:
    using GridMap3D = GridMapND<uint8_t, 3>;
    struct Config {
        std::array<int, 3> map_size = {{1000, 100, 81}};
        std::array<double, 3> map_resolution = {{0.25, 0.2, 0.1}};
        std::array<std::string, 3> axis_name = {{"s", "d", "t"}};

        double s_back_len = 7.0;
        double MaxLongitudinalVel = 50.0;
        double MinLongitudinalVel = 0.0;
        double MaxLongitudinalAcc = 3.0;
        double MaxLongitudinalDecel = -2.0;
        double MaxLateralVel = 3.0;
        double MaxLateralAcc = 2.5;

        int MaxNumOfGridAlongTime = 2;

        // TODO: check this logic, the time inflate steps number may be too small to construct continuous cubes with large overlap to ensure the feasibility of optimization space
        std::array<int, 6> inflate_steps = {{20, 5, 10, 10, 1, 1}};
    };

    SscPlanning3DMap();
    SscPlanning3DMap(const Config& config);
    ~SscPlanning3DMap();
    
    /**
     * @brief Load data, set initial fs and origin
     * @param {*}
     * @return {*}
     */    
    void load(const FrenetState& initial_fs);

    /**
     * @brief Run once to generate the semantic cubes sequence
     * @param {*}
     * @return {*}
     */
    bool runOnce(const std::vector<FsVehicle>& ego_traj, const std::unordered_map<int, std::vector<FsVehicle>>& surround_laned_trajs, const std::unordered_map<int, std::vector<FsVehicle>>& surround_unlaned_obs_trajs, DrivingCorridorWorldMetric* semantic_cubes_sequence);  

    /**
     * @brief Get final global metric cube list
     * @param driving_corridor description of semantic cubes in coord index
     * @param semantic_cubes final cubes list
     */    
    void getFinalGlobalMetricCubesList(const DrivingCorridor& driving_corridor, DrivingCorridorWorldMetric* driving_corridor_world_metric);

    /**
     * @brief Transform cube from coordination to world metric
     * @param {*}
     * @return {*}
     */    
    void transformCoordCubeToWorldMetric(const DrivingCube& driving_cube, DrivingCubeWorldMetric* cube_world_metric);

    /**
     * @brief Transform coord semantic cube to semantic cube
     * @param coord_semantic_cube semantic cube in the formation of coordinate
     * @param semantic_cube semantic cube in the formation of value
     */    
    void transformCoordSemCubeToSemCube(const SemanticCube<int>& coord_semantic_cube, SemanticCube<double>* semantic_cube);

    /**
     * @brief Transform coord point3i to world metric point3f
     * @param {*}
     * @return {*}
     */
    void transformCoordToWorldMetric(const Point3i& coord_point, Point3f* world_point);

    /**
     * @brief Construct corridors using initial trajectory 
     * @param ego_traj ego vehicle's scatter trajectory state
     * @param corridor generated corridor 
     * @return is success
     */    
    bool generateCorridor(const std::vector<FsVehicle>& ego_traj, DrivingCorridor* corridor);

    /**
     * @brief Judge whether semantic cube contains seed
     * @param cube semantic cube need to judge
     * @param seed trajectory seed need to judge
     * @return result
     */    
    bool checkIfCubeContainsSeed(const SemanticCube<int>& cube, const Point3i& seed);



    /**
     * @brief Inflate the cube in 3D grid map until reached to the occupied space
     * @param init_seman_cube the initial scale of semantic cube
     */    
    void inflateCube(SemanticCube<int>* cube);

    /**
     * @brief Judge whether the semantic cube is free 
     * @param semantic_coord_cube cube needs to judge
     * @return is free
     */    
    bool checkIfSemanticCubeIsFree(const SemanticCube<int>& semantic_coord_cube);

    /**
     * @brief Inflate cube in x positive direction
     * @param n_step max number of movement step
     * @param cube cube needs to be inflated
     * @return is finished
     */    
    bool inflateCubeOnXPosAxis(const int& n_step, SemanticCube<int>* cube);



    /**
     * @brief Inflate cube in x negative direction
     * @param n_step max number of movement step
     * @param cube cube needs to be inflated
     * @return is finished
     */    
    bool inflateCubeOnXNegAxis(const int& n_step, SemanticCube<int>* cube);

    /**
     * @brief Inflate cube in Y positive direction
     * @param n_step max number of movement step
     * @param cube cube needs to be inflated
     * @return is finished
     */    
    bool inflateCubeOnYPosAxis(const int& n_step, SemanticCube<int>* cube);

    /**
     * @brief Inflate cube in Y negative direction
     * @param n_step max number of movement step
     * @param cube cube needs to be inflated
     * @return is finished
     */    
    bool inflateCubeOnYNegAxis(const int& n_step, SemanticCube<int>* cube);

    /**
     * @brief Inflate cube in Z positive direction
     * @param n_step max number of movement step
     * @param cube cube needs to be inflated
     * @return is finished
     */    
    bool inflateCubeOnZPosAxis(const int& n_step, SemanticCube<int>* cube);


    /**
     * @brief Check if plane is free on X axis
     * @param x coord in x
     * @param cube cube needs to be judge
     * @return is free
     */    
    bool checkIfPlaneIsFreeOnXAxis(const SemanticCube<int>& cube, const int& x) const;

    /**
     * @brief Check if plane is free on Y axis
     * @param y coord in y
     * @param cube cube needs to be judge
     * @return is free
     */     
    bool checkIfPlaneIsFreeOnYAxis(const SemanticCube<int>& cube, const int& y) const;

    /**
     * @brief Check if plane is free on Z axis
     * @param z coord in z
     * @param cube cube needs to be judge
     * @return is free
     */     
    bool checkIfPlaneIsFreeOnZAxis(const SemanticCube<int>& cube, const int& z) const;
    

    /**
     * @brief Construct seeds information using ego trajectory information
     * @param ego_traj ego vehicle's trajectory in frenet coordination
     * @return is success
     */
    bool constructSeeds(const std::vector<FsVehicle>& ego_traj, std::vector<Point3i>* seeds);

    // Construct 3D map using dynamic obstacles and static obstacles information
    void contruct3DMap(const std::unordered_map<int, std::vector<FsVehicle>>& surround_trajs);

    // Fill the dynamic obstacles information
    void fillDynamicObstacles(const std::unordered_map<int, std::vector<FsVehicle>>& surround_trajs);

    void fillSingleDynamicObstacle(const std::vector<FsVehicle>& fs_traj);

    // Fill the static obstacles information
    // Note that in the first version, the static obstacles are not considered here, this is just a empty interface
    // TODO: add the static obstacles' occupied grid here
    void fillStaticObstacles();


    GridMap3D* p_3d_grid_{nullptr};
    Config config_;

    FrenetState initial_fs_;


};

// Optimization interface, based on CGAL
class SscOptimizationInterface {
 public:
    using ET = CGAL::MP_Float;
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
    
    SscOptimizationInterface();
    ~SscOptimizationInterface();

    /**
     * @brief load data
     * @param ref_stamps time stamps of the point in in the intersection of two cubes
     * @param start_constraints start points' constraints
     * @param end_constraints end points' constraints
     * @param unequal_constraints position limit of each point
     * @param equal_constraints ensure the continuity of the connections between each two cubes
     */    
    void load(const std::vector<double>& ref_stamps, const EqualConstraint& start_constraints, const EqualConstraint& end_constraints, std::array<std::vector<double>, 4>& unequal_constraints, std::vector<std::vector<double>>& equal_constraints);

    /**
     * @brief Run optimization
     * @param {*}
     * @return {*}
     */    
    void runOnce(std::vector<double>* optimized_s, std::vector<double>* optimized_d);

    /**
     * @brief Optimize in single dimension
     * @param {*}
     */
    void optimizeSingleDim(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries, std::string dimension_name);
    
    /**
     * @brief Calculate boundaries for intermediate points
     * @param {*}
     */    
    void calculateBoundariesForIntermediatePoints(const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries, bool** fl, double** l, bool** fu, double** u);

    /**
     * @brief Calculate equal constraints, note that position constraints in the connection don't need to be considered
     * @param {*}
     */
    void calculateAbMatrix(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, const std::vector<std::vector<double>>& equal_constraints, std::vector<double*>* A, double** b);

    /**
     * @brief Calculate the matrices related to objective function, for both s and d dimensions, D and c has the same value
     * @param D 
     * @param c
     */
    void calculateDcMatrix(std::vector<double*>* D, double** c);

    std::vector<double> ref_stamps_;
    EqualConstraint start_constraints_;
    EqualConstraint end_constraints_;
    std::array<std::vector<double>, 4> unequal_constraints_;
    std::vector<std::vector<double>> equal_constraints_;

    std::unordered_map<std::string, std::vector<double>> optimized_data_;

    
};

// Optimization trajectory parameters 
class SscOptimizer {
 public:
    SscOptimizer();
    ~SscOptimizer();

    /**
     * @brief Load data
     * @param start_constraint
     * @param end_constraint
     * @param driving_corridor
     */    
    void load(const EqualConstraint& start_constraint, const EqualConstraint& end_constraint, const DrivingCorridorWorldMetric& driving_corridor);

    /**
     * @brief Run optimizer to generate optimized variables 
     * @param {*}
     * @return {*}
     */
    void runOnce(std::vector<double>* s, std::vector<double>* d, std::vector<double>* t);

    /**
     * @brief Generate unequal constraints for intermediate variables 
     * @param unequal_constraints constraints generated 
     */    
    void generateUnequalConstraints(std::array<std::vector<double>, 4>* unequal_constraints);

    /**
     * @brief Generate equal constraints that were deployed to ensure the continuity in the intersection in each two cubes
     * @param equal_constraints equal constraints generated
     */  
    void generateEqualConstraints(std::vector<std::vector<double>>* equal_constraints);

    SscOptimizationInterface* ssc_opt_itf_{nullptr};

    EqualConstraint start_constraint_;
    EqualConstraint end_constraint_;
    DrivingCorridorWorldMetric driving_corridor_;
};

// Generate interpolated curves
class BezierPiecewiseCurve {
 public:
    BezierPiecewiseCurve(const std::vector<double>& s, const std::vector<double>& d, std::vector<double>& ref_stamps);
    ~BezierPiecewiseCurve();

    /**
     * @brief Calculate curve
     * @param {*}
     * @return {*}
     */    
    std::vector<Point3f> generateTraj(double sample_gap = 0.01);

    /**
     * @brief Calculate single point 
     * @param {*}
     * @return {*}
     */    
    Point3f generatePoint(int segment_index, double remain, double time_stamp);

    int segment_num_;
    std::vector<double> ref_stamps_;

    std::vector<std::vector<double>> s_coefficients_;
    std::vector<std::vector<double>> d_coefficients_;
};

// Trajectory planning core
class SscTrajectoryPlanningCore {
 public:
    SscTrajectoryPlanningCore();
    ~SscTrajectoryPlanningCore();

    // Load data
    void load(const Vehicle& cur_vehicle_state, const Lane& reference_lane, const std::vector<Vehicle>& ego_traj, const std::unordered_map<int, std::vector<Vehicle>>& sur_laned_veh_trajs, const std::vector<DecisionMaking::Obstacle>& sur_unlaned_obs);

    // Generate trajectory
    void runOnce(bool* result, std::vector<Point3f>* trajectory);

    BezierPiecewiseCurve* bezier_curve_traj_itf_{nullptr};
    SscPlanning3DMap* ssc_3d_map_itf_{nullptr};
    BpTpBridge* bridge_itf_{nullptr};
    SscOptimizer* ssc_opt_itf_{nullptr};


    Vehicle current_vehicle_state_;
    Lane reference_lane_;
    std::vector<Vehicle> ego_traj_;
    std::unordered_map<int, std::vector<Vehicle>> sur_laned_veh_trajs_;
    std::vector<DecisionMaking::Obstacle> sur_unlaned_obs_;
};

} // End of namespace ssc planner