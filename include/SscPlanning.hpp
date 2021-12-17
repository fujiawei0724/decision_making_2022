/*
 * @Author: fujiawei0724
 * @Date: 2021-11-22 16:30:19
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-12-17 21:16:32
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

// Optimization interface, based on OOQP
class OoqpOptimizationInterface {
 public:
    OoqpOptimizationInterface() = default;
    ~OoqpOptimizationInterface() = default;

    /**
     * @brief load data
     * @param ref_stamps time stamps of the point in in the intersection of two cubes
     * @param start_constraints start points' constraints
     * @param end_constraints end points' constraints
     * @param unequal_constraints position limit of each point
     * @param equal_constraints ensure the continuity of the connections between each two cubes
     */    
    void load(const std::vector<double>& ref_stamps, const EqualConstraint& start_constraints, const EqualConstraint& end_constraints, std::array<std::vector<double>, 4>& unequal_constraints, std::vector<std::vector<double>>& equal_constraints) {
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
    bool runOnce(std::vector<double>* optimized_s, std::vector<double>* optimized_d) {
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
    void optimizeSingleDim(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries, std::string dimension_name) {
        // ~Stage I: calculate objective function Q and c
        Eigen::SparseMatrix<double, Eigen::RowMajor> Q;
        Eigen::VectorXd c;
        calculateQcMatrix(Q, c);

        // ~Stage II: calculate equal constraints, includes start point constraints, end point constraints and continuity constraints
        Eigen::SparseMatrix<double, Eigen::RowMajor> A;
        Eigen::VectorXd b;
        calculateAbMatrix(single_start_constraints, single_end_constraints, equal_constraints_, A, b);

        // ~Stage III: calculate low and up boundaries for intermediate points
        Eigen::Matrix<char, Eigen::Dynamic, 1> useLowerLimitForX;
        Eigen::Matrix<char, Eigen::Dynamic, 1> useUpperLimitForX;
        Eigen::VectorXd lowerLimitForX;
        Eigen::VectorXd upperLimitForX;
        calculateBoundariesForIntermediatePoints(single_lower_boundaries, single_upper_boundaries, useLowerLimitForX, useUpperLimitForX, lowerLimitForX, upperLimitForX);

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
    void calculateQcMatrix(Eigen::SparseMatrix<double, Eigen::RowMajor>& Q, Eigen::VectorXd& c) {
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
        Q = Q_matrix.sparseView();
        c.resize(variables_num);
        c.setZero();
    }

    /**
     * @brief Calculate equal constraints, note that position constraints in the connection don't need to be considered
     * @param {*}
     */
    void calculateAbMatrix(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, const std::vector<std::vector<double>>& equal_constraints, Eigen::SparseMatrix<double, Eigen::RowMajor>& A, Eigen::VectorXd& b) {
                
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
    }

    /**
     * @brief Calculate boundaries for intermediate points
     * @param {*}
     */    
    void calculateBoundariesForIntermediatePoints(const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries, Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimitForX, Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimitForX, Eigen::VectorXd& lowerLimitForX, Eigen::VectorXd& upperLimitForX) {
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
    bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::VectorXd& b,
                    Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimitForX, Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimitForX, 
                    Eigen::VectorXd& lowerLimitForX, Eigen::VectorXd& upperLimitForX, std::vector<double>* optimized_values) {
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
        int mz = 0; // TOCO: check this parameter, unequal constraint , set with 0
        int nnzQ = Q_triangular.nonZeros();
        int nnzA = Acopy.nonZeros();
        int nnzC = 0; // Unequal constraint , set with 0

        QpGenSparseMa27* qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);
        // Fill in problem data.
        double* cp = &ccopy.coeffRef(0);
        int* krowQ = Q_triangular.outerIndexPtr();
        int* jcolQ = Q_triangular.innerIndexPtr();
        double* dQ = Q_triangular.valuePtr();
        double* xlow = &lowerLimitForX.coeffRef(0);
        char* ixlow = &useLowerLimitForX.coeffRef(0);
        double* xupp = &upperLimitForX.coeffRef(0);
        char* ixupp = &useUpperLimitForX.coeffRef(0);
        int* krowA = Acopy.outerIndexPtr();
        int* jcolA = Acopy.innerIndexPtr();
        double* dA = Acopy.valuePtr();
        double* bA = &bcopy.coeffRef(0);
        int* krowC = 0;
        int* jcolC = 0;
        double* dC = 0;
        double* clow = 0;
        char* iclow = 0;
        double* cupp = 0;
        char* icupp = 0;

        QpGenData* prob = (QpGenData*)qp->makeData(cp, krowQ, jcolQ, dQ, xlow, ixlow, xupp, ixupp, krowA, jcolA, dA, bA, krowC, jcolC, dC, clow, iclow, cupp, icupp);
        
        // Create object to store problem variables
        QpGenVars* vars = (QpGenVars*)qp->makeVariables(prob);

        // Create object to store problem residual data
        QpGenResiduals* resid = (QpGenResiduals*)qp->makeResiduals(prob);

        // Create solver object
        GondzioSolver* s = new GondzioSolver(qp, prob);

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

    std::vector<double> ref_stamps_;
    EqualConstraint start_constraints_;
    EqualConstraint end_constraints_;
    std::array<std::vector<double>, 4> unequal_constraints_;
    std::vector<std::vector<double>> equal_constraints_;

    std::unordered_map<std::string, std::vector<double>> optimized_data_;
    std::unordered_map<std::string, bool> optimization_res_;
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