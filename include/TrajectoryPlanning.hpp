/*
 * @Author: fujiawei0724
 * @Date: 2021-11-04 15:05:54
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2021-11-08 11:57:43
 * @Descripttion: The components for trajectory planning. 
 */

#pragma once

#include "Const.hpp"
#include "VehicleState.hpp"

namespace TrajectoryPlanner {


// Equal constraints related to start point and end point
class EqualConstraint {
public:
    // Constructor
    EqualConstraint() = default;
    EqualConstraint(double s, double d_s, double dd_s, double d, double d_d, double dd_d) {
        s_ = s;
        d_s_ = d_s;
        dd_s_ = dd_s;
        d_ = d;
        d_d_ = d_d;
        dd_d_ = dd_d;
    }
    // Destructor
    ~EqualConstraint() = default;

    // s means the longitudinal dimension, d means the latitudinal dimension, d_ means the first derivative, dd_ denotes the second derivative
    double s_;
    double d_s_;
    double dd_s_;
    double d_;
    double d_d_;
    double dd_d_;
};

// Unequal constraints related to intermediate points' position
// Note that the unequal constraints are only related to the position
class UnequalConstraint {
public:
    // Constructor
    UnequalConstraint() = default;
    UnequalConstraint(double s_low, double s_up, double d_low, double d_up) {
        s_low_ = s_low;
        s_up_ = s_up;
        d_low_ = d_low;
        d_up_ = d_up;
    }
    // Destructor 
    ~UnequalConstraint() = default;
    
    double s_low_;
    double s_up_;
    double d_low_;
    double d_up_;
};


// The semantic cube to constrain the position of trajectory interpolation points
class SemanticCube {
public:
    // Constructor
    SemanticCube() = default;
    SemanticCube(double s_start, double s_end, double d_start, double d_end, double t_start, double t_end) {
        s_start_ = s_start;
        s_end_ = s_end;
        d_start_ = d_start;
        d_end_ = d_end;
        t_start_ = t_start;
        t_end_ = t_end;
    }
    // Destructor
    ~SemanticCube() = default;

    double s_start_;
    double s_end_;
    double d_start_;
    double d_end_;
    double t_start_;
    double t_end_;

};




} // End of namespace trajectory planner

