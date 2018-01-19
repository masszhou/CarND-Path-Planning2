//
// Created by zzhou on 16.01.18.
//

#ifndef PATH_PLANNING_STRUCTURE_H
#define PATH_PLANNING_STRUCTURE_H

#include <string>

using namespace std;

struct CarStatus{
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double v_yaw;
    double v_yaw_desired;
    double v_s;
    double v_d;
    double a_yaw;
    double a_s;
    double a_d;
    string state;
};

struct Trajectory{
    vector<double> x_vec;
    vector<double> y_vec;
    vector<double> s_vec;
    vector<double> d_vec;
    double s_target;
    double d_target;
    double v_feasible; // yaw direction
    double v_target;
};

#endif //PATH_PLANNING_STRUCTURE_H
