//
// Created by zzhou on 16.01.18.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <vector>
#include <cmath>
#include <map>

#include "map.h"
#include "structure.h"


class Vehicle {
public:
    Vehicle();

    void load_map_info(const string &file_path);

    vector<vector<double>> update(vector<double> ego_car_data, vector<vector<double>> sensor_fusion,
                                  vector<double> previous_path_x, vector<double> previous_path_y,
                                  double end_path_s, double end_path_d);

    vector<string> get_successor_states();

    // trajectory generation
    Trajectory generate_state_traj(const string &state,
                                   const map<int, Vehicle> &preds,
                                   const vector<double> &previous_path_x,
                                   const vector<double> &previous_path_y);

    vector<vector<double>> generate_trajectory(double v_desired, vector<vector<double>> waypoints,
                                                   vector<double> previous_path_x, vector<double> previous_path_y);
    vector<vector<double>> get_waypoint(vector<double> param);

    // FSM
    vector<double> get_KL_param(const map<int, Vehicle>& preds);
    vector<double> get_PLCL_param(const map<int, Vehicle>& preds);
    vector<double> get_PLCR_param(const map<int, Vehicle>& preds);
    vector<double> get_LCL_param(const map<int, Vehicle>& preds);
    vector<double> get_LCR_param(const map<int, Vehicle>& preds);
    vector<double> get_kinematics(const map<int, Vehicle> &preds, int lane_id);

    // cost functions
    double cal_total_cost(const Trajectory& traj, const map<int, Vehicle>& preds, const vector<vector<double>>& sensor_fusion);
    double cal_cost_not_in_center_lane(double d_target);
    double cal_cost_best_target_speed(double v_target);
    double cal_cost_lane_change(int curr_lane_id, int target_lane_id);
    double cal_cost_accumulated_d(const vector<vector<double>> &traj);
    double cal_cost_collision(int curr_lane_id, int target_lane_id, const map<int, Vehicle>& preds);
    double cal_cost_cross_two_lane(int lane_id_curr, double d_curr, int lane_id_target, double d_target);

    // helper functions
    map<int, Vehicle> predict_other_vehicles(const vector<vector<double>> &sensor_fusion, double duration);
    bool get_vehicle_ahead(const map<int, Vehicle> &preds, int lane, Vehicle &r_vehicle);
    bool get_vehicle_neighbor_lane(const map<int, Vehicle> &preds, int neighbor_lane_id, Vehicle &r_vehicle);

    // helper functions
    int get_lane_id();
    double deg2rad(double x) { return x * M_PI / 180; }
    double mph2ms(double v_mph){return v_mph/2.24; }
    double ms2mph(double v_ms){return v_ms*2.24; }
    double sigmoid(double x);

    Map map_info;
    CarStatus status;
    double new_planning_time;

};


#endif //PATH_PLANNING_VEHICLE_H
