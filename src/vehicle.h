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

    vector<string> successor_states();

    vector<vector<double>> generate_trajectory(double v_desired, vector<vector<double>> waypoints,
                                                   vector<double> previous_path_x, vector<double> previous_path_y);

    vector<vector<double>>
    generate_KL_trajectory(const map<int, Vehicle>& preds, vector<double> previous_path_x, vector<double> previous_path_y);

    vector<vector<double>> get_waypoint(vector<double> param);

    vector<double> get_KL_param(const map<int, Vehicle>& preds);
    vector<double> get_LCL_param(const map<int, Vehicle>& preds);
    vector<double> get_LCR_param(const map<int, Vehicle>& preds);

    vector<double> get_kinematics(const map<int, Vehicle> &preds, int lane_id);

    map<int, Vehicle> predict_other_vehicles(const vector<vector<double>> &sensor_fusion, double duration);
    bool get_vehicle_ahead(const map<int, Vehicle> &preds, int lane, Vehicle &r_vehicle);

    // helper functions
    int get_lane_id();
    double deg2rad(double x) { return x * M_PI / 180; }
    double mph2ms(double v_mph){return v_mph/2.24; }
    double ms2mph(double v_ms){return v_ms*2.24; }

    Map map_info;
    CarStatus status;

};


#endif //PATH_PLANNING_VEHICLE_H
