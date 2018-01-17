//
// Created by zzhou on 16.01.18.
//

#include "vehicle.h"

#include <iostream>
#include "spline.h"

#include "constants.h"

Vehicle::Vehicle() {
    this->status.state = "KL";
    this->status.v_yaw_desired = 5;
}

void Vehicle::load_map_info(const string &file_path) {
    this->map_info.load_map_info(file_path);
}


vector<vector<double>>
Vehicle::update(vector<double> ego_car_data, vector<vector<double>> sensor_fusion, vector<double> previous_path_x,
                vector<double> previous_path_y, double end_path_s, double end_path_d) {

    this->status.x = ego_car_data[0];
    this->status.y = ego_car_data[1];
    this->status.s = ego_car_data[2];
    this->status.d = ego_car_data[3];
    this->status.yaw = ego_car_data[4];
    this->status.v_yaw = ego_car_data[5];

    // new trajectory continue from end path
    if (!previous_path_x.empty()){
        this->status.s = end_path_s;
        this->status.d = end_path_d;
    }

    // predict other cars at time of previous end path
    double duration = (double)previous_path_x.size() * (double)SIM_DT;
    map<int, Vehicle> preds = predict_other_vehicles(sensor_fusion, duration);

    // evaluate next feasible state and trajectory
    vector<string> next_states = this->successor_states();
    vector<vector<double>> next_trajectory;

    double v_best = 0;
    string next_state;
    vector<double> next_param;
    vector<vector<double>> next_wps;
    vector<vector<double>> next_traj;

    for (auto state : next_states){
        vector<double> candidates_param;

        if (state == "KL"){
            cout << "[update] KL" << endl;
            candidates_param = get_KL_param(preds);
            cout << "[update] KL, feasible s, d, v_feasible, v_target = ";
            for(auto each : candidates_param){
                cout << each << ", ";
            }
            cout << endl;

            if (candidates_param[3] > v_best){
                v_best = candidates_param[3];
                next_param = candidates_param;
                next_state = "KL";
            }

        }else if (state == "LCL"){
            cout << "[update] LCL" << endl;
            candidates_param = get_LCL_param(preds);
            cout << "[update] LCL, feasible s, d, v_feasible, v_target = ";
            for(auto each : candidates_param){
                cout << each << ", ";
            }
            cout << endl;

            if (candidates_param[3] > v_best){
                v_best = candidates_param[3];
                next_param = candidates_param;
                next_state = "LCL";
            }

        }else if (state =="LCR"){
            cout << "[update] LCR" << endl;
            candidates_param = get_LCR_param(preds);
            cout << "[update] LCR, feasible s, d, v_feasible, v_target = ";
            for(auto each : candidates_param){
                cout << each << ", ";
            }
            cout << endl;

            if (candidates_param[3] > v_best){
                v_best = candidates_param[3];
                next_param = candidates_param;
                next_state = "LCR";
            }
        }
    }
    cout << "next_state, s, d, v_desired, v_target = " << next_state << ", ";
    for (auto each : next_param){
        cout << each << ", ";
    }
    cout << endl;
    next_wps = get_waypoint(next_param);
    next_trajectory = generate_trajectory(next_param[2], next_wps, previous_path_x, previous_path_y);
    this->status.v_yaw_desired = next_param[2];

    //---- drive in straight
//    vector<double> wp0 = {this->status.s + 30, 6};
//    vector<double> wp1 = {this->status.s + 60, 6};
//    vector<double> wp2 = {this->status.s + 90, 6};
//    vector<vector<double>> waypoints = {wp0, wp1, wp2};
//    vector<vector<double>> new_trajactory = generate_trajectory(22, waypoints, previous_path_x, previous_path_y);

    vector<vector<double>> dummy_trajectory;
    dummy_trajectory = generate_KL_trajectory(preds, previous_path_x, previous_path_y);

//    if (this->status.v_yaw < 10 ){
//        dummy_trajectory = generate_KL_trajectory(preds, previous_path_x, previous_path_y);
//    } else{
//        dummy_trajectory = generate_LCL_trajectory(preds, previous_path_x, previous_path_y);
//    }


    return next_trajectory;
}

vector<string> Vehicle::successor_states() {
    vector<string> states;
    states.push_back("KL");
    string state = this->status.state;
    if (state.compare("KL") == 0) {
        states.push_back("LCL");
        states.push_back("LCR");
    } else if (state.compare("LCL") == 0) {
        states.push_back("LCL");
    } else if (state.compare("LCR") == 0) {
        states.push_back("LCR");
    }
    return states;
}

vector<vector<double>>
Vehicle::generate_trajectory(double v_desired, vector<vector<double>> waypoints,
                             vector<double> previous_path_x, vector<double> previous_path_y) {
    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = this->status.x;
    double ref_y = this->status.y;
    double ref_yaw = deg2rad(this->status.yaw);

    int prev_size = (int)previous_path_x.size();

    // if previous size is almost empty, use the car as starting reference
    if (prev_size < 2) {
        // use two points that make path tangent to the car
        // i.e. drive along yaw direction
        double prev_car_x = this->status.x - cos(this->status.yaw);
        double prev_car_y = this->status.y - sin(this->status.yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(this->status.x);
        ptsy.push_back(prev_car_y);
        ptsy.push_back(this->status.y);
    }
    else {
        // use previous path's end point as starting reference
        // redefine reference state as previous path and points
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];
        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        //use two points that make the path tangent to the previous path's end points
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // add waypoints into new trajectory
    for (auto wp: waypoints){
        vector<double> next_wp_xy = this->map_info.getXY(wp[0], wp[1]);
        ptsx.push_back(next_wp_xy[0]);
        ptsy.push_back(next_wp_xy[1]);
    }

    // shift car reference angle to 0 degrees
    for (int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;
        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // create spline
    // using spline lib
    // use 6 waypoints since spline lib is cubic interpolation
    // 1-3 three points in previous_path[0] to [2]
    // 4-6 three points at car_s + 35/40/50,
    // https://discussions.udacity.com/t/how-many-points-and-which-points-for-interpolation-spline/413380
    tk::spline s;
    // set (x,y) points to spline
    s.set_points(ptsx, ptsy);

    // new trajectory
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
    // start with all of the previous path points from last time
    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // ------ Jerk-Minimized Trajectory from Aaron---------
    // calculate how to break up spline points so that we travel at our desired reference velocity
    // walkthrough video from 30' to 38'
    // purpose of this code, all the given trajectory points are equally divided into segments,
    // with considering 0.02 sim_dt and current desired velocity.
    // which means for next iteration, the previous_path contains not ONLY coordinates info but also with speed info.
    // the beginning of previous_path can contains more early speed info.
    // only the last part of previous_path contains last calculated speed.
    // e.g. v_prev = ds_prev / 0.02, sim_dt = 0.02

    double target_x = 30.0; // a compensation coefficient, to approximate the driving curve as straight line
    double target_y = s(target_x); // query y value with given x in spline
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
    double x_add_on = 0;
    // d = N * 0.02 * velocity, 0.02 is dt to next waypoints
    for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
        double N = (target_dist / (.02 * v_desired)); // ref_vel/2.24 = ref_vel*0.447, mph to m/s
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to normal after rotating it earlier
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    return {next_x_vals, next_y_vals};

}


vector<double> Vehicle::get_kinematics(const map<int, Vehicle> &preds, int lane_id) {

    double v_feasible = this->status.v_yaw_desired;
    double dist_feasible = SENSOR_RANGE; // obstacle free with sensor range in give lane
//    cout << "[get_kinematics] last v_feasible = " << v_feasible << endl;

    //---- ahead collision test
    Vehicle car_ahead;
    double v_target = MAX_SPEED;
    if (get_vehicle_ahead(preds, lane_id, car_ahead)){
        double dist = car_ahead.status.s - this->status.s;
//        cout << "[get_kinematics] found car ahead, dist = " << dist << ", v = " << car_ahead.status.v_yaw << endl;

        if (dist < 50 && dist > 25){
            v_target = car_ahead.status.v_yaw;
//            cout << "[get_kinematics] follow with v_target = " << v_target << endl;
        } else if (dist < 25 && dist > 10){
            // lower speed to avoid crash
            v_target = car_ahead.status.v_yaw -1; // [m/s]
//            cout << "[get_kinematics] slow down and follow with v_target = "<< v_target << endl;
        } else if (dist < 10){
            v_target = car_ahead.status.v_yaw -2;
            this->status.v_yaw_desired -= 5 * 0.02;
//            cout << "[get_kinematics] break with v_target = " << v_target << endl;
        } else {
//            cout << "[get_kinematics] free with v_target = " << v_target << endl;
        }

    } else {
//        cout << "[get_kinematics] free with v_target = " << v_target << endl;
    }

    // about magic number 0.224 in Aaron's code
    // 1[mph]=0.447[m/s] <-> 1[m/s] = 2.24[mph]
    // discussions: https://discussions.udacity.com/t/maximum-jerk-of-comfort/3823837
    // "The jerk threshold for discomfort lies around 0.5 m/s3, ranging up to 0.9m/s3"
    // https://www.diva-portal.org/smash/get/diva2:839140/FULLTEXT01.pdf
    // 1[mph]=0.447[m/s] <-> 1[m/s] = 2.24[mph]
    // ref_v -= 0.224 means jerk=5[m/s/s], dt= 0.02[s]
    // 0.02[s] * a = 0.224[mph] -> 0.02[s] * a = 0.1[m/s] -> a = 5[m/s/s]
    if (this->status.v_yaw < v_target) {
        v_feasible += MAX_JERK * SIM_DT;  // accelate with max allowed acceleration due to max_jerk
        v_feasible = min(v_feasible, (double)MAX_SPEED);
    } else if (this->status.v_yaw > v_target) {
        v_feasible -= MAX_JERK * SIM_DT;
        v_feasible = max(v_feasible, (double)0);
    }
//    cout << "[get_kinematics] next v_feasible = " << v_feasible << endl;

    return {dist_feasible, v_feasible, v_target};
}

vector<vector<double>>
Vehicle::generate_KL_trajectory(const map<int, Vehicle>& preds, vector<double> previous_path_x, vector<double> previous_path_y) {

    int lane_id = this->get_lane_id();
    lane_id = max(lane_id, 0);
    double d_target = lane_id*MAP_LANE_WIDTH + 0.5*MAP_LANE_WIDTH;

    vector<double> kinematics = this->get_kinematics(preds, this->get_lane_id());
    this->status.v_yaw_desired = kinematics[1];
    double s_delta = kinematics[0]/3;
//    cout << "[generate_KL_trajectory] s_delta = " << s_delta << endl;
    //---- drive in straight
    vector<double> wp0 = {this->status.s + 30, d_target};
    vector<double> wp1 = {this->status.s + 30 + s_delta, d_target};
    vector<double> wp2 = {this->status.s + 30 + 2*s_delta, d_target};

    vector<vector<double>> waypoints = {wp0, wp1, wp2};
//    cout << "[generate_KL_trajectory] ego car.v_yaw_desired = " << this->status.v_yaw_desired << endl;

    vector<vector<double>> new_trajactory = generate_trajectory(this->status.v_yaw_desired, waypoints, previous_path_x, previous_path_y);

    return new_trajactory;
}


vector<double> Vehicle::get_KL_param(const map<int, Vehicle> &preds) {
    int lane_id = this->get_lane_id();
    lane_id = max(lane_id, 0); // larger than 0
    lane_id = min(lane_id, 2); // smaller then 2

    double d_target = lane_id*MAP_LANE_WIDTH + 0.5*MAP_LANE_WIDTH;
    vector<double> kinematics = this->get_kinematics(preds, lane_id);

    return {kinematics[0], d_target, kinematics[1], kinematics[2]};
}

vector<double> Vehicle::get_LCL_param(const map<int, Vehicle> &preds) {
    int lane_id = this->get_lane_id() - 1;
    if (lane_id < 0){
        return {-1, -1, -1, -1};
    }

    double d_target = lane_id*MAP_LANE_WIDTH + 0.5*MAP_LANE_WIDTH;
    vector<double> kinematics = this->get_kinematics(preds, lane_id);

    return {kinematics[0], d_target, kinematics[1], kinematics[2]};
}

vector<double> Vehicle::get_LCR_param(const map<int, Vehicle> &preds) {
    int lane_id = this->get_lane_id() + 1;
    if (lane_id > 2){
        return {-1, -1, -1, -1};
    }

    double d_target = lane_id*MAP_LANE_WIDTH + 0.5*MAP_LANE_WIDTH;
    vector<double> kinematics = this->get_kinematics(preds, lane_id);

    return {kinematics[0], d_target, kinematics[1], kinematics[2]};
}

vector<vector<double>> Vehicle::get_waypoint(vector<double> param) {
    double s_delta = param[0]/3;
    double d_target = param[1];

    //---- drive in straight
    vector<double> wp0 = {this->status.s + 30, d_target};
    vector<double> wp1 = {this->status.s + 30 + s_delta, d_target};
    vector<double> wp2 = {this->status.s + 30 + 2*s_delta, d_target};

    vector<vector<double>> waypoints = {wp0, wp1, wp2};

    return waypoints;
}

bool Vehicle::get_vehicle_ahead(const map<int, Vehicle> &preds, int lane, Vehicle &r_vehicle) {
    bool found_vehicle = false;
    double min_distance = std::numeric_limits<double>::infinity();

    Vehicle temp_vehicle;
    for (auto vehicle : preds){
        // first is vehicle ID from sensor fusion
        // second is vector<vehicle> object

        // distance between leading vehicle at the time of beginning of new planning
        // cout << "other car, id, s, d = "<<vehicle.first<<", "<<vehicle.second.status.s<<", "<<vehicle.second.status.d<<endl;
        double dist = vehicle.second.status.s - this->status.s;
        if (vehicle.second.get_lane_id() == lane && dist > 0){
            if ( dist < min_distance ){
                min_distance = dist;
                r_vehicle = vehicle.second;
                found_vehicle = true;
            }
        }
    }
    return found_vehicle;
}

int Vehicle::get_lane_id() {
    // get lane id from d
    // id definition
    // 2 1 0 || 0 1 2
    int lane_id = -1;
    if (this->status.d >= 0 && this->status.d < MAP_LANE_WIDTH){
        lane_id = 0;
    } else if (this->status.d >= MAP_LANE_WIDTH && this->status.d < 2*MAP_LANE_WIDTH) {
        lane_id = 1;
    } else if (this->status.d >= 2*MAP_LANE_WIDTH && this->status.d < 3*MAP_LANE_WIDTH) {
        lane_id = 2;
    }
    return  lane_id;
}

map<int, Vehicle> Vehicle::predict_other_vehicles(const vector<vector<double>> &sensor_fusion, double duration) {
    map<int, Vehicle> preds;

    for (auto vehicle_data : sensor_fusion){
        int car_id = (int)vehicle_data[0];

        Vehicle other_car;
        other_car.status.x = vehicle_data[1];
        other_car.status.y = vehicle_data[2];
        double vx = vehicle_data[3];
        double vy = vehicle_data[4];
        other_car.status.s = vehicle_data[5];
        other_car.status.d = vehicle_data[6];
        other_car.status.v_yaw = sqrt(vx * vx + vy * vy);

        other_car.status.s += (duration * other_car.status.v_yaw);  // an approximation, since other car drive along the lane
        preds[car_id] = other_car;
    }

    return preds;
}





