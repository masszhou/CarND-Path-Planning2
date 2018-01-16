//
// Created by zzhou on 16.01.18.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <string>
#include <vector>

using namespace std;

class Map {
public:
    Map();

    void load_map_info(const string &file_path);

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    vector<double> getFrenetSD(double x, double y, double theta);

    // Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double> getXY(double s, double d);

    vector<double> getNormal(double x, double y, double theta);

    double distance(double x1, double y1, double x2, double y2);
    int ClosestWaypoint(double x, double y);
    int NextWaypoint(double x, double y, double theta);

private:
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    vector<double> interpolated_waypoints_x;
    vector<double> interpolated_waypoints_y;
    vector<double> interpolated_waypoints_s;
    vector<double> interpolated_waypoints_dx;
    vector<double> interpolated_waypoints_dy;
};


#endif //PATH_PLANNING_MAP_H
