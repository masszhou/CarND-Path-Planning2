//
// Created by zzhou on 16.01.18.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include "map.h"
class Vehicle {
public:
    Vehicle();

    void load_map_info(const string &file_path);

    Map map_info;

};


#endif //PATH_PLANNING_VEHICLE_H
