//
// Created by zzhou on 16.01.18.
//

#include "vehicle.h"


Vehicle::Vehicle() {

}

void Vehicle::load_map_info(const string &file_path) {
    this->map_info.load_map_info(file_path);
}

