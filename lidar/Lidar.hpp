#ifndef LIDAR_HPP
#define LIDAR_HPP

#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <string.h>
#include <unistd.h>
#include <memory>

class Lidar
{
    
    public:
    Lidar(const std::string& port);
    ~Lidar();
    bool processLidarData();
    
    static const int NUM_ZONES = 5;
    std::array<float, NUM_ZONES> _min_zone_distances;
    std::array<float, NUM_ZONES> _stable_zone_distances;

    private:
        CYdLidar _laser;
        LaserScan _scan;
        std::string _port;
        float _zoneStart = -1.745f;
        float _zoneSize  = 2 * 1.745f / NUM_ZONES;
};


#endif