#include "Lidar.hpp"

Lidar::Lidar(const std::string& port) :
    _laser(),
    _scan(),
    _port(port)
{
    _laser.setlidaropt(LidarPropSerialPort, _port.c_str(), _port.size());

    int baudrate = 230400;
    _laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));

    int lidar_type = TYPE_TRIANGLE;
    _laser.setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));

    int device_type = YDLIDAR_TYPE_SERIAL;
    _laser.setlidaropt(LidarPropDeviceType, &device_type, sizeof(int));

    int sample_rate = 4;
    _laser.setlidaropt(LidarPropSampleRate, &sample_rate, sizeof(int));

    int abnormal_check_count = 4;
    _laser.setlidaropt(LidarPropAbnormalCheckCount, &abnormal_check_count, sizeof(int));

    bool resolution_fixed = false;
    _laser.setlidaropt(LidarPropFixedResolution, &resolution_fixed, sizeof(bool));

    bool auto_reconnect = true;
    _laser.setlidaropt(LidarPropAutoReconnect, &auto_reconnect, sizeof(bool));

    float angle_max = 100.0f;
    _laser.setlidaropt(LidarPropMaxAngle, &angle_max, sizeof(float));

    float angle_min = -100.0f;
    _laser.setlidaropt(LidarPropMinAngle, &angle_min, sizeof(float));

    bool ret = _laser.initialize();
    if (ret)
        ret = _laser.turnOn();
    else
    {
        fprintf(stderr, "%s\n", _laser.DescribeError());
        fflush(stderr);
    }
}

Lidar::~Lidar()
{
    _laser.turnOff();
    _laser.disconnecting();
}



// bool Lidar::processLidarData()
// {
//     _min_zone_distances.fill(std::numeric_limits<float>::max());

//     if (_laser.doProcessSimple(_scan))
//     {
//         for (const auto& point: _scan.points)
//         {
//             if (point.range <= 0)
//                 continue;
//             int zone = (int)((point.angle - _zoneStart)/_zoneSize);
//             if (zone >= 0 && zone < NUM_ZONES)
//             {
//                 // std::cout << "point.range: " << point.range << "\n";
//                 _min_zone_distances[zone] = std::min(point.range, _min_zone_distances[zone]);
//             }
//         }
//         return true;
//     }
//     fprintf(stderr, "Failed to get Lidar Data: %s\n", _laser.DescribeError());
//     fflush(stderr);
//     return false;
// }


// bool Lidar::processLidarData()
// {
//     _min_zone_distances.fill(std::numeric_limits<float>::max());
//     if (_laser.doProcessSimple(_scan))
//     {
//         if (!_scan.points.empty())
//         {
//             fprintf(stdout, "points=%zu first_angle=%.4f first_range=%.4f zoneStart=%.4f zoneSize=%.4f\n",
//                     _scan.points.size(),
//                     _scan.points[0].angle,
//                     _scan.points[0].range,
//                     _zoneStart,
//                     _zoneSize);
//             fflush(stdout);
//         }
//         else
//         {
//             std::cout << "no points\n";
//             return false;
//         }
//     }
// }

bool Lidar::processLidarData()
{
    std::array<float, NUM_ZONES> temp;
    temp.fill(std::numeric_limits<float>::max());
    
    if (_laser.doProcessSimple(_scan))
    {
        for (const auto& point: _scan.points)
        {
            if (point.range <= 0) continue;
            int zone = (int)((point.angle - _zoneStart) / _zoneSize);
            if (zone >= 0 && zone < NUM_ZONES)
                temp[zone] = std::min(point.range, temp[zone]);
        }
        _stable_zone_distances = temp;
        return true;
    }
    return false;
}