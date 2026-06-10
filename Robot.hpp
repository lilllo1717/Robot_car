#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <string.h>
#include <unistd.h>
#include <memory>
#include <sys/socket.h>
#include <csignal>

#include "audio/Audio.hpp"
#include "lidar/Lidar.hpp"
#include "navigator/Navigator.hpp"
#include "arduinoConnection/ArduinoConnection.hpp"

const float SAFE_DISTANCE    = 0.5f;

class RobotCar
{
    public:
        enum class RobotMode {AUTO, MANUAL, STOPPED};
        enum class RobotState {LEFT, RIGHT, FORWARD, BACKING_UP, TURN_AROUND};
        RobotCar();
        ~RobotCar();

        void start();
        void stop();
        void switchMode(RobotMode newMode);

        void lidarLoop();

        void runRobotManual();
        bool runRobotAutopilot();


    private:
        ArduinoConnection _arduino;
        Audio _audio;
        Lidar _lidar;
        Navigator _navigator;

        std::thread _lidarThread;
        std::thread _audioThread;
        // std::atomic<bool> _running;
        bool _running;


        std::mutex _lidarMutex;
        std::mutex _audioMutex;

        std::chrono::steady_clock::time_point _turnStartTime;
        std::chrono::milliseconds _turnDuration;
        std::chrono::milliseconds _backupDuration;
        std::chrono::milliseconds _turnAroundDuration;

        
        std::atomic<RobotMode> _mode = RobotMode::STOPPED;
        RobotState _state = RobotState::FORWARD;

        bool _isTurning;


};

#endif
