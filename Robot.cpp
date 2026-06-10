#include "Robot.hpp"

RobotCar::RobotCar() :
    _arduino("/dev/ttyACM0", 9600),
    _audio(),
    _lidar("/dev/ttyUSB0"),
    _navigator(),
    _lidarThread(),
    _audioThread(),
    _running(false),
    _lidarMutex(),
    _audioMutex(),
    _turnStartTime(),
    _turnDuration(500),
    _backupDuration(1000),
    _turnAroundDuration(1200),
    _mode(),
    _state(),
    _isTurning(false)
{
    _arduino.connect();
}

RobotCar::~RobotCar()
{
    _running = false;
    if (_lidarThread.joinable())
        _lidarThread.join();
    _arduino.sendCommand("STOP");
}

void RobotCar::switchMode(RobotMode newMode)
{
    _mode = newMode;
}

void RobotCar::lidarLoop()
{
    while (_running)
    {
        // std::lock_guard<std::mutex> lock(_lidarMutex);
        _lidar.processLidarData();
    }
}


void RobotCar::start()
{
    switchMode(RobotMode::AUTO);
    std::string cmd_input = "forward";
    _running = true;
    _lidarThread = std::thread(&RobotCar::lidarLoop, this);

    while(_running)
    {
        if (_mode == RobotMode::MANUAL)
        {
            runRobotManual();
        }
        else if (_mode == RobotMode::AUTO)
        {
            if (!runRobotAutopilot())
            {
                
                _mode = RobotMode::STOPPED;
                _running = false;
                fprintf(stderr, "Autopilot failed, stopping car\n");

            }

        }
    }

}

void RobotCar::stop()
{
    _arduino.sendCommand("STOP");
    _running = false;
}