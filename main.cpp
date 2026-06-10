#include "Robot.hpp"

RobotCar* g_robot = nullptr;

void signalHandler(int sig)
{
    (void)sig;
    if (g_robot)
        g_robot->stop();
}

int main(void)
{
    RobotCar robot;
    g_robot = &robot;
    std::signal(SIGINT, signalHandler);
    ydlidar::os_init();
    robot.start();
    return (0);
};
