#include "Robot.hpp"

bool RobotCar::runRobotAutopilot()
{
   std::this_thread::sleep_for(std::chrono::milliseconds(100));
   // std::lock_guard<std::mutex> lock(_lidarMutex);
   // _lidar.processLidarData();
   std::cout << "running autopilot" << "\n";
   float most_left  = _lidar._stable_zone_distances[0];
   float left       = _lidar._stable_zone_distances[1];
   float front      = _lidar._stable_zone_distances[2];
   float right      = _lidar._stable_zone_distances[3];
   float most_right = _lidar._stable_zone_distances[4];

   std::cout << "Lidar distances - most_left: " << most_left << " left: " << left << " front: " << front
             << " right: " << right << " most_right: " << most_right << "\n";

   if (_isTurning)
   {
      auto elapsed = std::chrono::steady_clock::now() - _turnStartTime;
      std::chrono::milliseconds turnTime;
      if (_state == RobotState::TURN_AROUND)
         turnTime = _turnAroundDuration;
      else
         turnTime = _turnDuration;
      if (elapsed >= turnTime)
      {
         _isTurning = false;
      }
      return true;
   }
   if (front > SAFE_DISTANCE)
   {
      _arduino.sendCommand("FORWARD\n");
      _state = RobotState::FORWARD;
   }
   else if (left > right && most_left > SAFE_DISTANCE)
   {
      _arduino.sendCommand("LEFT\n");
      _state = RobotState::LEFT;
      _turnStartTime = std::chrono::steady_clock::now();
      _isTurning = true;
   }
   else if (right > left && most_right > SAFE_DISTANCE)
   {
      _arduino.sendCommand("RIGHT\n");
      _state = RobotState::RIGHT;
      _turnStartTime = std::chrono::steady_clock::now();
      _isTurning = true;
   }
   else
   {
      _state = RobotState::TURN_AROUND;
      _arduino.sendCommand("LEFT\n");
      _turnStartTime = std::chrono::steady_clock::now();
      _isTurning = true;
      
   }
   return true;
}