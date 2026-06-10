#ifndef ARDUINO_CONNECTION_HPP
#define ARDUINO_CONNECTION_HPP

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
#include <termios.h>
#include <fcntl.h>

class ArduinoConnection
{
    private:
        std::string _port;
        int _baudrate;
        int _serialFd;
        bool _connected;

        
    public:
        ArduinoConnection(const std::string& port, int baudrate);
        void sendCommand(const std::string& command);
        ~ArduinoConnection();

        bool connect();
        void disconnect();





};


#endif