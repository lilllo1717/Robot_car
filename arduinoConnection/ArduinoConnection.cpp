#include "ArduinoConnection.hpp"

ArduinoConnection::ArduinoConnection(const std::string& port, int baudrate)
    :   _port(port),
        _baudrate(baudrate),
        _connected(false)
{

}

ArduinoConnection::~ArduinoConnection()
{
    if (_connected)
    {
        close(_serialFd);
    }
}

bool ArduinoConnection::connect()
{
    _serialFd = open(_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (_serialFd < 0)
    {
        fprintf(stderr, "Failed to open port %s\n", _port.c_str());
        return false;
    }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(_serialFd, &tty);
    cfsetspeed(&tty, B9600);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_lflag &= ~ICANON;   // raw mode, don't wait for newline
    tty.c_lflag &= ~ECHO;     // no echo
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;     // no signal chars
    tty.c_iflag &= ~IXON;     // no flow control
    tty.c_iflag &= ~IXOFF;
    tty.c_oflag &= ~OPOST;    // raw output
    tty.c_cc[VMIN]  = 0;      // non blocking read
    tty.c_cc[VTIME] = 0;
    tcsetattr(_serialFd, TCSANOW, &tty);
    sleep(2);
    _connected = true;
    return true;
}

void ArduinoConnection::disconnect()
{
    if (_connected)
    {
        sendCommand("STOP\n");
        _connected = false;
        close(_serialFd);
    }
}

void ArduinoConnection::sendCommand(const std::string& command)
{
    if (!_connected)
        return;
    std::cout << "command: " << command << "\n";
    ssize_t written = write(_serialFd, command.c_str(), command.size());
    if (written < 0)
    {
        fprintf(stderr, "Failed to send command\n");
        _connected = false;
    }
}
