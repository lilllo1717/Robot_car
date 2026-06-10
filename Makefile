CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -I/usr/local/include/src
LDFLAGS  := -L/usr/local/lib
LDLIBS   := -lydlidar_sdk -lpthread -lm
TARGET   := RobotCar

SRCS     := main.cpp \
            Robot.cpp \
            runRobotAutopilot.cpp \
            runRobotManual.cpp \
            arduinoConnection/ArduinoConnection.cpp \
            audio/Audio.cpp \
            lidar/Lidar.cpp \
            navigator/Navigator.cpp

OBJS     := $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	find . -name "*.o" -delete

fclean: clean
	rm -f $(TARGET)

re: fclean all

run: all
	./$(TARGET)

.PHONY: all clean fclean re run