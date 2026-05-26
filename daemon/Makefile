CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -pthread -I src -I third_party
LDFLAGS  := -pthread

SRC := src/main.cpp \
       src/can/socket_can.cpp \
       src/can/can_bus_manager.cpp \
       src/config/config_loader.cpp \
       src/motor/actuator.cpp \
       src/ipc/udp_broadcaster.cpp \
       src/ipc/udp_server.cpp \
       src/control/robot.cpp

OBJ := $(SRC:src/%.cpp=build/%.o)

BIN := build/humanoid_daemon

all: $(BIN)

$(BIN): $(OBJ)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

build/%.o: src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf build

.PHONY: all clean
