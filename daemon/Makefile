CXX      := g++
DEPFLAGS := -MMD -MP
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -pthread -I src -I third_party $(DEPFLAGS)
LDFLAGS  := -pthread

SRC := src/main.cpp \
       src/can/socket_can.cpp \
       src/can/can_bus_manager.cpp \
       src/can/generic_listener.cpp \
       src/config/config_loader.cpp \
       src/motor/actuator.cpp \
       src/ipc/udp_broadcaster.cpp \
       src/ipc/udp_server.cpp \
       src/control/robot.cpp

OBJ := $(SRC:src/%.cpp=build/%.o)
DEP := $(OBJ:.o=.d)

BIN := build/humanoid_daemon

all: $(BIN)

$(BIN): $(OBJ)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

build/%.o: src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

-include $(DEP)

clean:
	rm -rf build

.PHONY: all clean
