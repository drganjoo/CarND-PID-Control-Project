cmake_minimum_required (VERSION 3.5)
project(PID)

set(CMAKE_CXX_STANDARD 17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")
set(sources src/PID.cpp src/main.cpp src/Simulator.cpp src/twiddle.cpp src/helpers.cpp src/PIDSimControl.cpp src/PIDSimControl.h)

include_directories(c:/msys64/usr/local/include)
link_directories(c:/msys64/usr/local/lib)

add_executable(pid ${sources})

target_link_libraries(pid ssl libuv libuWS.dll ws2_32 z)
