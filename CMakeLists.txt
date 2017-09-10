cmake_minimum_required (VERSION 3.5)
project(PID)

#set(CMAKE_CXX_STANDARD 17)
add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")
set(sources src/PID.cpp src/main.cpp src/Simulator.cpp src/twiddle.cpp src/helpers.cpp src/SpeedController.cpp src/SpeedController.h)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

if (WIN32)
    include_directories(c:/msys64/usr/local/include)
    link_directories(c:/msys64/usr/local/lib)
endif()

add_executable(PID ${sources})

target_link_libraries(PID ssl uv uWS z)

if (WIN32)
    target_link_libraries(PID ws2_32)
endif()
