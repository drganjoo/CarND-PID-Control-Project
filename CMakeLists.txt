project(PID)
cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/PID.cpp src/main.cpp src/Simulator.cpp src/twiddle.cpp src/helpers.cpp)

if (MSVC)
    add_definitions(-D_USE_MATH_DEFINES)

    set(VCPKG_INSTALL C:/vcpkg/installed/x86-windows)
    message("Using vcpkg_install ${VCPKG_INSTALL}")

    include_directories(${VCPKG_INSTALL}/include)
    link_directories(${VCPKG_INSTALL}/lib)
    link_directories(${VCPKG_INSTALL}/debug/lib)
else()
    include_directories(/usr/local/include)
    link_directories(/usr/local/lib)
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(pid ${sources})

if (NOT MSVC)
    target_link_libraries(pid z ssl uv uWS)
else()
    add_custom_command(TARGET pid POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "$<$<CONFIG:debug>:${VCPKG_INSTALL}/debug/bin>$<$<CONFIG:release>:${VCPKG_INSTALL}/bin>"
        $<TARGET_FILE_DIR:pid>)

    target_link_libraries(pid ssleay32 libuv uWS ws2_32)
    target_link_libraries(pid optimized zlib )
    target_link_libraries(pid debug zlibd )
endif()