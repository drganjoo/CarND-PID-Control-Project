//
// Created by Fahad Zubair on 9/3/17.
//

#pragma once

#ifndef PID_HELPERS_H
#define PID_HELPERS_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <ostream>


// For converting back and forth between radians and degrees.
constexpr double pi(); 
double deg2rad(double x);
double rad2deg(double x);

#endif //PID_HELPERS_H
