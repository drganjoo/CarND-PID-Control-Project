//
// Created by Fahad Zubair on 9/3/17.
//

#ifndef PID_HELPERS_H
#define PID_HELPERS_H

struct Measurement{
    double cte;
    double speed;
    double angle;
};

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

#endif //PID_HELPERS_H
