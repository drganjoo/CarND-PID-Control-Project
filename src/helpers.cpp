#include "helpers.h"
#include <ostream>

std::ostream& operator <<(std::ostream &os, const TelemetryMessage &m) {
	os << m.cte << ", " << m.angle << ", " << m.speed;
	return os;
}


constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
