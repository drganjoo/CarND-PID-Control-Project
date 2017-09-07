#ifdef MSVC
#pragma warning(push)
#pragma warning(disable: 4514)
#pragma warning(disable: 4820)
#endif
#include "PID.h"
#ifdef MSVC
#pragma warning(pop)
#endif

using namespace std;
using namespace std::chrono;

/*---------------------------------------------------------------------------------*/
PID::PID() : PID("") {

}

PID::PID(string debug_name) {
    Init(0, 0, 0, 0);
    debug_name_ = debug_name;
}

void PID::Init(double init_kp, double init_ki, double init_kd, double init_cte) {
    kp_ = init_kp;
    ki_ = init_ki;
    kd_ = init_kd;
    p_error_ = init_cte;
    i_error_ = 0;
    d_error_ = 0;
    total_error_ = 0;

    last_time_ = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
}

void PID::UpdateError(double cte, bool include_in_error /*= false */) {
//    milliseconds now  = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
//    double dt = (now.count() - last_time_.count()) / 1000.0;
//    last_time_ = now;

    //if (dt == 0)
    double dt = 0.1;

    d_error_ = (p_error_ - cte) / dt;
    p_error_ = cte;
    i_error_ += cte * dt;

    if (include_in_error)
        total_error_ += cte * cte;
}

double PID::TotalError() {
    return total_error_;
}

void PID::SetLastCte(double cte) {
    p_error_ = cte;
}

double PID::GetOutput() {
    double output = -kp_ * p_error_ - kd_ * d_error_ - ki_ * i_error_;

    if (output > 1)
        output = 1;
    else if (output < -1)
        output = -1;

    return output;
}