#ifndef PID_H
#define PID_H

#include <ostream>
#include <chrono>
#include <string>

class PID {
public:
    /*
    * Constructor
    */
    PID();
    PID(std::string debug_name);

    /*
    * Destructor.
    */
    ~PID() = default;

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double init_cte);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte, bool include_in_error = false);

    /*
    * Calculate the total PID error.
    */
    double TotalError();

    void SetLastCte(double cte);
    double GetOutput();

    double GetDError() {
        return d_error_;
    }

private:
    double p_error_;
    double i_error_;
    double d_error_;
    double kp_;
    double ki_;
    double kd_;
    double total_error_;
    std::chrono::milliseconds last_time_;
    std::string debug_name_;
};

#endif /* PID_H */
