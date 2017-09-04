#ifndef PID_H
#define PID_H

#include <chrono>

class PID {
public:
    /*
    * Errors
    */
    double p_error;
    double i_error;
    double d_error;

    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double init_cte);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();

    void SetLastCte(double cte);
    double GetOutput();

private:
    double last_cte_;
    double total_error_;
    std::chrono::milliseconds last_time_;
};

#endif /* PID_H */
