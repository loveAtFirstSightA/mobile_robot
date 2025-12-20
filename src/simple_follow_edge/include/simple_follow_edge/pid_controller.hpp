#ifndef SIMPLE_FOLLOW_EDGE__PID_CONTROLLER_HPP_
#define SIMPLE_FOLLOW_EDGE__PID_CONTROLLER_HPP_

class PIDController
{
public:
    PIDController(double p, double i, double d, double setpoint)
        : p_(p), i_(i), d_(d), setpoint_(setpoint), prev_error_(0), integral_(0) {}

    double compute(double current_value)
    {
        double error = setpoint_ - current_value;
        integral_ += error;
        double derivative = error - prev_error_;
        prev_error_ = error;
        return p_ * error + i_ * integral_ + d_ * derivative;
    }

private:
    double p_;
    double i_;
    double d_;
    double setpoint_;
    double prev_error_;
    double integral_;
};

#endif  // SIMPLE_FOLLOW_EDGE__PID_CONTROLLER_HPP_
