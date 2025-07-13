#include <cmath>
#include "pid_controller.hpp"

PIDController::PIDController(double kp, double ki, double kd, double max_output, double dead_zone)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), previous_error_(0.0), max_output_(max_output), dead_zone_(dead_zone) {}

double PIDController::compute(double target, double current, double dt) {
    double error = target - current;
    // 死区判断
    if (std::abs(error) < dead_zone_) {
        integral_ = 0.0; // 死区内清除积分
        return 0.0;
    }
    integral_ += error * dt;
    double derivative = (error - previous_error_) / dt;
    previous_error_ = error;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    // 最大输出限幅
    if (output > max_output_) output = max_output_;
    if (output < -max_output_) output = -max_output_;
    return output;
}

void PIDController::set_params(double kp, double ki, double kd, double max_output, double dead_zone) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_output_ = max_output;
    dead_zone_ = dead_zone;
}