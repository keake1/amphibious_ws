#ifndef MY_PID_CONTROLLER_PID_CONTROLLER_HPP
#define MY_PID_CONTROLLER_PID_CONTROLLER_HPP

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double max_output = 1e6, double dead_zone = 0.0);
    double compute(double target, double current, double dt);
    void set_params(double kp, double ki, double kd, double max_output = 1e6, double dead_zone = 0.0);
private:
    double kp_;  // 比例增益
    double ki_;  // 积分增益
    double kd_;  // 微分增益
    double integral_;  // 积分值
    double previous_error_;  // 上一次误差
    double max_output_;  // 最大输出值
    double dead_zone_; // 死区阈值
};

#endif  // MY_PID_CONTROLLER_PID_CONTROLLER_HPP