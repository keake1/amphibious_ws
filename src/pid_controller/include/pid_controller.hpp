#ifndef MY_PID_CONTROLLER_PID_CONTROLLER_HPP
#define MY_PID_CONTROLLER_PID_CONTROLLER_HPP

class PIDController {
public:
    PIDController(double kp, double ki, double kd);
    double compute(double target, double current, double dt);

private:
    double kp_;  // 比例增益
    double ki_;  // 积分增益
    double kd_;  // 微分增益
    double integral_;  // 积分值
    double previous_error_;  // 上一次误差
};

#endif  // MY_PID_CONTROLLER_PID_CONTROLLER_HPP