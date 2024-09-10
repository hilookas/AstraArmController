#ifndef _TRAP_TRAJ_H
#define _TRAP_TRAJ_H

class TrapezoidalTrajectory {
public:
    struct Config_t {
        float vel_limit = 3200.0f;   // [step/s]
        float accel_limit = 3200.0f; // [step/s^2]
        float decel_limit = 3200.0f; // [step/s^2]
        float inertia = 0;
        float current_meas_period = 0.01; // TIMER_TIMEOUT_US -> s
    };
    
    struct Step_t {
        float Y;
        float Yd;
        float Ydd;
    };

    bool planTrapezoidal(float Xf, float Xi, float Vi);
    Step_t eval(float t);

    Config_t config_;

    float Xi_;
    float Xf_ = 2048;
    float Vi_;

    float Ar_;
    float Vr_;
    float Dr_;

    float Ta_;
    float Tv_;
    float Td_;
    float Tf_;

    float yAccel_;

    float t_;

    float pos_setpoint_ = 2048; // _setpoint 和 last_ 是不大一样的，last是真实值，setpoint是目标值，拿last值算是有问题的，这个范式应该是一个萝卜钓竿，不断跟随，而不是跟随自己现有的情况
    float vel_setpoint_;
    float torque_setpoint_;
    
    void update();

    bool trajectory_done_ = true;
};

#endif