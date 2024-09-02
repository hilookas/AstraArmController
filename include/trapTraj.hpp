#ifndef _TRAP_TRAJ_H
#define _TRAP_TRAJ_H

class TrapezoidalTrajectory {
public:
    struct Config_t {
        float vel_limit = 400.0f;   // [step/s]
        float accel_limit = 400.0f; // [step/s^2]
        float decel_limit = 400.0f; // [step/s^2]
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
    
    float update();

    bool trajectory_done_ = true;
};

#endif