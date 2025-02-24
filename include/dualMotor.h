#ifndef DUAL_MOTOR_H
#define DUAL_MOTOR_H

#include <Arduino.h>

#define JOINT_NUM 2
#define NONE_JOINT_NUM 4

void dualMotorUpdatePos(uint16_t pos[]);

void dualMotorSetup();

void dualMotorReadPos(uint16_t read_pos[]);

void setupTorque(int enable);

void initJoint(int id, int offset);

extern float pidtune_kp, pidtune_ki, pidtune_kd, pidtune_ki_max, pidtune_kp2, pidtune_kp2_err_point, pidtune_ki_clip_thres, pidtune_ki_clip_coef;

#endif