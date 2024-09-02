#ifndef DUAL_MOTOR_H
#define DUAL_MOTOR_H

#include <Arduino.h>

#define JOINT_NUM 2
#define NONE_JOINT_NUM 4

void dualMotorUpdatePos(uint16_t pos[]);

void dualMotorSetup();

void dualMotorReadPos(uint16_t read_pos[]);

void setupTorque(int enable);

#endif