#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "dualMotor.h"

struct config_t {
  int version = 3;
  int EIShaper_enabled = 0;
  float EIShaper_freq = 5.5;
  float EIShaper_V = 0.05;
  float EIShaper_ctrl_freq = 48;

  float joint_vel_max = 800.f;
  float joint_acc = 800.f;
  int joint_backlash_compensate_feedforward = 200; // 200 pwm

  int non_joint_vel_max = 32766;
  int non_joint_acc = 25;
  bool init_pos_inited = false;
  int init_pos[4 * JOINT_NUM];
};

extern config_t config;

void write_config();

void configInit();

#endif