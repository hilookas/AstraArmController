#ifndef _MAIN_H_
#define _MAIN_H_

extern int err_cnt[10];

struct config_t {
  int version = 2;
  int EIShaper_enabled = 1;
  float EIShaper_freq = 5.5;
  float EIShaper_V = 0.05;
  float EIShaper_ctrl_freq = 48;
  int servo_vel = 32766;
  int servo_acc = 5;
  int servo_backlash = 32; // 默认死区是2, 128可以更好的
  int servo_vel2 = 32766;
  int servo_acc2 = 25;
};

extern config_t config;

#endif