#ifndef _MAIN_H_
#define _MAIN_H_

extern int err_cnt[10];

// #define AS_LEADER // lookas

struct config_t {
  int version = 1;
  int EIShaper_enabled = 1;
  float EIShaper_freq = 5.5;
  float EIShaper_V = 0.05;
  float EIShaper_ctrl_freq = 48;
  int servo_speed = 32766;
  int servo_acc = 25;
  int servo_backlash = 128; // 默认死区是2, 128可以更好的
  int servo_backlash2 = 128;
};

extern config_t config;

#endif