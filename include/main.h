#ifndef _MAIN_H_
#define _MAIN_H_

extern int err_cnt[10];

struct config_t {
  int version = 2;
  int servo_vel = 32766;
  int servo_acc = 5;
};

extern config_t config;

#endif