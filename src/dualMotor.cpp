#include <Arduino.h>
#include <SCServo.h>
#include "config.h"
#include "dualMotor.h"
#include "trapTraj.hpp"
#include <esp_task_wdt.h>

SMS_STS sts;

#define TIMER_TIMEOUT_US 15000 // change: current_meas_period
// velocity update in servo seems at 50Hz.

esp_timer_handle_t timer;

TrapezoidalTrajectory traj[JOINT_NUM];
float last_pos[JOINT_NUM + NONE_JOINT_NUM] = { 2048, 2048, 2048, 2048, 2048, 2048,  };
float last_vel[JOINT_NUM + NONE_JOINT_NUM] = { 0 };

uint16_t none_joint_goal_pos[NONE_JOINT_NUM] = { 2048, 2048, 2048, 2048,  };
uint16_t raw_goal_pos[JOINT_NUM] = { 2048, 2048,  };

#define ACC 100
#define MAX_VEL 10000

void read_pos() {
  int raw_pos[4 * JOINT_NUM];
  for (int i = 0; i < 4 * JOINT_NUM; ++i) {
    raw_pos[i] = sts.ReadPos(4 + i);
    if (sts.Err == 1) {
      Serial.print("Error reading #"); Serial.print(4 + i); Serial.print(", checkout your wire connection"); Serial.println();
    }
  }
  int raw_pos_non_joint[NONE_JOINT_NUM];
  for (int i = 0; i < NONE_JOINT_NUM; ++i) {
    raw_pos_non_joint[i] = sts.ReadPos(4 + 4 * JOINT_NUM + i);
    if (sts.Err == 1) {
      Serial.print("Error reading #"); Serial.print(4 + 4 * JOINT_NUM + i); Serial.print(", checkout your wire connection"); Serial.println();
    }
  }
  
  // Zero offest
  for (int i = 0; i < 4 * JOINT_NUM; ++i) {
    raw_pos[i] -= config.init_pos[i];
    if (raw_pos[i] < 0) raw_pos[i] += 4096;
    // Move middle from 0 to 2048
    raw_pos[i] += 2048;
    if (raw_pos[i] >= 4096) raw_pos[i] -= 4096;
  }
  
  // average load into 4 servo
  float pos[JOINT_NUM + NONE_JOINT_NUM];
  for (int i = 0; i < JOINT_NUM; ++i) {
    pos[i] = (raw_pos[i * 4] + (4096 - raw_pos[i * 4 + 1]) + raw_pos[i * 4 + 2] + (4096 - raw_pos[i * 4 + 3])) / 4.0;
  }

  for (int i = 0; i < NONE_JOINT_NUM; ++i) {
    pos[JOINT_NUM + i] = raw_pos_non_joint[i];
  }
  
  float vel[JOINT_NUM + NONE_JOINT_NUM];
  for (int i = 0; i < JOINT_NUM + NONE_JOINT_NUM; ++i) {
    vel[i] = (pos[i] - last_pos[i]) / TIMER_TIMEOUT_US * 1000000;
    float filter_vel = 0.2;
    vel[i] = vel[i] * filter_vel + last_vel[i] * (1 - filter_vel);
    // float filter_pos = 0.15860013531; // 1 / (1 + 1 / (2 * pi * f * TIMER_TIMEOUT_S))
    // pos[i] = pos[i] * filter_pos + last_pos[i] * (1 - filter_pos);
    last_pos[i] = pos[i];
    last_vel[i] = vel[i];
  }
}

bool torque_enabled = false;

int setupTorque_enable = 0;
bool updateSetupTorque = false;

void doSetupTorque(int enable) {
  if (enable == 0) {
    torque_enabled = false;
  } else if (enable == 1) {
    torque_enabled = true;
  }
  
  for (int i = 0; i < NONE_JOINT_NUM; ++i) {
    sts.EnableTorque(4 + 4 * JOINT_NUM + i, enable);
  }

  if (enable == 0) {
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      sts.EnableTorque(4 + i, enable);
    }
  } else if (enable == 1) {
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      sts.EnableTorque(4 + i, enable);
    }
    
    for (int i = 0; i < JOINT_NUM + NONE_JOINT_NUM; ++i) {
      last_vel[i] = 0;
    }
    read_pos();

    // set current pos as target
    uint16_t pos[JOINT_NUM + NONE_JOINT_NUM];
    for (int i = 0; i < JOINT_NUM + NONE_JOINT_NUM; ++i) {
      pos[i] = last_pos[i];
    }
    for (int i = 0; i < JOINT_NUM; ++i) {
      traj[i].trajectory_done_ = true;
      traj[i].Xf_ = pos[i];
      traj[i].pos_setpoint_ = pos[i];
    }
    dualMotorUpdatePos(pos);
    Serial.println("setup torque");
  } else if (enable == 128) {
    int init_pos0[4 * JOINT_NUM];
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      init_pos0[i] = sts.ReadPos(4 + i);
      if (!(50 < init_pos0[i] && init_pos0[i] < 4096 - 50)) {
        Serial.print("Maybe cause wrong init_pos0, check power status, or reinstall the servo #"); Serial.print(i); Serial.println();
        while (1) ;
      }
    }
    
    // preload
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      sts.writeWord(4 + i, SCSCL_GOAL_TIME_L, config.joint_backlash_compensate_feedforward);
    }

    // wait for stable
    sleep(1);
    
    int init_pos1[4 * JOINT_NUM];
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      init_pos1[i] = sts.ReadPos(4 + i);
    }

    // inversed preload
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      sts.writeWord(4 + i, SCSCL_GOAL_TIME_L, config.joint_backlash_compensate_feedforward + 1024); // negative as scs goal time format
    }

    // wait for stable
    sleep(1);
    
    int init_pos2[4 * JOINT_NUM];
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      init_pos2[i] = sts.ReadPos(4 + i);
    }

    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      if (std::abs(init_pos1[i] - init_pos2[i]) > 200) {
        Serial.print("Gap is too wide, check power status, or reinstall the servo #"); Serial.print(i); Serial.println();
        while (1) ;
      }

      config.init_pos_inited = true;
      config.init_pos[i] = (init_pos1[i] + init_pos2[i]) / 2;
    }
    write_config();
  }
}

void setupTorque(int enable) {
  setupTorque_enable = enable;
  updateSetupTorque = true;
}

int initJoint_id = 0;
int initJoint_offset = 0;
bool updateInitJoint = false;

void doInitJoint(int id, int offset) {
  Serial.printf("INIT_JOINT id=%d offset=%d\n", id, offset);

  sts.EnableTorque(id, 128);
  Serial.printf("Set current position as mid\n");

  sts.unLockEprom(id);//unlock EPROM-SAFE
  int offset_servo = sts.readWord(id, SMS_STS_OFS_L);
  Serial.printf("Offset in servo: %d\n", offset_servo);
  offset_servo += offset;
  if (offset_servo < 0) offset_servo += 4096;
  if (offset_servo >= 4096) offset_servo -= 4096;
  sts.writeWord(id, SMS_STS_OFS_L, offset_servo);
  sts.LockEprom(id);//EPROM-SAFE locked

  offset_servo = sts.readWord(id, SMS_STS_OFS_L);
  Serial.printf("Updated offset in servo: %d\n", offset_servo);
}

void initJoint(int id, int offset) {
  initJoint_id = id;
  initJoint_offset = offset;
  updateInitJoint = true;
}

float pidtune_kp = 0, pidtune_kd = 0, pidtune_ki = 0, pidtune_ki_max = 800, pidtune_kp2 = 0, pidtune_kp2_err_point = 0, pidtune_ki_clip_thres = 10, pidtune_ki_clip_coef = 0.5;

void timer_callback(void *arg) {
  if (updateSetupTorque) {
    updateSetupTorque = false;
    doSetupTorque(setupTorque_enable);
  }

  if (updateInitJoint) {
    updateInitJoint = false;
    doInitJoint(initJoint_id, initJoint_offset);
  }

  if (!torque_enabled) {
    return;
  }

  if (!config.init_pos_inited) {
    Serial.println("Joint pos is not inited");
    return;
  }

  read_pos();

  float goal_pos[JOINT_NUM];
  for (int i = 0; i < JOINT_NUM; ++i) {
    traj[i].update();
    goal_pos[i] = traj[i].pos_setpoint_;
  }

  // static float last_goal_pos[JOINT_NUM];
  // static float last_goal_pos_inited = false;
  // if (!last_goal_pos_inited) {
  //   for (int i = 0; i < JOINT_NUM; ++i) {
  //     last_goal_pos[i] = goal_pos[i];
  //   }
  //   last_goal_pos_inited = true;
  // }

  float kp = pidtune_kp, kd = pidtune_kd, ki = pidtune_ki, kp2 = pidtune_kp2, kp2_err_point = pidtune_kp2_err_point;

  static float last_err[JOINT_NUM];
  static float i_out[JOINT_NUM] = {};
  float out[JOINT_NUM] = {};
  float debug_signal[JOINT_NUM];

  // dithering to overcome coulomb friction
  // Ref: https://ieeexplore.ieee.org/document/7139742
  static float sticktion_compensation = 60;
  sticktion_compensation = -sticktion_compensation;
  
  for (int i = 0; i < JOINT_NUM; ++i) {
    float err = goal_pos[i] - last_pos[i];

    // float goal_vel = goal_pos[i] - last_goal_pos[i];
    // last_goal_pos[i] = goal_pos[i];

    // float friction_compensation = goal_vel * 0.30385482 + 61.1004804 * (goal_vel > 0 ? 1 : -1);

    float p_out = kp * err + (kp2 - kp) * (err > kp2_err_point ? err - kp2_err_point : 0) + (kp2 - kp) * (err < -kp2_err_point ? err + kp2_err_point : 0);
    i_out[i] += ki * err;
    if (i_out[i] > pidtune_ki_max) i_out[i] = pidtune_ki_max;
    if (i_out[i] < -pidtune_ki_max) i_out[i] = -pidtune_ki_max;
    // debug_signal[i] = 200;
    // if (std::abs(last_vel[i]) > pidtune_ki_clip_thres) {
    //   i_out[i] = i_out[i] * pidtune_ki_clip_coef;
    //   debug_signal[i] = 400;
    // }
    float d_out = kd * (err - last_err[i]);

    out[i] = /*friction_compensation +*/ sticktion_compensation + p_out + i_out[i] + d_out;

    // Serial.print(p_out);
    // Serial.print(",");
    // Serial.print(i_out[i]);
    // Serial.print(",");
    // Serial.print(d_out);
    // Serial.print(",");
    // Serial.print(out[i]);
    // Serial.print(",");
    // Serial.print("  ");

    last_err[i] = err;
  }
  // Serial.println(millis());

  float raw_out[4 * JOINT_NUM];
  for (int i = 0; i < JOINT_NUM; ++i) {
    // float backlash_compensate_feedforward = config.joint_backlash_compensate_feedforward;
    float backlash_compensate_feedforward = 150;
    raw_out[i * 4] = backlash_compensate_feedforward + out[i];
    raw_out[i * 4 + 1] = backlash_compensate_feedforward + -out[i];
    raw_out[i * 4 + 2] = backlash_compensate_feedforward + out[i];
    raw_out[i * 4 + 3] = backlash_compensate_feedforward + -out[i];
  }

  for (int i = 0; i < 4 * JOINT_NUM; ++i) {
    int ready_send = -raw_out[i]; // Max: 1000 as servo document
    if (ready_send > 800) ready_send = 800;
    if (ready_send < -800) ready_send = -800;
    if (ready_send < 0) ready_send = -(ready_send) + 1024;
    sts.writeWord(4 + i, SCSCL_GOAL_TIME_L, ready_send);
  }

  for (int i = 0; i < NONE_JOINT_NUM; ++i) {
    sts.WritePosEx(4 + 4 * JOINT_NUM + i, none_joint_goal_pos[i], config.non_joint_vel_max, config.non_joint_acc);
  }

  // In timer thread, Racing condition with main thread

  // Serial.println();

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(raw_goal_pos[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(goal_pos[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(last_pos[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(debug_signal[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(last_vel[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(out[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < 4 * JOINT_NUM; ++i) {
  //   Serial.print(raw_out[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // Serial.println(millis());
}

void dualMotorSetup() {
#define S_RXD 18
#define S_TXD 19
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  while (!Serial1) delay(1);
  sts.pSerial = &Serial1;
  
  sts.writeWord(15, SMS_STS_TORQUE_LIMIT_L, 500); // Protect servo

  read_pos();

  for (int i = 0; i < JOINT_NUM; ++i) {
    // traj[i].config_.vel_limit = config.joint_vel_max;
    // traj[i].config_.accel_limit = config.joint_acc;
    // traj[i].config_.decel_limit = config.joint_acc;
  }

  // https://github.com/espressif/esp-idf/blob/v5.2.2/examples/system/esp_timer/main/esp_timer_example_main.c
  const esp_timer_create_args_t timer_args = {
    .callback = &timer_callback
  };
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer, TIMER_TIMEOUT_US));
}

void dualMotorUpdatePos(uint16_t pos[]) {
  if (!torque_enabled) {
    setupTorque(1);
  }
  for (int i = 0; i < JOINT_NUM; ++i) {
    raw_goal_pos[i] = pos[i];
    traj[i].planTrapezoidal(pos[i], traj[i].pos_setpoint_, traj[i].vel_setpoint_);
  }

  for (int i = 0; i < NONE_JOINT_NUM; ++i) {
    none_joint_goal_pos[i] = pos[JOINT_NUM + i];
  }
}

void dualMotorReadPos(uint16_t read_pos[]) {
  for (int i = 0; i < JOINT_NUM + NONE_JOINT_NUM; ++i) {
    read_pos[i] = last_pos[i];
  }
}