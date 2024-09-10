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
    float filter = 0.01;
    vel[i] = vel[i] * filter + last_vel[i] * (1 - filter);
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

void timer_callback(void *arg) {
  if (updateSetupTorque) {
    updateSetupTorque = false;
    doSetupTorque(setupTorque_enable);
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

  float kp = 10, kd = 0, ki = 0.4;

  static float last_err[JOINT_NUM];
  static float i_out[JOINT_NUM] = {};
  float out[JOINT_NUM] = {};
  for (int i = 0; i < JOINT_NUM; ++i) {
    float err = goal_pos[i] - last_pos[i];

    if (std::abs(last_vel[i]) < 1 && std::abs(err) < 3) { // better positioning
      kp = 20;
    } else if (std::abs(last_vel[i]) < 1 && std::abs(err) < 5) {
      kp = 20;
    } else if (std::abs(last_vel[i]) < 3 && std::abs(err) < 10) {
      kp = 20;
    } else {
      kp = 10;
    }

    if (std::abs(last_vel[i]) < 3 && std::abs(err) < 10) { // better positioning
      ki = 2;
    } else {
      ki = 0;
      i_out[i] = 0;
    }

    float p_out = kp * err;
    i_out[i] += ki * err;
    float d_out = kd * (err - last_err[i]);

    float sticktion_compensation = 0;
    if (err > 0) {
      sticktion_compensation = 80;
    } else {
      sticktion_compensation = -80;
    }

    out[i] = sticktion_compensation + p_out + i_out[i] + d_out;

    last_err[i] = err;
  }

  float raw_out[4 * JOINT_NUM];
  for (int i = 0; i < JOINT_NUM; ++i) {
    raw_out[i * 4] = out[i];
    raw_out[i * 4 + 1] = -out[i];
    raw_out[i * 4 + 2] = out[i];
    raw_out[i * 4 + 3] = -out[i];
  }

  for (int i = 0; i < 4 * JOINT_NUM; ++i) {
    int ready_send = -(config.joint_backlash_compensate_feedforward + raw_out[i]); // Max: 1000 as servo document
    if (ready_send > 800) ready_send = 800;
    if (ready_send < -800) ready_send = -800;
    if (ready_send < 0) ready_send = -(ready_send) + 1024;
    sts.writeWord(4 + i, SCSCL_GOAL_TIME_L, ready_send);
  }

  for (int i = 0; i < NONE_JOINT_NUM; ++i) {
    sts.WritePosEx(4 + 4 * JOINT_NUM + i, none_joint_goal_pos[i], config.non_joint_vel_max, config.non_joint_acc);
  }

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(goal_pos[i]);
  //   Serial.print(" ");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(last_pos[i]);
  //   Serial.print(" ");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(last_vel[i]);
  //   Serial.print(" ");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(out[i]);
  //   Serial.print(" ");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < 4 * JOINT_NUM; ++i) {
  //   Serial.print(raw_out[i]);
  //   Serial.print(" ");
  // }
  // Serial.print("  ");

  // Serial.println();
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
    traj[i].Xf_ = last_pos[i];
    traj[i].pos_setpoint_ = last_pos[i];
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