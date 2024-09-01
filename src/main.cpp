#include <Arduino.h>
#include <SCServo.h>
#include <lwip/sockets.h>
#include <Adafruit_SSD1306.h>
#include "comm.h"
#include "main.h"
#include <FS.h>
#include <LittleFS.h>
#include <trapTraj.hpp>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels, 32 as default.
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void InitScreen() {
// the IIC used to control OLED screen.
// GPIO 21 - S_SDA, GPIO 22 - S_SCL, as default.
#define S_SCL 22
#define S_SDA 21
  Wire.begin(S_SDA, S_SCL);
// SSD1306: 0x3C
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
}

SMS_STS sts;

#define TIMER_TIMEOUT_US 10000
// 舵机内部速度变量的更新频率好像是50Hz

esp_timer_handle_t timer;

#define BACKLASH_COMP_FEEDFORWARD 200

#define SERVO_NUM 4
#define JOINT_NUM 1

int init_pos[SERVO_NUM];

TrapezoidalTrajectory traj[JOINT_NUM];
float last_pos[JOINT_NUM] = { 2048 };
float last_vel[JOINT_NUM] = { 0 };

#define ACC 100
#define MAX_VEL 10000

void read_pos(int pos[]) {
  for (int i = 0; i < SERVO_NUM; ++i) {
    pos[i] = sts.ReadPos(4 + i);
  }
}

void timer_callback(void *arg) {
  int raw_pos[SERVO_NUM];
  for (int i = 0; i < SERVO_NUM; ++i) {
    raw_pos[i] = sts.ReadPos(4 + i);
  }
  
  // Zero offest
  for (int i = 0; i < SERVO_NUM; ++i) {
    raw_pos[i] -= init_pos[i];
    if (raw_pos[i] < 0) raw_pos[i] += 4096;
    // Move middle from 0 to 2048
    raw_pos[i] += 2048;
    if (raw_pos[i] >= 4096) raw_pos[i] -= 4096;
  }
  
  // average load into 4 servo
  float pos[JOINT_NUM];
  for (int i = 0; i < JOINT_NUM; ++i) {
    pos[i] = (raw_pos[i * 4] + (4096 - raw_pos[i * 4 + 1]) + raw_pos[i * 4 + 2] + (4096 - raw_pos[i * 4 + 3])) / 4.0;
  }
  
  float vel[JOINT_NUM];
  for (int i = 0; i < JOINT_NUM; ++i) {
    vel[i] = (pos[i] - last_pos[i]) / TIMER_TIMEOUT_US * 1000000;
    float filter = 0.01;
    vel[i] = vel[i] * filter + last_vel[i] * (1 - filter);
    last_pos[i] = pos[i];
    last_vel[i] = vel[i];
  }

  float goal_pos[JOINT_NUM];
  for (int i = 0; i < JOINT_NUM; ++i) {
    goal_pos[i] = traj[i].update();
  }

  float kp = 20, kd = 0, ki = 0.4;

  static float last_err[JOINT_NUM];
  static float i_out[JOINT_NUM] = {};
  float out[JOINT_NUM] = {};
  for (int i = 0; i < JOINT_NUM; ++i) {
    float err = goal_pos[i] - pos[i];

    float p_out = kp * err;
    i_out[i] += ki * err;
    float d_out = kd * (err - last_err[i]);

    out[i] = p_out + i_out[i] + d_out;

    last_err[i] = err;
  }

  float raw_out[SERVO_NUM];
  for (int i = 0; i < JOINT_NUM; ++i) {
    raw_out[i * 4] = out[i];
    raw_out[i * 4 + 1] = -out[i];
    raw_out[i * 4 + 2] = out[i];
    raw_out[i * 4 + 3] = -out[i];
  }

  for (int i = 0; i < SERVO_NUM; ++i) {
    int ready_send = -(BACKLASH_COMP_FEEDFORWARD + raw_out[i]); // Max: 1000 as servo document
    if (ready_send > 800) ready_send = 800;
    if (ready_send < -800) ready_send = -800;
    if (ready_send < 0) ready_send = -(ready_send) + 1024;
    sts.writeWord(4 + i, SCSCL_GOAL_TIME_L, ready_send);
  }

  for (int i = 0; i < JOINT_NUM; ++i) {
    Serial.print(vel[i]);
    Serial.print(" ");
  }

  for (int i = 0; i < JOINT_NUM; ++i) {
    Serial.print(goal_pos[i]);
    Serial.print(" ");
  }

  for (int i = 0; i < JOINT_NUM; ++i) {
    Serial.print(pos[i]);
    Serial.print(" ");
  }

  for (int i = 0; i < JOINT_NUM; ++i) {
    Serial.print(out[i]);
    Serial.print(" ");
  }

  for (int i = 0; i < SERVO_NUM; ++i) {
    Serial.print(raw_out[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  comm_init();

  InitScreen();

#define S_RXD 18
#define S_TXD 19
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  while (!Serial1) delay(1);
  sts.pSerial = &Serial1;
  
  read_pos(init_pos);

  for (int i = 0; i < SERVO_NUM; ++i) {
    if (!(50 < init_pos[i] && init_pos[i] < 4096 - 50)) {
      Serial.print("Maybe cause wrong init_pos, check power status, or reinstall the servo #"); Serial.print(i); Serial.println();
      while (1) ;
    }
  }
  
  // preload
  for (int i = 0; i < SERVO_NUM; ++i) {
    sts.writeWord(4 + i, SCSCL_GOAL_TIME_L, BACKLASH_COMP_FEEDFORWARD);
  }

  // wait for stable
  sleep(1);
  
  int init_pos1[SERVO_NUM];
  read_pos(init_pos1);

  // inversed preload
  for (int i = 0; i < SERVO_NUM; ++i) {
    sts.writeWord(4 + i, SCSCL_GOAL_TIME_L, BACKLASH_COMP_FEEDFORWARD + 1024); // negative as scs goal time format
  }

  // wait for stable
  sleep(1);
  
  int init_pos2[SERVO_NUM];
  read_pos(init_pos2);

  for (int i = 0; i < SERVO_NUM; ++i) {
    if (std::abs(init_pos1[i] - init_pos2[i]) > 200) {
      Serial.print("Gap is too wide, check power status, or reinstall the servo #"); Serial.print(i); Serial.println();
      while (1) ;
    }

    init_pos[i] = (init_pos1[i] + init_pos2[i]) / 2;
  }

  // https://github.com/espressif/esp-idf/blob/v5.2.2/examples/system/esp_timer/main/esp_timer_example_main.c
  const esp_timer_create_args_t timer_args = {
    .callback = &timer_callback
  };
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer, TIMER_TIMEOUT_US));
}

void loop() {
  comm_type_t type;
  uint8_t buf[20];
  bool ret = comm_recv_poll_last(&type, buf);

  if (!ret) {
    // TODO PING PONG
    if (type == COMM_TYPE_CTRL) {
      uint16_t pos[8];
      memcpy(pos, buf, sizeof pos);
      for (int i = 0; i < 6; ++i) pos[i] = ntohs(pos[i]);

      if (traj[0].trajectory_done_) {
        traj[0].planTrapezoidal(pos[0], last_pos[0], last_vel[0]);
      } else {
        TrapezoidalTrajectory::Step_t traj_step = traj[0].eval(traj[0].t_);
        traj[0].planTrapezoidal(pos[0], traj_step.Y, traj_step.Yd);
      }
    }
  }

  delay(1); // 让出线程
}