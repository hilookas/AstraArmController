#include <Arduino.h>
#include <SCServo.h>
#include <lwip/sockets.h>
#include <Adafruit_SSD1306.h>
#include "comm.h"
#include "main.h"
#include <FS.h>
#include <LittleFS.h>

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

#define TIMER_TIMEOUT_US 8000

esp_timer_handle_t timer;

#define BACKLASH_COMP_FEEDFORWARD 200

#define SERVO_NUM 8

int init_pos[SERVO_NUM];
int goal_pos[SERVO_NUM] = { 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048,  };
// int goal_pos[SERVO_NUM] = { 2048, 2048, 2048, 2048, };

void read_pos(int pos[]) {
  for (int i = 0; i < SERVO_NUM; ++i) {
    pos[i] = sts.ReadPos(4 + i);
  }
}

void timer_callback(void *arg) {
  int pos[SERVO_NUM];
  read_pos(pos);
  
  for (int i = 0; i < SERVO_NUM; ++i) {
    pos[i] -= init_pos[i];
    if (pos[i] < 0) pos[i] += 4096;
    // Move middle from 0 to 2048
    pos[i] += 2048;
    if (pos[i] >= 4096) pos[i] -= 4096;
  }
  
  // // get real pos based on the average of two servo
  // for (int i = 0; i < SERVO_NUM / 2; ++i) {
  //   pos[i * 2] = (pos[i * 2] + 4096 - pos[i * 2 + 1]) / 2;
  //   pos[i * 2 + 1] = 4096 - pos[i * 2];
  // }
  
  // average load into 4 servo
  for (int i = 0; i < SERVO_NUM / 4; ++i) {
    pos[i * 4] = (pos[i * 4] + 4096 - pos[i * 4 + 1] + pos[i * 4 + 2] + 4096 - pos[i * 4 + 3]) / 4;
    pos[i * 4 + 1] = 4096 - pos[i * 4];
    pos[i * 4 + 2] = pos[i * 4];
    pos[i * 4 + 3] = 4096 - pos[i * 4];
  }

  int err[SERVO_NUM];
  for (int i = 0; i < SERVO_NUM; ++i) {
    err[i] = goal_pos[i] - pos[i];
  }

  int stiction_compensate = 0;

  int kp = 80; // 80 for accurate positioning
  for (int i = 0; i < SERVO_NUM; ++i) {
    int ready_send = -(BACKLASH_COMP_FEEDFORWARD + kp * err[i] + stiction_compensate * (err[i] > 0 ? 1 : -1));
    if (ready_send > 800) ready_send = 800;
    if (ready_send < -800) ready_send = -800;
    if (ready_send < 0) ready_send = -(ready_send) + 1024;
    sts.writeWord(4 + i, SCSCL_GOAL_TIME_L, ready_send);
  }

  for (int i = 0; i < SERVO_NUM; ++i) {
    Serial.print(err[i]);
    Serial.print(" ");
  }

  for (int i = 0; i < SERVO_NUM; ++i) {
    Serial.print(BACKLASH_COMP_FEEDFORWARD + kp * err[i] + stiction_compensate * (err[i] > 0 ? 1 : -1));
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
  sleep(2);
  
  int init_pos1[SERVO_NUM];
  read_pos(init_pos1);

  // inversed preload
  for (int i = 0; i < SERVO_NUM; ++i) {
    sts.writeWord(4 + i, SCSCL_GOAL_TIME_L, BACKLASH_COMP_FEEDFORWARD + 1024); // negative as scs goal time format
  }

  // wait for stable
  sleep(2);
  
  int init_pos2[SERVO_NUM];
  read_pos(init_pos2);

  // inversed preload
  for (int i = 0; i < SERVO_NUM; ++i) {
    sts.writeWord(4 + i, SCSCL_GOAL_TIME_L, 0); // negative as scs goal time format
  }

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

      goal_pos[0] = pos[0];
      goal_pos[1] = 4096 - pos[0];
      goal_pos[2] = pos[0];
      goal_pos[3] = 4096 - pos[0];
      goal_pos[4] = pos[1];
      goal_pos[5] = 4096 - pos[1];
      goal_pos[6] = pos[1];
      goal_pos[7] = 4096 - pos[1];
    }
  }

  delay(1); // 让出线程
}