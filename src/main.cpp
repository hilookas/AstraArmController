#include <Arduino.h>
#include <SCServo.h>
#include <lwip/sockets.h>
#include <Adafruit_SSD1306.h>
#include "EIShaper.h"
#include "comm.h"
#include "main.h"
#include "dualMotor.h"
#include "config.h"

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

int ctrl_cnt;
int feedback_cnt;
uint32_t last_stat_time;

uint32_t last_action_time;

uint32_t last_ping_time;
uint32_t last_pong_time;

void setup() {
  comm_init();

  InitScreen();

  configInit();
  
  dualMotorSetup();

  setupTorque(0);

  uint16_t pos[8];
  dualMotorReadPos(pos);
  if (config.EIShaper_enabled) {
    EIShaperInit(pos);
  }

  ctrl_cnt = 0;
  feedback_cnt = 0;
  last_stat_time = millis();

  last_action_time = millis();

  last_ping_time = 0;
  last_pong_time = 0;
}

#define FLOAT_TO_U32(x) (*(uint32_t *)&(x))
#define U32_TO_FLOAT(x) (*(float *)&(x))

void loop() {
  comm_type_t type;
  uint8_t buf[COMM_PAYLOAD_SIZE_MAX];
  bool ret = comm_recv_poll_last(&type, buf);

  if (!ret) {
    // TODO PING PONG
    if (type == COMM_TYPE_PING) {
      comm_send_blocking(COMM_TYPE_PONG, buf);
    } else if (type == COMM_TYPE_PONG) {
      last_pong_time = millis();
    } else if (type == COMM_TYPE_TORQUE) {
      setupTorque(buf[0]);
    } else if (type == COMM_TYPE_CTRL) {
      last_action_time = millis();

      uint16_t pos[8];
      memcpy(pos, buf, sizeof pos);
      for (int i = 0; i < 6; ++i) pos[i] = ntohs(pos[i]);

      // EIShaper
      if (config.EIShaper_enabled) {
        EIShaperApply(pos);
      }

      ++ctrl_cnt;
      dualMotorUpdatePos(pos);

      ++feedback_cnt;
      dualMotorReadPos(pos);

      for (int i = 0; i < 6; ++i) pos[i] = htons(pos[i]);
      comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)pos);
    } else if (type == COMM_TYPE_CONFIG_WRITE) {
      uint32_t cmd[2];
      memcpy(cmd, buf, sizeof cmd);
      for (int i = 0; i < 2; ++i) cmd[i] = ntohl(cmd[i]);

      if (cmd[0] == 0x01) {
        config.EIShaper_enabled = (bool)cmd[1];
        write_config();
        uint16_t pos[8];
        dualMotorReadPos(pos);
        if (config.EIShaper_enabled) {
          EIShaperInit(pos);
        }
        cmd[1] = (uint32_t)config.EIShaper_enabled;
      } else if (cmd[0] == 0x02) {
        config.EIShaper_freq = U32_TO_FLOAT(cmd[1]);
        write_config();
        uint16_t pos[8];
        dualMotorReadPos(pos);
        if (config.EIShaper_enabled) {
          EIShaperInit(pos);
        }
        cmd[1] = FLOAT_TO_U32(config.EIShaper_freq);
      } else if (cmd[0] == 0x03) {
        config.EIShaper_V = U32_TO_FLOAT(cmd[1]);
        write_config();
        uint16_t pos[8];
        dualMotorReadPos(pos);
        if (config.EIShaper_enabled) {
          EIShaperInit(pos);
        }
        cmd[1] = FLOAT_TO_U32(config.EIShaper_V);
      } else if (cmd[0] == 0x04) {
        config.EIShaper_ctrl_freq = U32_TO_FLOAT(cmd[1]);
        write_config();
        uint16_t pos[8];
        dualMotorReadPos(pos);
        if (config.EIShaper_enabled) {
          EIShaperInit(pos);
        }
        cmd[1] = FLOAT_TO_U32(config.EIShaper_ctrl_freq);;
      } else if (cmd[0] == 0x05) {
        config.joint_vel_max = U32_TO_FLOAT(cmd[1]);
        write_config();
        cmd[1] = FLOAT_TO_U32(config.joint_vel_max);
      } else if (cmd[0] == 0x06) {
        config.joint_acc = U32_TO_FLOAT(cmd[1]);
        write_config();
        cmd[1] = FLOAT_TO_U32(config.joint_acc);
      } else if (cmd[0] == 0x07) {
        config.joint_backlash_compensate_feedforward = (cmd[1]);
        write_config();
        cmd[1] = (config.joint_backlash_compensate_feedforward);
      } else if (cmd[0] == 0x08) {
        config.non_joint_vel_max = (cmd[1]);
        write_config();
        cmd[1] = (config.non_joint_vel_max);
      } else if (cmd[0] == 0x09) {
        config.non_joint_acc = (cmd[1]);
        write_config();
        cmd[1] = (config.non_joint_acc);
      } else {
        Serial.println("Unknown command");
      }

      for (int i = 0; i < 2; ++i) cmd[i] = htonl(cmd[i]);
      memcpy(buf, cmd, sizeof cmd);
      comm_send_blocking(COMM_TYPE_CONFIG_FEEDBACK, (uint8_t *)buf);
    } else if (type == COMM_TYPE_CONFIG_READ) {
      uint32_t cmd[2];
      memcpy(cmd, buf, sizeof cmd);
      for (int i = 0; i < 2; ++i) cmd[i] = ntohl(cmd[i]);

      if (cmd[0] == 0x01) {
        cmd[1] = (uint32_t)config.EIShaper_enabled;
      } else if (cmd[0] == 0x02) {
        cmd[1] = FLOAT_TO_U32(config.EIShaper_freq);
      } else if (cmd[0] == 0x03) {
        cmd[1] = FLOAT_TO_U32(config.EIShaper_V);
      } else if (cmd[0] == 0x04) {
        cmd[1] = FLOAT_TO_U32(config.EIShaper_ctrl_freq);
      } else if (cmd[0] == 0x05) {
        cmd[1] = FLOAT_TO_U32(config.joint_vel_max);
      } else if (cmd[0] == 0x06) {
        cmd[1] = FLOAT_TO_U32(config.joint_acc);
      } else if (cmd[0] == 0x07) {
        cmd[1] = (config.joint_backlash_compensate_feedforward);
      } else if (cmd[0] == 0x08) {
        cmd[1] = (config.non_joint_vel_max);
      } else if (cmd[0] == 0x09) {
        cmd[1] = (config.non_joint_acc);
      } else {
        Serial.println("Unknown command");
      }

      for (int i = 0; i < 2; ++i) cmd[i] = htonl(cmd[i]);
      memcpy(buf, cmd, sizeof cmd);
      comm_send_blocking(COMM_TYPE_CONFIG_FEEDBACK, (uint8_t *)buf);
    } else if (type == COMM_TYPE_PIDTUNE) {
      uint32_t cmd[8];
      memcpy(cmd, buf, sizeof cmd);
      for (int i = 0; i < 8; ++i) cmd[i] = ntohl(cmd[i]);
      Serial.printf("PIDTUNE kp=%.2f ki=%.2f kd=%.2f\n", U32_TO_FLOAT(cmd[0]), U32_TO_FLOAT(cmd[1]), U32_TO_FLOAT(cmd[2]));
      pidtune_kp = U32_TO_FLOAT(cmd[0]);
      pidtune_ki = U32_TO_FLOAT(cmd[1]);
      pidtune_kd = U32_TO_FLOAT(cmd[2]);
      pidtune_ki_clip_thres = U32_TO_FLOAT(cmd[3]);
      pidtune_ki_clip_coef = U32_TO_FLOAT(cmd[4]);
    } else {
      Serial.println("Unknown comm type");
    }
  }

  uint32_t this_action_time = millis();
  if (this_action_time - last_action_time > 10) { // 每隔 10ms执行一次
    last_action_time = this_action_time;

    // Serial.println("No ctrl");

    uint16_t pos[8];
    ++feedback_cnt;
    dualMotorReadPos(pos);

    for (int i = 0; i < 6; ++i) pos[i] = htons(pos[i]);
    comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)pos);

    // follower 已经10ms没有收到信号了
    ;
  }

  uint32_t this_stat_time = millis();
  if (this_stat_time - last_stat_time > 1000) {
    // Update Display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);

    float ctrl_freq = (float)ctrl_cnt / (this_stat_time - last_stat_time) * 1000;
    float feeback_freq = (float)feedback_cnt / (this_stat_time - last_stat_time) * 1000;
    ctrl_cnt = 0;
    feedback_cnt = 0;
    // Row1
    display.print(millis() / 1000); display.print(F("s ")); display.print(ctrl_freq); display.print(F("Hz ")); display.println();

    display.display();

    last_stat_time = this_stat_time;
  }

  delay(1); // 让出线程
}