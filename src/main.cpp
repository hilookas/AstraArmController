#include <Arduino.h>
#include <SCServo.h>
#include <lwip/sockets.h>
#include <Adafruit_SSD1306.h>
#include "comm.h"
#include "main.h"
#include <FS.h>
#include <LittleFS.h>

config_t config;

void print_config() {
  Serial.print("config version: "); Serial.println(config.version);
  Serial.print("5 servo_vel: "); Serial.println(config.servo_vel);
  Serial.print("6 servo_acc: "); Serial.println(config.servo_acc);
}

void write_config() {
  File file = LittleFS.open("/config.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  if (file.write((uint8_t *)&config, sizeof config) == 0) {
    Serial.println("Write failed");
    file.close();
    return;
  }

  file.close();
  
  Serial.println("Config written");
  print_config();
}

void read_config() {
  File file = LittleFS.open("/config.txt");
  if (!file) {
    Serial.println("Failed to open file for reading");

    Serial.println("Initing config");
    write_config();
    return;
  }

  if (file.isDirectory()) {
    Serial.println("Is dir?");
    file.close();
    return;
  }

  config_t temp_config;
  size_t read_size = file.read((uint8_t *)&temp_config, sizeof temp_config);
  if (read_size != (sizeof temp_config)) {
    Serial.println("Wrong size.");
    file.close();

    Serial.println("Initing config");
    write_config();
    return;
  }

  if (temp_config.version != config.version) {
    Serial.println("Wrong version");
    file.close();

    Serial.println("Initing config");
    write_config();
    return;
  }

  memcpy((uint8_t *)&config, (uint8_t *)&temp_config, sizeof config);

  file.close();

  Serial.println("Config read");
  print_config();
}

int err_cnt[10];

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

void setupTorque(int enable) {
  sts.EnableTorque(12, enable);
  sts.EnableTorque(13, enable);
}

void doPose(uint16_t pos[]) {
  // 舵机编码器值范围 [0,4096) 对应 [0,2*PI) L形态各个关节为中值2048
  int servo_vel = config.servo_vel; // 32766
  int servo_acc = config.servo_acc; // max: 256

  sts.WritePosEx(12, pos[0], servo_vel, servo_acc);
  sts.WritePosEx(13, pos[1], servo_vel, servo_acc);
}

void readPose(uint16_t pos[]) {
  pos[0] = sts.ReadPos(12);
  pos[1] = sts.ReadPos(13);
}

int ctrl_cnt;
int feedback_cnt;
uint32_t last_stat_time;

uint32_t last_action_time;

uint32_t last_ping_time;
uint32_t last_pong_time;

bool have_last_pos_cmd = false;
uint16_t last_pos_cmd[8];

void setup() {
  memset(err_cnt, 0, sizeof err_cnt);

  comm_init();

  InitScreen();

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }

  read_config();

#define S_RXD 18
#define S_TXD 19
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  while (!Serial1) delay(1);
  sts.pSerial = &Serial1;

  setupTorque(0);

  uint16_t pos[8];
  readPose(pos);

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
  uint8_t buf[20];
  bool ret = comm_recv_poll_last(&type, buf);

  if (!ret) {
    // TODO PING PONG
    if (type == COMM_TYPE_PING) {
      comm_send_blocking(COMM_TYPE_PONG, buf);
    } else if (type == COMM_TYPE_PONG) {
      last_pong_time = millis();
    } else if (type == COMM_TYPE_TORQUE) {
      setupTorque(buf[0]);
      if (buf[0] == 0) {
        have_last_pos_cmd = false;
      }
    } else if (type == COMM_TYPE_CTRL) {
      last_action_time = millis();

      uint16_t pos[8];
      memcpy(pos, buf, sizeof pos);
      for (int i = 0; i < 6; ++i) pos[i] = ntohs(pos[i]);

      ++ctrl_cnt;
      doPose(pos);

      have_last_pos_cmd = true;
      memcpy(last_pos_cmd, pos, sizeof pos);

      ++feedback_cnt;
      readPose(pos);

      for (int i = 0; i < 6; ++i) pos[i] = htons(pos[i]);
      comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)pos);
    } else if (type == COMM_TYPE_CONFIG_WRITE) {
      uint32_t cmd[2];
      memcpy(cmd, buf, sizeof cmd);
      for (int i = 0; i < 2; ++i) cmd[i] = ntohl(cmd[i]);

      if (cmd[0] == 0x05) {
        config.servo_vel = (cmd[1]);
        write_config();
        cmd[1] = (config.servo_vel);
      } else if (cmd[0] == 0x06) {
        config.servo_acc = (cmd[1]);
        write_config();
        cmd[1] = (config.servo_acc);
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

      if (cmd[0] == 0x05) {
        cmd[1] = (config.servo_vel);
      } else if (cmd[0] == 0x06) {
        cmd[1] = (config.servo_acc);
      } else {
        Serial.println("Unknown command");
      }

      for (int i = 0; i < 2; ++i) cmd[i] = htonl(cmd[i]);
      memcpy(buf, cmd, sizeof cmd);
      comm_send_blocking(COMM_TYPE_CONFIG_FEEDBACK, (uint8_t *)buf);
    } else {
      ++err_cnt[2];
    }
  }
  uint32_t this_action_time = millis();
  if (this_action_time - last_action_time > 10) { // 每隔 10ms执行一次
    last_action_time = this_action_time;

    // Serial.println("No ctrl");
    
    if (have_last_pos_cmd) {
      doPose(last_pos_cmd);
    }

    uint16_t pos[8];
    ++feedback_cnt;
    readPose(pos);

    for (int i = 0; i < 6; ++i) pos[i] = htons(pos[i]);
    comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)pos);

    // follower 已经10ms没有收到信号了
    ++err_cnt[2];
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
    display.print(millis() / 1000); display.print(F("s ")); display.print(ctrl_freq); display.print(F("Hz ")); display.print(err_cnt[0]); display.print(F(" ")); display.print(err_cnt[1]); display.print(F(" ")); display.print(err_cnt[2]); display.println();

    display.display();

    last_stat_time = this_stat_time;
  }

  delay(1); // 让出线程
}