#include <Arduino.h>
#include <SCServo.h>
#include <lwip/sockets.h>
#include <Adafruit_SSD1306.h>
#include "common.h"
#include "EIShaper.h"
#include "comm.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels, 32 as default.
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void InitScreen(){
// the IIC used to control OLED screen.
// GPIO 21 - S_SDA, GPIO 22 - S_SCL, as default.
#define S_SCL 22
#define S_SDA 21
  Wire.begin(S_SDA, S_SCL);
// SSD1306: 0x3C
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
}

SMS_STS sts;

#ifndef AS_LEADER
#endif

int ctrl_cnt;
uint32_t last_stat_time;

void setupTorque(int enable) {
#ifdef AS_LEADER
  sts.EnableTorque(11, enable);
  sts.EnableTorque(12, enable);
  sts.EnableTorque(13, enable);
  sts.EnableTorque(14, enable);
  sts.EnableTorque(15, enable);
  sts.EnableTorque(16, enable);
  sts.EnableTorque(17, enable);
  sts.EnableTorque(18, enable);
  if (enable != 1) {
    sts.EnableTorque(19, enable);
  }
#else
  sts.EnableTorque(11, enable);
  sts.EnableTorque(7, enable);
  sts.EnableTorque(8, enable);
  sts.EnableTorque(5, enable);
  sts.EnableTorque(6, enable);
  sts.EnableTorque(14, enable);
  sts.EnableTorque(15, enable);
  sts.EnableTorque(16, enable);
  sts.EnableTorque(17, enable);
  sts.EnableTorque(18, enable);
  sts.EnableTorque(19, enable);
#endif
}

void setup() {
  memset(err_cnt, 0, sizeof err_cnt);

  comm_init();

  InitScreen();

#define S_RXD 18
#define S_TXD 19
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  while (!Serial1) delay(1);
  sts.pSerial = &Serial1;
  
  setupTorque(0);

#ifdef AS_LEADER
  sts.writeByte(19, SMS_STS_MODE, 0); // 位置
#else
  sts.writeWord(19, SMS_STS_TORQUE_LIMIT_L, 500); // 保护舵机

  // uint16_t pos[7];
  // readPose(pos);
  // EIShaperInit(pos);
#endif
  
  ctrl_cnt = 0;
  last_stat_time = millis();
}

void doPose(uint16_t pos[], int servo_speed = 32766) {
  // 舵机编码器值范围 [0,4096) 对应 [0,2*PI) L形态各个关节为中值2048
  int servo_acc = 254;
#ifdef AS_LEADER
  sts.WritePosEx(11, pos[0], servo_speed, servo_acc);
  sts.WritePosEx(12, pos[1], servo_speed, servo_acc);
  sts.WritePosEx(13, 2048 - ((int)pos[1] - 2048), servo_speed, servo_acc);
  sts.WritePosEx(14, pos[2], servo_speed, servo_acc);
  sts.WritePosEx(15, 2048 - ((int)pos[2] - 2048), servo_speed, servo_acc);
  sts.WritePosEx(16, pos[3], servo_speed, servo_acc);
  sts.WritePosEx(17, pos[4], servo_speed, servo_acc);
  sts.WritePosEx(18, pos[5], servo_speed, servo_acc);
  // sts.WritePosEx(19, pos[6], servo_speed, servo_acc);
#else
  sts.WritePosEx(11, pos[0], servo_speed, servo_acc);
  sts.WritePosEx(7, pos[1], servo_speed, servo_acc);
  sts.WritePosEx(8, 2048 - ((int)pos[1] - 2048), servo_speed, servo_acc);
  sts.WritePosEx(5, pos[1], servo_speed, servo_acc);
  sts.WritePosEx(6, 2048 - ((int)pos[1] - 2048), servo_speed, servo_acc);
  sts.WritePosEx(14, pos[2], servo_speed, servo_acc);
  sts.WritePosEx(15, 2048 - ((int)pos[2] - 2048), servo_speed, servo_acc);
  sts.WritePosEx(16, pos[3], servo_speed, servo_acc);
  sts.WritePosEx(17, pos[4], servo_speed, servo_acc);
  sts.WritePosEx(18, pos[5], servo_speed, servo_acc);
  sts.WritePosEx(19, pos[6], servo_speed, servo_acc);
#endif
}

void readPose(uint16_t pos[]) {
#ifdef AS_LEADER
  pos[0] = sts.ReadPos(11);
  pos[1] = sts.ReadPos(12);
  pos[2] = sts.ReadPos(14);
  pos[3] = sts.ReadPos(16);
  pos[4] = sts.ReadPos(17);
  pos[5] = sts.ReadPos(18);
  pos[6] = sts.ReadPos(19);
#else
  pos[0] = sts.ReadPos(11);
  pos[1] = sts.ReadPos(7);
  pos[2] = sts.ReadPos(14);
  pos[3] = sts.ReadPos(16);
  pos[4] = sts.ReadPos(17);
  pos[5] = sts.ReadPos(18);
  pos[6] = sts.ReadPos(19);
#endif
}

void loop() {
#ifdef AS_LEADER
  comm_type_t type;
  uint8_t buf[20];
  bool ret = comm_recv_poll_last(&type, buf);

  if (!ret) {
    // TODO PING PONG
    if (type == COMM_TYPE_TORQUE) {
      setupTorque(buf[0]);
    } else {
      ++err_cnt[2];
    }
  }

  uint16_t pos[7];
  readPose(pos);

  for (int i = 0; i < 7; ++i) pos[i] = htons(pos[i]);

  comm_send_blocking(COMM_TYPE_CTRL, (uint8_t *)pos);

  delay(10); // 确保不超过follower 240Hz处理上限（由通信频率限制）
  // delay(3): 190Hz
  // delay(10): 81Hz

  ++ctrl_cnt;
  uint32_t this_time = millis();
  if (this_time - last_stat_time > 1000) {
    float ctrl_freq = (float)ctrl_cnt / (this_time - last_stat_time) * 1000;

    // Update Display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    // Row1
    display.print(millis() / 1000); display.print(F("s ")); display.print(ctrl_freq); display.print(F("Hz ")); display.print(err_cnt[0]); display.print(F(" ")); display.print(err_cnt[1]); display.print(F(" ")); display.print(err_cnt[2]); display.println();
    display.display();

    ctrl_cnt = 0;
    last_stat_time = this_time;
  }
#else
  comm_type_t type;
  uint16_t pos[7];
  bool ret = comm_recv_poll_last(&type, (uint8_t *)pos);
  // TODO PING PONG and TORQUE
  if (ret) {
    delay(1);
    return;
  }

  if (type != COMM_TYPE_CTRL) {
    ++err_cnt[2];
    return;
  }
  
  for (int i = 0; i < 7; ++i) pos[i] = ntohs(pos[i]);

  // EIShaper
  // EIShaperApply(pos);

  doPose(pos);

  readPose(pos); // 回传qpos // 会极大降低速度

  for (int i = 0; i < 7; ++i) pos[i] = htons(pos[i]);

  comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)pos);

  ++ctrl_cnt;
  uint32_t this_time = millis();
  if (this_time - last_stat_time > 1000) {
    float ctrl_freq = (float)ctrl_cnt / (this_time - last_stat_time) * 1000;
    // Serial.printf("crtl freq: %.2fHz\n", ctrl_freq);

    int servos[] = { 11, 7, 8, 5, 6, 14, 15, 16, 17, 18, 19 };
    //                0  1  2  3  4   5   6   7   8   9  10
    const int servos_cnt = 11;
    float voltage_all = 0;
    float current_all = 0;
    float currents[servos_cnt];
    for (int i = 0; i < servos_cnt; ++i) {
      assert(sts.FeedBack(servos[i]) != -1);
      float pos = (float)sts.ReadPos(-1) / 4096 * 2 * M_PI;
      float vel = sts.ReadSpeed(-1) / 4096 * 2 * M_PI;
      float load = sts.ReadLoad(-1) * 0.1;
      float current = sts.ReadCurrent(-1) * 6.5;
      current_all += current;
      currents[i] = current;
      float voltage = sts.ReadVoltage(-1) * 0.1;
      voltage_all = voltage;
      float temp = sts.ReadTemper(-1);
      // Serial.printf("STS %d: pos %frad, vel %frad/s, load %f%, current %fmA, voltage %fV, temp %f\n", servos[i], pos, vel, load, current, voltage, temp);
    }
    // Serial.printf("voltage: %f, current_all: %f\n", voltage_all, current_all);

    // Update Display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    // Row1
    display.print(millis() / 1000); display.print(F("s ")); display.print(ctrl_freq); display.print(F("Hz ")); display.print(err_cnt[0]); display.print(F(" ")); display.print(err_cnt[1]); display.print(F(" ")); display.print(err_cnt[2]); display.println();
    // Row2
    display.print(voltage_all); display.print(F("V ")); display.print(current_all); display.print(F("mA")); display.println();
    // Row3,4
    display.print((int)currents[1]); display.print(F("+")); display.print((int)currents[2]); display.print(F("+")); display.print((int)currents[3]); display.print(F("+")); display.print((int)currents[4]); display.print(F(" ")); display.print((int)currents[5]); display.print(F("+")); display.print((int)currents[6]); display.println();
    display.print((int)currents[0]); display.print(F(" ")); display.print((int)currents[7]); display.print(F(" ")); display.print((int)currents[8]); display.print(F(" ")); display.print((int)currents[9]); display.println(); 
    display.display();

    ctrl_cnt = 0;
    last_stat_time = this_time;
  }
#endif
}