#include <Arduino.h>
#include <SCServo.h>
#include <lwip/sockets.h>
#include <Adafruit_SSD1306.h>

#define AS_LEADER // lookas

int err_cnt[10];

// FUCK YOU ARDUINO https://arduino.stackexchange.com/questions/66530/class-enum-was-not-declared-in-this-scope
typedef enum {
  COMM_TYPE_PING,
  COMM_TYPE_PONG,
  COMM_TYPE_CTRL,
  COMM_TYPE_FEEDBACK,
  COMM_TYPE_TORQUE,
} comm_type_t;

// 初始化串口
// 返回是否发生错误
bool serial_init(void) {
  Serial.begin(921600);
  while (!Serial) delay(1);
  return false;
}

// 堵塞输出一个字符
// c 为要发送的字符
// 返回是否发生错误
bool serial_send_blocking(uint8_t c) {
  Serial.write(c);
  return false;
}

// 非堵塞轮询一个字符
// c 为要接受的字符
// 返回是否发生错误
bool serial_recv_poll(uint8_t *c) {
  // 线程不安全！！！
  if (Serial.available()) {
    *c = Serial.read();
    return false;
  } else {
    return true;
  }
}

#define COMM_PAYLOAD_SIZE_MAX 100

// typedef enum {
//   COMM_TYPE_PING,
//   COMM_TYPE_PONG,
//   COMM_TYPE_CTRL,
//   COMM_TYPE_FEEDBACK,
//   COMM_TYPE_TORQUE,
// } comm_type_t;

// 不同类型的数据包大小
int comm_payload_size[] = {
  14, // COMM_TYPE_PING
  14, // COMM_TYPE_PONG
  14, // COMM_TYPE_CTRL
  14, // COMM_TYPE_FEEDBACK
  14, // COMM_TYPE_TORQUE
};

// 不能被忽略的数据包
// 置为 true 会导致 comm_recv_poll_last 不跳过这个数据包
// 但是将其置为 true 并不会
bool comm_type_importance[] = {
  false, // COMM_TYPE_PING
  false, // COMM_TYPE_PONG
  false, // COMM_TYPE_CTRL
  false, // COMM_TYPE_FEEDBACK
  true,  // COMM_TYPE_TORQUE
};

// 堵塞发送一个数据包
// type 传递数据类型，payload 传递实际数据
// 返回是否发生错误
bool comm_send_blocking(comm_type_t type, const uint8_t payload[]) {
  bool ret;
  ret = serial_send_blocking(0x5A); // 0b01011010 as header
  if (ret) return true;
  ret = serial_send_blocking((uint8_t)type); // 0b01011010 as header
  if (ret) return true;
  for (int i = 0; i < comm_payload_size[type]; ++i) {
    ret = serial_send_blocking(payload[i]);
    if (ret) return true;
  }
  return false;
}

static uint8_t recv_buf[2 + COMM_PAYLOAD_SIZE_MAX];
static int recv_buf_p;

// 尝试接收一个数据包
// 如果没有发生错误，则 *type 内为接收到的数据包类型，payload 为接收到的实际数据
// 返回是否发生错误
// 这个函数发生错误十分正常，在没有接收到数据的时候，就会返回错误（不阻塞）
bool comm_recv_poll(comm_type_t *type, uint8_t payload[]) {
  bool ret;
  while (true) {
    uint8_t buf;
    ret = serial_recv_poll(&buf);
    if (ret) return true; // 没有新数据

    // 无效数据
    if (recv_buf_p == 0 && buf != 0x5A) {
      err_cnt[0] += 1;
      // fprintf(stderr, "comm: warning: Received wrong byte!");
      continue;
    }
    recv_buf[recv_buf_p++] = buf;

    // 缓冲区太小存不下完整数据包
    assert(recv_buf_p <= sizeof recv_buf);

    // 收到未知类型的数据包
    // assert(recv_buf_p != 2 || recv_buf[1] < (sizeof comm_payload_size) / (sizeof comm_payload_size[0]));
    if (recv_buf_p == 2 && recv_buf[1] >= (sizeof comm_payload_size) / (sizeof comm_payload_size[0])) {
      err_cnt[1] += 1;
      // fprintf(stderr, "comm: warning: Received wrong type!");
      recv_buf_p = 0; // 重置状态
      continue;
    }

    // 数据包到结尾了
    if (recv_buf_p >= 2 && recv_buf_p == 2 + comm_payload_size[recv_buf[1]]) {
      // 复制数据输出
      *type = (comm_type_t)recv_buf[1];
      memcpy(payload, recv_buf + 2, comm_payload_size[recv_buf[1]]);
      // 清空缓冲区
      recv_buf_p = 0;
      break;
    }
    // TODO 增加超时机制
  }
  return false;
}

// 尽量清空读缓冲区（同步两台设备的周期），并且返回最后一组数据
// 如果没有发生错误，则 *type 内为接收到的数据包类型，payload 为接收到的实际数据
// 返回是否发生错误
// 这个函数发生错误十分正常，在没有接收到数据的时候，就会返回错误（不阻塞）
// 注意：这个函数会导致部分回传的数据包丢包（如果处理的节奏跟不上的话）
// 但是必要的丢包是值得的，否则缓冲区会被堆积，实时性无法得到保证
bool comm_recv_poll_last(comm_type_t *type, uint8_t payload[]) {
  bool last_ret = true; // 默认为没有数据（发生错误）
  
  bool ret = comm_recv_poll(type, payload);
  while (!ret) {
    if (comm_type_importance[*type]) { // 不能被忽略的数据包
      return ret;
    }
    last_ret = ret;
    ret = comm_recv_poll(type, payload);
  }
  return ret = last_ret;
}

// 返回是否发生错误
bool comm_init(void) {
  bool ret = serial_init();
  if (ret) return true; // 没有新数据
  recv_buf_p = 0;
  return false;
}

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
const int EIShaperBufLen = 128;
uint16_t EIShaperBuf[EIShaperBufLen][7];
int EIShaperCur;

int EIShaperT2, EIShaperT3;
float EIShaperT2_1, EIShaperT2_2, EIShaperT3_1, EIShaperT3_2;
float EIShaperA1, EIShaperA2, EIShaperA3;

void EIShaperInit() {
  float freq = 5.5; // 要消除的振动频率 ESSENTIAL!
  float omega = 2 * M_PI * freq;
  float zeta = 0.01; // useless

  // 计算EI整形器需要的各个延时环节系数，以及增益
  float t1 = 0;
  float t2 = M_PI / omega;
  float t3 = 2*t2;
  float V = 0.05; // 抑制参数 ESSENTIAL!
  float A1 = (1+V)/4;
  float A2 = (1-V)/2;
  float A3 = (1+V)/4;

  // 离散化处理
  // ctrl freq 计算公式
  // 220*(9*((7+7+6)*8+250) + 16*8*1000000/115200)
  float ctrl_freq = 48; // 控制频率 ESSENTIAL!
  float ctrl_period_T = 1 / ctrl_freq; // ~5ms
  float t2_discrete = t2 / ctrl_period_T;
  float t3_discrete = t3 / ctrl_period_T;
  assert(t3_discrete + 1 < EIShaperBufLen);

  // 保存到全局变量
  EIShaperT2 = t2_discrete; // float2int
  EIShaperT3 = t3_discrete; // float2int
  // 防止几个时间太接近影响效果
  assert(EIShaperT2 != 0);
  assert(EIShaperT2 != EIShaperT3);
  // 插值用
  EIShaperT2_1 = t2_discrete - EIShaperT2;
  EIShaperT2_2 = 1 - EIShaperT2_1;
  EIShaperT3_1 = t3_discrete - EIShaperT3;
  EIShaperT3_2 = 1 - EIShaperT3_1;
  assert(0 <= EIShaperT2_1 && EIShaperT2_1 <= 1);
  assert(0 <= EIShaperT2_2 && EIShaperT2_2 <= 1);
  assert(0 <= EIShaperT3_1 && EIShaperT3_1 <= 1);
  assert(0 <= EIShaperT3_2 && EIShaperT3_2 <= 1);
  EIShaperA1 = A1;
  EIShaperA2 = A2;
  EIShaperA3 = A3;

  // 初始化 buffer
  uint16_t pos[7];
  readPose(pos);
  for (int i = 0; i < EIShaperBufLen; ++i) {
    memcpy(EIShaperBuf[i], pos, sizeof pos);
  }
  
  // Serial.printf("EI Shaper!\n");
  // Serial.printf("t1: %f, t2: %f, t3: %f\n", t1, t2, t3);
  // Serial.printf("A1: %f, A2: %f, A3: %f\n", A1, A2, A3);
  // Serial.printf("t2_discrete: %f, t3_discrete: %f\n", t2_discrete, t3_discrete);
}

void EIShaperApply(uint16_t pos[]) {
  ++EIShaperCur;
  EIShaperCur %= EIShaperBufLen;
  // memcpy(EIShaperBuf[EIShaperCur], pos, sizeof pos); // 错误的，pos此时是指针，不是数组，所以sizeof结果不是14而是4
  memcpy(EIShaperBuf[EIShaperCur], pos, 2 * 7); // 先存一下原始数据

  int t2 = (EIShaperCur - EIShaperT2 + EIShaperBufLen) % EIShaperBufLen;
  int t3 = (EIShaperCur - EIShaperT3 + EIShaperBufLen) % EIShaperBufLen;
  int t2_2 = (t2 - 1 + EIShaperBufLen) % EIShaperBufLen;
  int t3_2 = (t3 - 1 + EIShaperBufLen) % EIShaperBufLen;

  for (int i = 0; i < 7; ++i) {
    pos[i] = pos[i] * EIShaperA1
      + (EIShaperBuf[t2][i] * EIShaperT2_2 + EIShaperBuf[t2_2][i] * EIShaperT2_1) * EIShaperA2
      + (EIShaperBuf[t3][i] * EIShaperT3_2 + EIShaperBuf[t3_2][i] * EIShaperT3_1) * EIShaperA3;
  }
}
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

  EIShaperInit();
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
  EIShaperApply(pos);

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