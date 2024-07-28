#include <Arduino.h>
#include <main.h>

const int EIShaperBufLen = 128;
uint16_t EIShaperBuf[EIShaperBufLen][7];
int EIShaperCur;

int EIShaperT2, EIShaperT3;
float EIShaperT2_1, EIShaperT2_2, EIShaperT3_1, EIShaperT3_2;
float EIShaperA1, EIShaperA2, EIShaperA3;

void EIShaperInit(uint16_t pos[]) {
  float freq = config.EIShaper_freq; // 要消除的振动频率 ESSENTIAL!
  float omega = 2 * M_PI * freq;
  float zeta = 0.01; // useless

  // 计算EI整形器需要的各个延时环节系数，以及增益
  float t1 = 0;
  float t2 = M_PI / omega;
  float t3 = 2*t2;
  float V = config.EIShaper_V; // 抑制参数 ESSENTIAL!
  float A1 = (1+V)/4;
  float A2 = (1-V)/2;
  float A3 = (1+V)/4;

  // 离散化处理
  // ctrl freq 计算公式
  // 220*(9*((7+7+6)*8+250) + 16*8*1000000/115200)
  float ctrl_freq = config.EIShaper_ctrl_freq; // 控制频率 ESSENTIAL!
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
  for (int i = 0; i < EIShaperBufLen; ++i) {
    memcpy(EIShaperBuf[i], pos, 2 * 7);
  }
  
  Serial.printf("EI Shaper!\n");
  Serial.printf("t1: %f, t2: %f, t3: %f\n", t1, t2, t3);
  Serial.printf("A1: %f, A2: %f, A3: %f\n", A1, A2, A3);
  Serial.printf("t2_discrete: %f, t3_discrete: %f\n", t2_discrete, t3_discrete);
}

void EIShaperApply(uint16_t pos[]) {
  ++EIShaperCur;
  EIShaperCur %= EIShaperBufLen;
  // memcpy(EIShaperBuf[EIShaperCur], pos, sizeof pos); // 错误的，pos此时是指针，不是数组，所以sizeof结果不是16而是4
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