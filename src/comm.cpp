#include "comm.h"
#include <string.h>
#include <Arduino.h>
#include "main.h"

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

// 不同类型的数据包大小
int comm_payload_size[] = {
  14, // COMM_TYPE_PING
  14, // COMM_TYPE_PONG
  14, // COMM_TYPE_CTRL
  14, // COMM_TYPE_FEEDBACK
  14, // COMM_TYPE_TORQUE
  14, // COMM_TYPE_CONFIG_WRITE
  14, // COMM_TYPE_CONFIG_READ
  14, // COMM_TYPE_CONFIG_FEEDBACK
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
  true,  // COMM_TYPE_CONFIG_WRITE
  true,  // COMM_TYPE_CONFIG_READ
  true,  // COMM_TYPE_CONFIG_FEEDBACK
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