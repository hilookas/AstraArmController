#ifndef _COMM_H_
#define _COMM_H_

#include <stdbool.h>
#include <stdint.h>

bool serial_init(void);

bool serial_send_blocking(uint8_t c);

bool serial_recv_poll(uint8_t *c);

#define COMM_PAYLOAD_SIZE_MAX 100

typedef enum {
  COMM_TYPE_PING,
  COMM_TYPE_PONG,
  COMM_TYPE_CTRL,
  COMM_TYPE_FEEDBACK,
  COMM_TYPE_TORQUE,
  COMM_TYPE_CONFIG_WRITE,
  COMM_TYPE_CONFIG_READ,
  COMM_TYPE_CONFIG_FEEDBACK,
  COMM_TYPE_PIDTUNE,
} comm_type_t;

extern int comm_payload_size[];

extern bool comm_type_importance[];

bool comm_send_blocking(comm_type_t type, const uint8_t payload[]);

bool comm_recv_poll(comm_type_t *type, uint8_t payload[]);

bool comm_recv_poll_last(comm_type_t *type, uint8_t payload[]);

bool comm_init(void);

#endif