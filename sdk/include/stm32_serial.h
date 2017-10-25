#pragma once

#include "rptypes.h"

#include "hal/util.h"
#include "hal/abs_rxtx.h"
#include "hal/thread.h"
#include "hal/locker.h"
#include "hal/event.h"

#define DEFAULT_TIMEOUT  2000
#define MAX_STM32_PACKETS 2048
#define MAX_SYNC_ERROR  20


typedef struct {
  _u16 sync;
  float imu_data[4];
  float stepper_angle;
  _u32 timeStamp;
} __attribute__((packed)) stm32_serial_packet_t;

class stm32_serial {
public:
  stm32_serial();

  u_result connect(const char * port_path, _u32 baudrate);
  void disconnect();
  bool isConnected();

  u_result transmit_handshake();

  u_result start_rx(_u32 timeout);
  u_result grabPacket(stm32_serial_packet_t * nodebuffer, size_t & count, _u32 timeout);

protected:
  bool                _isConnected;
  bool                _rxStarted;
  stm32_serial_packet_t                   _cached_packets_buf[2048];
  size_t                                   _cached_packets_count;

  u_result _transmit(_u8* const txbuf, const _u16 size);
  u_result _waitNode(stm32_serial_packet_t * node, _u32 timeout = DEFAULT_TIMEOUT);
  u_result _cachePacket(void);

  rp::hal::Locker            _lock;
  rp::hal::Event          _dataEvt;
  rp::hal::serial_rxtx*      _rxtx;
  rp::hal::Thread     _cachethread;
};
