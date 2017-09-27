#pragma once

#include "hal/abs_rxtx.h"
#include "hal/thread.h"
#include "hal/locker.h"
#include "hal/event.h"
#include "hal/util.h"

#include "rptypes.h"

#define DEFAULT_TIMEOUT  2000
#define MAX_STM32_PACKETS 2048

typedef struct {
  float imu_accelData[3];
  float imu_gyroData[3];
  uint32_t timeStamp;
} __attribute__((packed)) stm32_serial_packet_t;

class stm32_serial {
public:
  stm32_serial(){}

  u_result connect(const char * port_path, _u32 baudrate);
  void disconnect();
  bool isConnected();

  u_result init(_u32 timeout);
  u_result grabPacket(stm32_serial_packet_t * nodebuffer, size_t & count, _u32 timeout);

protected:
  bool                _isConnected;
  bool                _inited;
  stm32_serial_packet_t                   _cached_packets_buf[2048];
  size_t                                   _cached_packets_count;

  u_result _waitNode(stm32_serial_packet_t * node, _u32 timeout = DEFAULT_TIMEOUT);
  u_result _waitPacket(stm32_serial_packet_t * nodebuffer, size_t & count, _u32 timeout = DEFAULT_TIMEOUT);
  u_result _cachePacket(void);

  rp::hal::Locker            _lock;
  rp::hal::Event          _dataEvt;
  rp::hal::serial_rxtx*      _rxtx;
  rp::hal::Thread     _cachethread;
};
