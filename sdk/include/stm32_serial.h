#pragma once

#include "ros/ros.h"

#include "rptypes.h"

#include "hal/util.h"
#include "hal/abs_rxtx.h"
#include "hal/thread.h"
#include "hal/locker.h"
#include "hal/event.h"

#define DEFAULT_TIMEOUT  2000
#define MAX_STM32_PACKETS 2048
#define MAX_SYNC_ERROR  20
#define TX_BUFFER_SIZE  15

#define TX_HANDSHAKE_HEADER  0x00
#define TX_STEPPER_HEADER    0x01
#define TX_GYRO_BIAS_HEADER  0x02
#define TX_MCU_RESET_HEADER  0xAA
#define TX_RESET_HEADER      0xFE

typedef struct {
  _u16 sync;
  float imu_data[4];
  float stepper_angle;
  float wheel_odeometry[3];
  _u32 timeStamp;
} __attribute__((packed)) stm32_serial_packet_t;

class stm32_serial {
public:
  stm32_serial();
  ~stm32_serial();

  u_result connect(const char * port_path, _u32 baudrate);
  void disconnect();
  bool isConnected();

  u_result transmit_handshake(void);
  u_result transmit_stepper_cmd(const float vel_cmd);
  u_result transmit_gyro_bias(const float gyro_corr_z,
    const float gyro_bias_z);
  u_result transmit_reset_cmd(void);
  u_result transmit_MCUreset_cmd(void);

  u_result start_rx(_u32 timeout);
  u_result grabPacket(stm32_serial_packet_t * nodebuffer, size_t & count, _u32 timeout);

  void publish_pos_msg(ros::Publisher *pub, stm32_serial_packet_t *node, std::string frame_id);

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

  ros::Time ros_start;
  _u32 timeStamp_start;
};
