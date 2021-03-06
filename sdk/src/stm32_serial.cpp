#if defined(_WIN32)
#include "arch\win32\arch_win32.h"
#elif defined(_MACOS)
#include "arch/macOS/arch_macOS.h"
#elif defined(__GNUC__)
#include "arch/linux/arch_linux.h"
#else
#error "unsupported target"
#endif

#include "stm32_serial.h"
#include "math.h"

stm32_serial::stm32_serial(): _isConnected(false),_rxStarted(false),_cached_packets_count(0)
{
  _rxtx = rp::hal::serial_rxtx::CreateRxTx();
}

stm32_serial::~stm32_serial()
{
  // force disconnection
  transmit_MCUreset_cmd();
  delay(100);
  disconnect();

  rp::hal::serial_rxtx::ReleaseRxTx(_rxtx);
}

u_result stm32_serial::connect(const char * port_path, _u32 baudrate)
{
    if (isConnected()) return RESULT_ALREADY_DONE;

    if (!_rxtx)
    {
      return RESULT_INSUFFICIENT_MEMORY;
    }

    {
        rp::hal::AutoLocker l(_lock);

        // establish the serial connection...
        if (!_rxtx->bind(port_path, baudrate)  ||  !_rxtx->open())
            return RESULT_INVALID_DATA;

        _rxtx->flush(0);
    }

    _isConnected = true;

    return RESULT_OK;
}

void stm32_serial::disconnect()
{
    if (!_isConnected) return ;
    _rxtx->close();
}

bool stm32_serial::isConnected()
{
    return _isConnected;
}

u_result stm32_serial::_transmit(_u8* txbuf, _u16 size)
{
    if (!_isConnected)
      return RESULT_OPERATION_FAIL;

    // send header first
    _rxtx->senddata(txbuf, size) ;
    return RESULT_OK;
}

u_result stm32_serial::transmit_handshake(void)
{
    _u8 txbuf[3] = {0xa5,0x5a,TX_HANDSHAKE_HEADER};
    _u8 res[TX_BUFFER_SIZE - 7];

    ros_start = ros::Time::now();
    timeStamp_start = getms();

    _transmit(txbuf, 3);
    _transmit((_u8*)&timeStamp_start, 4);
    _transmit(res, TX_BUFFER_SIZE - 7);

    stm32_serial_packet_t node;
    if(IS_FAIL(_waitNode(&node, DEFAULT_TIMEOUT)))
      return RESULT_OPERATION_FAIL;

    _s32 sync_error = node.timeStamp - timeStamp_start;
    printf("sync error:%d\n",sync_error);
    if(abs(sync_error) > MAX_SYNC_ERROR)
      return RESULT_OPERATION_FAIL;

    return RESULT_OK;
}

u_result stm32_serial::transmit_stepper_cmd(const float vel_cmd)
{
    _u8 txbuf[3] = {0xa5,0x5a,TX_STEPPER_HEADER};
    _u8 res[TX_BUFFER_SIZE - 7];

    _transmit(txbuf, 3);
    _transmit((_u8*)&vel_cmd, 4);
    _transmit(res, TX_BUFFER_SIZE - 7);

    return RESULT_OK;
}

u_result stm32_serial::transmit_gyro_bias(const float gyro_corr_z,
  const float gyro_bias_z)
{
    _u8 txbuf[3] = {0xa5,0x5a,TX_GYRO_BIAS_HEADER};
    _u8 res[TX_BUFFER_SIZE - 7];

    _transmit(txbuf, 3);
    _transmit((_u8*)&gyro_corr_z, 4);
    _transmit((_u8*)&gyro_bias_z, 4);
    _transmit(res, TX_BUFFER_SIZE - 11);

    return RESULT_OK;
}

u_result stm32_serial::transmit_reset_cmd(void)
{
  _u8 txbuf[3] = {0xa5,0x5a,TX_RESET_HEADER};
  _u8 res[TX_BUFFER_SIZE - 3];

  memset(res, 0xAA, TX_BUFFER_SIZE - 3);

  _transmit(txbuf, 3);
  _transmit(res, TX_BUFFER_SIZE - 3);

  return RESULT_OK;
}

u_result stm32_serial::transmit_MCUreset_cmd(void)
{
    printf("Resetting STM32 MCU...\r\n");
    _u8  txbuf[3] = {0xa5,0x5a,TX_MCU_RESET_HEADER};
    _u8  res[TX_BUFFER_SIZE - 3];

    memset(res, 0xA5, TX_BUFFER_SIZE - 3);

    _transmit(txbuf, 3);
    _transmit(res, TX_BUFFER_SIZE - 3);

    return RESULT_OK;
}

u_result stm32_serial::_waitNode(stm32_serial_packet_t* node, _u32 timeout)
{
    int  recvPos = 0;
    _u32 startTs = getms();
    _u8  recvBuffer[sizeof(stm32_serial_packet_t)];
    _u8 *nodeBuffer = (_u8*)node;
    _u32 waitTime;

   while ((waitTime=getms() - startTs) <= timeout) {
        size_t remainSize = sizeof(stm32_serial_packet_t) - recvPos;
        size_t recvSize;

        int ans = _rxtx->waitfordata(remainSize, timeout-waitTime, &recvSize);
        if (ans == rp::hal::serial_rxtx::ANS_DEV_ERR)
            return RESULT_OPERATION_FAIL;
        else if (ans == rp::hal::serial_rxtx::ANS_TIMEOUT)
            return RESULT_OPERATION_TIMEOUT;

        if (recvSize > remainSize) recvSize = remainSize;

        _rxtx->recvdata(recvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos) {
          _u8 currentByte = recvBuffer[pos];
          switch (recvPos)
          {
            case 0:
              if ( currentByte != 0xa5 )
                continue;

              break;
            case 1:
              if (currentByte != 0x5a)
              {
                recvPos = 0;
                continue;
              }
              break;
          }

          nodeBuffer[recvPos++] = currentByte;
          if (recvPos == sizeof(stm32_serial_packet_t))
            return RESULT_OK;
        }
    }

    return RESULT_OPERATION_TIMEOUT;
}

u_result stm32_serial::_cachePacket(void)
{
    u_result                      ans;

    while(_rxStarted)
    {
      stm32_serial_packet_t node;
      if (IS_FAIL(ans = _waitNode(&node, DEFAULT_TIMEOUT)))
      {
        return ans;
      }
      _lock.lock();

      //Prevent overflow
      if(_cached_packets_count >= MAX_STM32_PACKETS)
        _cached_packets_count = MAX_STM32_PACKETS - 1;

      stm32_serial_packet_t* _cached_packets_buf_dest =
          _cached_packets_buf + _cached_packets_count;
      memcpy(_cached_packets_buf_dest, &node, sizeof(stm32_serial_packet_t));
      _cached_packets_count++;

      _dataEvt.set();
      _lock.unlock();

    }
    _rxStarted = false;
    return RESULT_OK;
}

u_result stm32_serial::start_rx(_u32 timeout)
{
    u_result ans;
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    if (_rxStarted) return RESULT_ALREADY_DONE;
    {
        rp::hal::AutoLocker l(_lock);

        _rxStarted = true;
        _cachethread = CLASS_THREAD(stm32_serial, _cachePacket);
        if (_cachethread.getHandle() == 0)
        {
            return RESULT_OPERATION_FAIL;
        }
    }
    return RESULT_OK;
}

u_result stm32_serial::grabPacket(stm32_serial_packet_t * nodebuffer, size_t & count, _u32 timeout)
{
    switch (_dataEvt.wait(timeout))
    {
    case rp::hal::Event::EVENT_TIMEOUT:
        count = 0;
        return RESULT_OPERATION_TIMEOUT;
    case rp::hal::Event::EVENT_OK:
        {
            if(_cached_packets_count == 0) return RESULT_OPERATION_TIMEOUT; //consider as timeout

            rp::hal::AutoLocker l(_lock);

            size_t size_to_copy = count < _cached_packets_count ? count : _cached_packets_count;

            memcpy(nodebuffer, _cached_packets_buf, size_to_copy*sizeof(stm32_serial_packet_t));
            count = size_to_copy;
            _cached_packets_count = 0;
        }
        return RESULT_OK;

    default:
        count = 0;
        return RESULT_OPERATION_FAIL;
    }
}
