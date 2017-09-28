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

stm32_serial::stm32_serial(): _isConnected(false),_rxStarted(false),_cached_packets_count(0)
{
  _rxtx = rp::hal::serial_rxtx::CreateRxTx();
}

u_result stm32_serial::connect(const char * port_path, _u32 baudrate)
{
    if (isConnected()) return RESULT_ALREADY_DONE;

    if (!_rxtx)
    {
      printf("wtf\n" );
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

void stm32_serial::transmit_test()
{
    _u8 txbuf[2] = {0xa5,0x5a};
    _u32 timeStamp = getms();

    _transmit(txbuf, 2);
    _transmit((_u8*)&timeStamp, 4);

    stm32_serial_packet_t node;
    if(IS_FAIL(_waitNode(&node, DEFAULT_TIMEOUT)))
    {
      printf("wtf\n");
      return;
    }
    printf("send time:%d\n",timeStamp);
    printf("receive time:%d\n",node.timeStamp);
    printf("sync error:%d\n",node.timeStamp - timeStamp);
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

u_result stm32_serial::_waitPacket(stm32_serial_packet_t * nodebuffer, size_t & count, _u32 timeout)
{
    if (!_isConnected) {
        count = 0;
        return RESULT_OPERATION_FAIL;
    }

    size_t   recvNodeCount =  0;
    _u32     startTs = getms();
    _u32     waitTime;
    u_result ans;

    while ((waitTime = getms() - startTs) <= timeout && recvNodeCount < count) {
        stm32_serial_packet_t node;
        if (IS_FAIL(ans = _waitNode(&node, timeout - waitTime))) {
            return ans;
        }

        nodebuffer[recvNodeCount++] = node;

        if (recvNodeCount == count) return RESULT_OK;
    }
    count = recvNodeCount;
    return RESULT_OPERATION_TIMEOUT;
}


u_result stm32_serial::_cachePacket(void)
{
    stm32_serial_packet_t                           local_buf[128];
    size_t                                             count = 128;
    stm32_serial_packet_t         local_packets[MAX_STM32_PACKETS];
    size_t                                   packet_count = 0;
    u_result                                 ans;
    memset(local_packets, 0, sizeof(local_packets));

    _waitPacket(local_buf, count); // // always discard the first data since it may be incomplete

    while(_rxStarted)
    {
        if (IS_FAIL(ans=_waitPacket(local_buf, count))) {
            if (ans != RESULT_OPERATION_TIMEOUT) {
                _rxStarted = false;
                return RESULT_OPERATION_FAIL;
            }
        }

        for (size_t pos = 0; pos < count; ++pos)
        {
            _lock.lock();
            memcpy(_cached_packets_buf, local_packets, packet_count*sizeof(stm32_serial_packet_t));
            _cached_packets_count = packet_count;
            _dataEvt.set();
            _lock.unlock();

            packet_count = 0;

            local_packets[packet_count++] = local_buf[pos];
            if (packet_count == MAX_STM32_PACKETS) packet_count-=1; // prevent overflow
        }
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
        if (_cachethread.getHandle() == 0) {
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
