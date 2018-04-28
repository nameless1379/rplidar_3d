#include "ch.h"
#include "hal.h"

#include "main.h"

#define  NUM_SEGMENT 5U
#define  RXBUF_START_SIZE  15U
#define  RXBUF_CMD_SIZE    15U
#define  RXBUF_SIZE  15U
#define  START_SEQ_SIZE 2U

/* Put Datas to transfer here*/
static PIMUStruct pIMU;
static uint32_t timestamp;

static uint8_t start_flag = 0;
static const uint8_t start_seq[START_SEQ_SIZE] = {0xa5, 0x5a};
static const float imu_data_invalid_seq[4] = {100.0f, 100.0f, 100.0f, 100.0f};
static const uint8_t size_of_segments[NUM_SEGMENT] = {START_SEQ_SIZE, 16, 4, 12, 4};
static uint8_t* segments[NUM_SEGMENT];

static thread_t* uart_host_thread_p;
static thread_reference_t uart_host_thread_handler = NULL;
static thread_reference_t uart_receive_thread_handler = NULL;
static uint8_t rxbuf[RXBUF_SIZE];

static uint8_t compare_input(void)
{
  uint8_t i;
  for (i = 0; i < 2; i++)
    if(rxbuf[i] != start_seq[i])
      return false;
  return true;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp)
{
  if(uartp == UART_TO_HOST)
  {
    chSysLockFromISR();
    chThdResumeI(&uart_host_thread_handler, MSG_OK);
    chSysUnlockFromISR();
  }
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {

  if(uartp == UART_TO_HOST && compare_input())
  {
    if(!start_flag && rxbuf[2] == 0x00)
    {
      start_flag = 1;
      chSysLockFromISR();
      chThdResumeI(&uart_host_thread_handler, MSG_OK);
      chSysUnlockFromISR();
    }
    else
    {
      float stepper_velcmd;
      chSysLockFromISR();
      switch(rxbuf[2])
      {
        case 0x01:
          stepper_velcmd = *((float*)(rxbuf + 3));
          if(stepper_velcmd == 0.0f)
            stepper_stop();
          else
            stepper_setvelocity(stepper_velcmd);
          break;
        case 0x02:
          break;
        case 0xAA:
          if(rxbuf[3] == 0xA5)
            system_resetCmd(ENABLE);
          break;
        case 0xFE:
          if(rxbuf[3] == 0xAA)
          {
            attitude_resetYaw(pIMU, 0.0f);
            chassis_wheelOdeometryReset();
          }
          break;
      }
      chThdResumeI(&uart_receive_thread_handler, MSG_OK);
      chSysUnlockFromISR();
    }
  }
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg_host = {
  NULL,txend2,rxend,NULL,NULL,
  115200,
  0,
  0,
  0
};

static inline void uart_host_send(const uint8_t* txbuf, const uint8_t size)
{
  uartStopSend(UART_TO_HOST);
  uartStartSend(UART_TO_HOST, size, txbuf);
  chSysLock();
  chThdSuspendS(&uart_host_thread_handler);
  chSysUnlock();
}

static THD_WORKING_AREA(uart_receive_thread_wa, 128);
static THD_FUNCTION(uart_receive_thread, p)
{
  (void)p;
  chRegSetThreadName("uart host transmitter");

  while(!chThdShouldTerminateX())
  {
    uartStartReceive(UART_TO_HOST, RXBUF_CMD_SIZE, rxbuf);

    chSysLock();
    chThdSuspendS(&uart_receive_thread_handler);
    chSysUnlock();
  }
}


#define  HOST_TRANSMIT_PERIOD  1000000/HOST_TRANSMIT_FREQ
static THD_WORKING_AREA(uart_host_thread_wa, 256);
static THD_FUNCTION(uart_host_thread, p)
{
  (void)p;
  chRegSetThreadName("uart host transmitter");

  float stepper_angle = 0.0f;

  uartStart(UART_TO_HOST, &uart_cfg_host);
  segments[0] = start_seq;
  segments[1] = (uint8_t*)(pIMU->qIMU);
  segments[2] = (uint8_t*)&stepper_angle;
  segments[3] = (uint8_t*)chassis_wheelOdeometryGet(); //chassis odeometry data
  segments[NUM_SEGMENT - 1] = (uint8_t*)&timestamp;

  uint32_t tick = chVTGetSystemTimeX();
  uint8_t i;

  while(!start_flag)
  {
    uartStartReceive(UART_TO_HOST, RXBUF_START_SIZE, rxbuf);

    chSysLock();
    chThdSuspendS(&uart_host_thread_handler);
    chSysUnlock();
  }

  uint32_t time_host = *((uint32_t*)(rxbuf+3));
  uint32_t time_curr = ST2MS(chVTGetSystemTimeX());
  int32_t timestamp_sync = time_host - time_curr;
  timestamp = ST2MS(chVTGetSystemTimeX()) + timestamp_sync;

  for (i = 0; i < NUM_SEGMENT; i++)
    uart_host_send(segments[i], size_of_segments[i]);

  stepper_init(STEPPER_CCW);

  chThdCreateStatic(uart_receive_thread_wa, sizeof(uart_receive_thread_wa),
                    NORMALPRIO + 7,
                    uart_receive_thread, NULL);

  chThdSleepMilliseconds(100);

  while(!chThdShouldTerminateX())
  {
    tick += US2ST(HOST_TRANSMIT_PERIOD);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    stepper_angle = stepper_get_angle();

    if(pIMU->state != IMU_STATE_READY)
      segments[1] = (uint8_t*)imu_data_invalid_seq;

    timestamp = ST2MS(chVTGetSystemTimeX()) + timestamp_sync;
    for (i = 0; i < NUM_SEGMENT; i++)
      uart_host_send(segments[i], size_of_segments[i]);
  }
}

void uart_host_init(void)
{
  pIMU = imu_get();
  uart_host_thread_p = chThdCreateStatic(uart_host_thread_wa, sizeof(uart_host_thread_wa),
                       NORMALPRIO,
                       uart_host_thread, NULL);
}
