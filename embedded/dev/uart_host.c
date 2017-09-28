#include "ch.h"
#include "hal.h"

#include "uart_host.h"
#include "mpu6050.h"

#define  NUM_SEGMENT 3U
#define  RXBUF_SIZE 6U
#define  HANDSHAKE_SIZE 2U

/* Put Datas to transfer here*/
static PIMUStruct pIMU;
static uint32_t timestamp;

static uint8_t start_flag = 0;
static const uint8_t start_seq[2] = {0xa5, 0x5a};
static const uint8_t handshake_seq[HANDSHAKE_SIZE] = {0xa5, 0x5a};
static const uint8_t size_of_segments[NUM_SEGMENT] = {2, 12, 4};
static const uint8_t seq_start[2] = {'\r','\n'};
static uint8_t* segments[NUM_SEGMENT];

static thread_t* uart_host_thread_p;
static thread_reference_t uart_host_thread_handler = NULL;
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

  if(uartp == UART_TO_HOST)
  {
    if(compare_input())
      start_flag = 1;

    chSysLockFromISR();
    chThdResumeI(&uart_host_thread_handler, MSG_OK);
    chSysUnlockFromISR();
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

#define   HOST_TRANSMIT_PERIOD  1000000/HOST_TRANSMIT_FREQ
static THD_WORKING_AREA(uart_host_thread_wa, 256);
static THD_FUNCTION(uart_host_thread, p)
{
  (void)p;
  chRegSetThreadName("uart host transmitter");

  uartStart(UART_TO_HOST, &uart_cfg_host);
  segments[0] = seq_start;
  segments[1] = pIMU->euler_angle;
  segments[2] = &timestamp;

  uint32_t tick = chVTGetSystemTimeX();
  uint8_t i;

  while(!start_flag)
  {
    uartStartReceive(UART_TO_HOST, RXBUF_SIZE, rxbuf);

    chSysLock();
    chThdSuspendS(&uart_host_thread_handler);
    chSysUnlock();
  }

  uint32_t time_host = *((uint32_t*)(rxbuf+2));
  uint32_t time_curr = ST2MS(chVTGetSystemTimeX());
  int32_t timestamp_sync = time_host - time_curr;

  uart_host_send(handshake_seq, HANDSHAKE_SIZE);
  uint32_t timestamp = ST2MS(chVTGetSystemTimeX()) + timestamp_sync;
  uart_host_send(&timestamp, 4);

  while(!chThdShouldTerminateX())
  {
    tick += US2ST(MPU6050_UPDATE_PERIOD);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    timestamp = chVTGetSystemTimeX();
    for (i = 0; i < NUM_SEGMENT; i++)
      uart_host_send(segments[i], size_of_segments[i]);
  }
}

void uart_host_init(void)
{
  pIMU = mpu6050_get();
  uart_host_thread_p = chThdCreateStatic(uart_host_thread_wa, sizeof(uart_host_thread_wa),
                       NORMALPRIO,
                       uart_host_thread, NULL);
}
