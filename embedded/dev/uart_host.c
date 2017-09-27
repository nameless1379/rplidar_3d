#include "ch.h"
#include "hal.h"

#include "uart_host.h"
#include "mpu6050.h"

static uint8_t lidar_tx_start_flag = 0;
static uint8_t rxbuf[7];
static const uint8_t rplidar_rx_descriptor[7] = {0xa5,0x5a,0x54,0x00,0x00,0x40,0x82};

static thread_reference_t uart_timestamp = NULL;
static uint32_t data_length = 0;
static uint32_t timestamp;

static PIMUStruct pIMU;

//static uint8_t rxbuf_fifo[5];
//static uint8_t rxchar_in_queue = 0;

static uint8_t compare_descriptor(void)
{
  uint8_t i;
  for (i = 0; i < 7U; i++)
    if(rxbuf[i] != rplidar_rx_descriptor[i])
      return false;
  return true;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c)
{
//  rxbuf_fifo[rxchar_in_queue++] = (uint8_t)c;
  while(!(*UART_LIDAR.usart->SR | USART_SR_TXE));

  *UART_LIDAR.usart->DR = (uint8_t)c;
  *UART_LIDAR.usart->CR1 |= USART_CR1_TE;

  if(!lidar_tx_start_flag)
  {
    if(c == 0xa5)
    {
      lidar_tx_start_flag = 1;
      rxbuf[0] = (uint8_t)c;
    }

    data_length = 0;
  }
  else if(lidar_tx_start_flag == 1)
  {
    rxbuf[data_length] = (uint8_t)c;
    if(data_length == 6)
    {
      if(compare_descriptor())
        lidar_tx_start_flag = 2;

      data_length = 0;
    }
  }
  else
  {
    if(data_length == 83)
    {
      timestamp = chVTGetSystemTimeX();
      palTogglePad(GPIOD,GPIOD_LED4);

      chSysLockFromISR();
      chThdResumeI(&uart_timestamp,MSG_OK);
      chSysUnlockFromISR();

      data_length = 0;
    }
  }

  data_length++;
}

static void rxerr(UARTDriver *uartp, uartflags_t e)
{
  palTogglePad(GPIOD,GPIOD_LED5);
}


/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
  NULL,
  NULL,
  NULL,
  rxchar,
  rxerr,
  115200,
  0,
  0,
  0
};

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg_host = {
  NULL,NULL,NULL,NULL,NULL,
  115200,
  0,
  0,
  0
};

static THD_WORKING_AREA(uart_timestamp_thread_wa,256);
static THD_FUNCTION(uart_timestamp_thread, p)
{
  (void)p;
  chRegSetThreadName("uart timestamp");
  uint8_t i;
  uint32_t timestamp;

  while(true)
  {
    chSysLock();
    chThdSuspendS(&uart_timestamp);
    chSysUnlock();

    chThdSleepMicroseconds(100);
    timestamp = chVTGetSystemTimeX();
    
    uartStopSend(UART_LIDAR);
    uartStartSend(UART_LIDAR, 4, &timestamp);
  }
}

static THD_WORKING_AREA(uart_host_thread_wa, 256);
static THD_FUNCTION(uart_host_thread, p)
{
  (void)p;
  chRegSetThreadName("uart host transmitter");
  pIMU = mpu6050_get();

  uartStart(UART_TO_HOST, &uart_cfg_host);
  dmaStreamRelease(*UART_TO_HOST.dmarx);

  while(true)
  {
    uint32_t tick = chVTGetSystemTimeX();

    uartStopSend(UART_TO_HOST);
    uartStartSend(UART_TO_HOST, 24, pIMU->accelData);

    chThdSleepMilliseconds(50);
  }
}

void uart_host_init(void)
{
  uartStart(UART_LIDAR, &uart_cfg);

  chThdCreateStatic(uart_timestamp_thread_wa,sizeof(uart_timestamp_thread_wa),
                    NORMALPRIO, uart_timestamp_thread, NULL);

  chThdCreateStatic(uart_host_thread_wa, sizeof(uart_host_thread_wa),
                   NORMALPRIO,
                   uart_host_thread, NULL);
}
