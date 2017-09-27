#include "ch.h"
#include "hal.h"

#include "uart_host.h"
#include "mpu6050.h"

static uint8_t lidar_tx_start_flag = 0;
static uint8_t rxbuf[7];
static const uint8_t rplidar_rx_descriptor[7] = {0xa5,0x5a,0x54,0x00,0x00,0x40,0x82};

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
      palTogglePad(GPIOD,GPIOD_LED4);

      //timestamp = chVTGetSystemTimeX();
      //uartStartSend(UART_LIDAR, 4, &timestamp);

      data_length = 0;
    }
  }

  data_length++;
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
  NULL,
  NULL,
  NULL,
  rxchar,
  NULL,
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

static THD_WORKING_AREA(uart_host_thread_wa, 256);
static THD_FUNCTION(uart_host_thread, p)
{
  (void)p;
  chRegSetThreadName("uart host transmitter");
  pIMU = mpu6050_get();

  uartStart(UART_TO_HOST, &uart_cfg_host);

  while(true)
  {
    uint32_t tick = chVTGetSystemTimeX();

    uartStartSend(UART_TO_HOST, 24, pIMU->accelData);
    uartStartSend(UART_TO_HOST, 4, &tick);

    chThdSleepMilliseconds(5);
  }
}

void uart_host_init(void)
{
  uartStart(UART_LIDAR, &uart_cfg);

  chThdCreateStatic(uart_host_thread_wa, sizeof(uart_host_thread_wa),
                    NORMALPRIO,
                    uart_host_thread, NULL);
}
