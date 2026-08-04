#include "usart.h"

/* RX line accumulation. Only the ISR writes these when s_rx_ready == 0;
 * the main loop only writes s_rx_ready back to 0 after copying. This keeps the
 * buffer free of concurrent writers. */
static char     s_rx_buf[USART1_RX_LINE_MAX];
static volatile uint16_t s_rx_len;
static volatile uint8_t  s_rx_ready;

void MX_USART1_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* --- GPIO: PA9 = USART1_TX, PA10 = USART1_RX, AF7 --- */
  /* GPIOA clock is also enabled in MX_GPIO_Init(); enabling again is idempotent. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* --- USART1 peripheral --- */
  __HAL_RCC_USART1_CLK_ENABLE();

  /* Disable while configuring. */
  USART1->CR1 = 0U;
  USART1->CR2 = 0U;                 /* 1 stop bit, no LIN, no clock */
  USART1->CR3 = 0U;                 /* no CTSS/RTSS, no SMARTCARD, no DMAR/DMAT */

  /* Baud divider for OVER16 (reset default). Derived from the live APB2 clock,
   * so no assumption is made about (and no change is made to) the clock tree. */
  USART1->BRR = (HAL_RCC_GetPCLK2Freq() + (USART1_BAUDRATE / 2U)) / USART1_BAUDRATE;

  /* 8N1 = M=0 (8 bits), PCE=0 (no parity): both 0, already covered by CR1=0. */
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE;   /* TX + RX on, UE off, RXNEIE off */

  /* NVIC for RXNE (group 4 is used by the project -> subpriority ignored). */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* Enable the USART last (per RM0090). RX interrupt is started later by
   * usart1_start_rx() so init and "begin reception" stay separate. */
  USART1->CR1 |= USART_CR1_UE;
}

void usart1_start_rx(void)
{
  USART1->CR1 |= USART_CR1_RXNEIE;
}

void usart1_puts(const char *s)
{
  while (s != NULL && *s != '\0')
  {
    while ((USART1->SR & USART_SR_TXE) == 0U)
    {
      /* wait for TDR empty */
    }
    USART1->DR = (uint32_t)(uint8_t)(*s);
    s++;
  }
}

int usart1_getline(char *out, uint16_t maxlen)
{
  uint16_t i;

  if (out == NULL || maxlen == 0U)
  {
    return 0;
  }

  if (!s_rx_ready)
  {
    return 0;
  }

  /* Copy the completed line. s_rx_ready is still 1 here, so the ISR will keep
   * discarding incoming bytes -> the buffer is safe to read. */
  for (i = 0U; i < s_rx_len && i + 1U < maxlen; i++)
  {
    out[i] = s_rx_buf[i];
  }
  out[i] = '\0';

  /* Release the buffer for the next line. */
  s_rx_len   = 0U;
  s_rx_ready = 0U;

  return 1;
}

/*
 * USART1 receive interrupt. Only RXNE is enabled as an interrupt source, so we
 * only need to handle that. Reading SR then DR also clears an overrun (ORE).
 * Bytes received while a line is still pending (s_rx_ready == 1) are dropped.
 */
void USART1_IRQHandler(void)
{
  if ((USART1->SR & USART_SR_RXNE) != 0U)
  {
    uint8_t byte = (uint8_t)(USART1->DR & 0xFFU);   /* read clears RXNE */

    if (!s_rx_ready)
    {
      if (byte == (uint8_t)'\r')
      {
        /* ignore CR (handle CR, LF, and CRLF terminators uniformly) */
      }
      else if (byte == (uint8_t)'\n')
      {
        s_rx_buf[s_rx_len] = '\0';
        s_rx_ready = 1U;
      }
      else if (s_rx_len < (USART1_RX_LINE_MAX - 1U))
      {
        s_rx_buf[s_rx_len] = (char)byte;
        s_rx_len++;
      }
      else
      {
        /* Line longer than the buffer: flush what we have so the parser can
         * report an error instead of hanging. */
        s_rx_buf[USART1_RX_LINE_MAX - 1U] = '\0';
        s_rx_ready = 1U;
      }
    }
    /* else: still processing previous line -> drop the byte */
  }
}

/* ====================================================================== */
/* USART6: IMU over RS485 (HAL-based, to match the imported imu.c driver). */
/* PC6=TX, PC7=RX (AF8), 921600 8N1, RX via DMA2 Stream1/Ch5 (circular).   */
/* RS485 direction (DE/RE) is handled by the transceiver hardware.         */
/* ====================================================================== */
#include "imu.h"
#include <stdio.h>
#include <stdarg.h>

UART_HandleTypeDef huart6;
DMA_HandleTypeDef  hdma_usart6_rx;

void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 921600U;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  (void)HAL_UART_Init(&huart6);
}

/* HAL calls this from HAL_UART_Init: set up USART6 GPIO + DMA + NVIC. */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef gpio = {0};

  if (huart->Instance != USART6)
  {
    return;
  }

  __HAL_RCC_USART6_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* PC6 = USART6_TX, PC7 = USART6_RX (AF8) */
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &gpio);

  /* DMA2 Stream1 / Channel5 -> USART6_RX (circular) */
  hdma_usart6_rx.Instance = DMA2_Stream1;
  hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
  hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
  hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  (void)HAL_DMA_Init(&hdma_usart6_rx);
  __HAL_LINKDMA(huart, hdmarx, hdma_usart6_rx);

  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
}

void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
}

void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart6);
}

/* Idle/DMA RX event for USART6: feed the IMU frame, then restart RX. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance == USART6)
  {
    if (size > 0U)
    {
      Imu_ProcessRxData(Imu_GetRxBuffer(), size);
    }
    Imu_StartReceive();
  }
}

/* USART6 error (e.g. overrun during the re-arm window): clear ORE and re-arm
 * RX. Without this, reception freezes on the FIRST error. Matches the imu
 * repo's MyMain_ErrorCallback. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    Imu_StartReceive();
  }
}

/* imu.c logs via LOG_INFO -> MyMain_PrintLog. Route to the USART1 console. */
void MyMain_PrintLog(const char *format, ...)
{
  char buf[160];
  va_list args;
  int len;

  va_start(args, format);
  len = vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (len > 0)
  {
    if (len >= (int)sizeof(buf))
    {
      len = (int)sizeof(buf) - 1;
    }
    buf[len] = '\0';
    usart1_puts(buf);
  }
}
