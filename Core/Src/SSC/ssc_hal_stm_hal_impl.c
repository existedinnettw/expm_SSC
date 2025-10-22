#include "ssc_hal.h"
#include <ecatappl.h>
#include <main.h>

#include <stm32f4xx_hal.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// --- STM32 HAL handles (extern from main.c or define here if needed) ---
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

// --- Pin/Port definitions: adapt to your CubeMX configuration ---
#define ESC_SPI_HANDLE hspi1
#define ESC_SPI_TIMEOUT 2 // ms
#define ESC_TIM_HANDLE htim2

// interface lib stm32cubemx already include main.h, which define Pin

// active low
static void
ESC_SPI_CS_Enable()
{
  HAL_GPIO_WritePin(SPI1_SS_ESC_GPIO_Port, SPI1_SS_ESC_Pin, GPIO_PIN_RESET);
  while (HAL_GPIO_ReadPin(SPI1_SS_ESC_GPIO_Port, SPI1_SS_ESC_Pin) != GPIO_PIN_RESET) {
  }
}
static void
ESC_SPI_CS_Disable()
{
  HAL_GPIO_WritePin(SPI1_SS_ESC_GPIO_Port, SPI1_SS_ESC_Pin, GPIO_PIN_SET);
  while (HAL_GPIO_ReadPin(SPI1_SS_ESC_GPIO_Port, SPI1_SS_ESC_Pin) != GPIO_PIN_SET) {
  }
}


/**
 * @private
 *
 */
typedef union
{
  UINT8 Byte[2]; // 0x220-0x221 (little endian)
  UINT16 Word;
} UALEVENT;

/**
 * @private
 * @brief
 * contains the content of the ALEvent register (0x220), this variable is
 * updated on each Access to the Esc
 */
UALEVENT EscALEvent;

#if ECAT_TIMER_INT == 1
#elif ECAT_TIMER_INT == 0
static UINT32 internal_timer = 0; // global timer variable, incremented by 1 every 1ms
#endif

// #if INTERRUPTS_SUPPORTED == 1
/**
 * @see irq_unlock
 */
void
ENABLE_ESC_INT()
{

  // __enable_irq();
  HAL_NVIC_EnableIRQ(ESC_SPI_IRQ_EXTI_IRQn);
  // HAL_NVIC_EnableIRQ(ESC_SYNC0_EXTI_IRQn);
  // HAL_NVIC_EnableIRQ(ESC_SYNC1_EXTI_IRQn);

  // #if ECAT_TIMER_INT == 1
  //   // repeated start Tim will result in error.
  //   // however, `ResetALEventMask` call wrap `HW_EscWriteWord` in DISABLE/ENABLE_ESC_INT
  //   if ((&ESC_TIM_HANDLE)->State == HAL_TIM_STATE_BUSY) {
  //     return;
  //   }
  //   //  HAL_TIM_Base_Start_IT(&ESC_TIM_HANDLE);
  //   if (HAL_TIM_Base_Start_IT(&ESC_TIM_HANDLE) != HAL_OK) {
  //     // LOG_ERR("ESC timer start failed");
  //     // printf("ESC timer start failed\n");
  //     // assert(FALSE);
  //     Error_Handler();
  //   }
  // #endif
  return;
}
/**
 * @see irq_lock
 * @details
 * aware this function will stop hal_delay
 */
void
DISABLE_ESC_INT()
{
  // __disable_irq();
  HAL_NVIC_DisableIRQ(ESC_SPI_IRQ_EXTI_IRQn);
  // I found most people just disable spi IRQ
  //  HAL_NVIC_DisableIRQ(ESC_SYNC0_EXTI_IRQn);
  //  HAL_NVIC_DisableIRQ(ESC_SYNC1_EXTI_IRQn);

  // #if ECAT_TIMER_INT == 1

  //   // repeated stop Tim will not result in error
  //   // will stop timer too. since all intterrupts have same priority, we don't need to disable irq
  //   // or try to disable irq only, `__HAL_TIM_DISABLE_IT`
  //   //  HAL_TIM_Base_Stop_IT(&ESC_TIM_HANDLE);
  //   if (HAL_TIM_Base_Stop_IT(&ESC_TIM_HANDLE) != HAL_OK) {
  //     // LOG_ERR("ESC timer stop failed");
  //     // printf("ESC timer stop failed\n");
  //     // assert(FALSE);
  //     Error_Handler();
  //   }
  // #endif
  return;
}
// #endif


/**
 * @see Application Note ET9300 ch4.2
 * @see Application Note ET9300 ch5.1 Interrupt Handler
 * @see void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 * @see void PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 */
void
counter_cb(TIM_HandleTypeDef* htim)
{
  DISABLE_ESC_INT();
  UNUSED(htim);
  // printf("timer called\n"); //check period called
#if ECAT_TIMER_INT == 1

  // `ECAT_CheckTimer` shall be called every 1ms.
  ECAT_CheckTimer();
// LOG_INF("1ms cb called");
// printf("1ms cb called\n");
#elif ECAT_TIMER_INT == 0
  internal_timer += 1;
#endif
  ENABLE_ESC_INT();
}

// ch5.2.1 Generic
UINT8
HW_Init(void)
{
  {
    DISABLE_ESC_INT();
    HAL_NVIC_DisableIRQ(ESC_SPI_IRQ_EXTI_IRQn);
    HAL_NVIC_DisableIRQ(ESC_SYNC0_EXTI_IRQn);
    HAL_NVIC_DisableIRQ(ESC_SYNC1_EXTI_IRQn);
    if (HAL_TIM_Base_Stop_IT(&ESC_TIM_HANDLE) != HAL_OK) {
      Error_Handler();
    }
    ESC_SPI_CS_Disable();
  }

  HAL_Delay(100);
  // read eeprom load pin, check eeprom load
  while (TRUE) {
    bool ret = HAL_GPIO_ReadPin(ESC_EEP_LOAD_GPIO_Port, ESC_EEP_LOAD_Pin) == GPIO_PIN_SET;
    if (ret) {
      break;
    } else {
      // LOG_INF("EEPROM load is not active yet...");
      printf("EEPROM load is not active yet, waiting...\n");
      // HAL have no us delay
      HAL_Delay(100);
    }
  }
  // LOG_INF("EEPROM load is active");
  printf("EEPROM load is active, start loading...\n");

  // check ESC SPI read/write ready
  UINT32 intMask = 0x00;
  while (TRUE) {
    // LOG_INF("Waiting for ESC to be ready...");
    // k_sleep(K_USEC(10000));
    intMask = 0x93;
    HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
    // k_sleep(K_USEC(2));
    intMask = 0;
    HW_EscReadDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
    // if (intMask != 0x93) {
    //   // print intMask value
    //   LOG_INF("ESC not ready to write in SPI value yet, intMask: %lx",
    //   intMask);
    // }
    if (intMask == 0x93) {
      break;
    }
    HAL_Delay(1);
  }

  intMask = 0x0;
  HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);


  printf("ESC is ready to write in SPI value\n");
  // intMask = 0x00; // notify all
  // HW_EscWriteDWordIsr(intMask, ESC_AL_EVENTMASK_OFFSET);

#if ECAT_TIMER_INT == 1
/**
 * start timer
 */
#if USE_HAL_TIM_REGISTER_CALLBACKS != 1
#error "current SSC implement use USE_HAL_TIM_REGISTER_CALLBACKS, plz enable it in CubeMX"
#endif
  ESC_TIM_HANDLE.PeriodElapsedCallback = counter_cb;
#endif

  printf("HW_Init() finished\n");

  // start global interrupt
  // HAL_NVIC_EnableIRQ(ESC_SPI_IRQ_EXTI_IRQn);
  // HAL_NVIC_EnableIRQ(ESC_SYNC0_EXTI_IRQn);
  // HAL_NVIC_EnableIRQ(ESC_SYNC1_EXTI_IRQn);

  // LOG_INF("HW_Init() finished");
  {
    HAL_NVIC_EnableIRQ(ESC_SPI_IRQ_EXTI_IRQn);
    HAL_NVIC_EnableIRQ(ESC_SYNC0_EXTI_IRQn);
    HAL_NVIC_EnableIRQ(ESC_SYNC1_EXTI_IRQn);
#if ECAT_TIMER_INT == 1
    if (HAL_TIM_Base_Start_IT(&ESC_TIM_HANDLE) != HAL_OK) {
      // LOG_ERR("ESC timer start failed");
      printf("ESC timer start failed\n");
      return 1; // Error
    }
#endif
    ENABLE_ESC_INT();
  }
  return 0; // Success
}
void
HW_Release(void)
{
}

/**
 * @private
 * @brief
 * The function operates a SPI access without addressing.
 * @details
 * The first two bytes of an access to the EtherCAT ASIC always deliver the
 * AL_Event register (0x220). It will be saved in the global "EscALEvent"
 */
static void
GetInterruptRegister(void)
{
  VARVOLATILE UINT8 dummy;
  HW_EscRead((MEM_ADDR*)&dummy, 0, 1);
}

// #if INTERRUPTS_SUPPORTED
/**
 * @private
 * @brief
 * The function operates a SPI access without addressing.
 * @details
 * Shall be implemented if interrupts are supported else this function is equal
 * to "GetInterruptRegsiter()" The first two bytes of an access to the EtherCAT
 * ASIC always deliver the AL_Event register (0x220). It will be saved in the
 * global "EscALEvent"
 */
static void
ISR_GetInterruptRegister(void)
{
  VARVOLATILE UINT8 dummy;
  HW_EscReadIsr((MEM_ADDR*)&dummy, 0, 1);
}
// #endif

/**
 * @see
 * Section III-ET1100 Hardware Description ch6.3.6
 * @see
 * Application Note ET9300 ch9
 * @see
 * Section II-ET1100 Register Description ch2.32
 */
UINT16
HW_GetALEventRegister(void)
{
  GetInterruptRegister();
  // printf("ESC ALEvent register: %x\n", EscALEvent.Word);
  return EscALEvent.Word;
}

// #if INTERRUPTS_SUPPORTED
/**
 * @see
 * Section III-ET1100 Hardware Description ch6.3.6
 * @see
 * Application Note ET9300 ch9
 * @see
 * Section II-ET1100 Register Description ch2.32
 */
UINT16
HW_GetALEventRegister_Isr(void)
{
  ISR_GetInterruptRegister();
  return EscALEvent.Word;
}
// #endif

#if AL_EVENT_ENABLED == 1 && IS_SSC_LOWER_5P10
/**
 * @todo
 * not yet implement
 */
void
HW_ResetALEventMask(UINT16 intMask) {};
void
HW_SetALEventMask(UINT16 intMask) {};
#endif

#if UC_SET_ECAT_LED == 1
void
HW_SetLed(UINT8 RunLed, UINT8 ErrLed)
{
  UNUSED(RunLed);
  UNUSED(ErrLed);
}
#endif

#if BOOTSTRAPMODE_SUPPORTED == 1
void
HW_RestartTarget(void) {};
#endif

#if IS_SSC_LOWER_5P10

void
HW_DisableSyncManChannel(UINT8 channel) {};
void
HW_EnableSyncManChannel(UINT8 channel) {};
TSYNCMAN*
HW_GetSyncMan(UINT8 channel)
{
  // return TSYNCMAN();
  return NULL;
};
#endif

#if ECAT_TIMER_INT == 0

#ifndef ECAT_TIMER_INC_P_MS
#error "ECAT_TIMER_INC_P_MS is necessay if ECAT_TIMER_INT is 0"
#endif

UINT32
HW_GetTimer(void)
{
  return internal_timer;
}

void
HW_ClearTimer(void)
{
  internal_timer = 0; // reset the timer
}

#endif

#if ESC_EEPROM_EMULATION == 1
UINT16
HW_EepromReload(void)
{
  return 0;
}
#endif

/**
 * ch5.2.2 Read Access
 * @see Section III-ET1100 Hardware Description ch6.3.5 ch6.3.8.1
 */
void
HW_EscRead(MEM_ADDR* pData, UINT16 Address, UINT16 Len)
{
  DISABLE_ESC_INT();
  HW_EscReadIsr(pData, Address, Len);
  ENABLE_ESC_INT();
  return;
}

void
HW_EscReadIsr(MEM_ADDR* pData, UINT16 Address, UINT16 Len)
{
  // Byte-by-byte read per requested style
  UINT8* pTmpData = (UINT8*)pData;

  // Address + command (2-byte addressing)
  uint16_t temp_addr_cmd = (Address << 3) | ESC_RD;
  uint8_t addr_buf[2];
  uint8_t ale_buf[2] = {0};
  addr_buf[0] = (temp_addr_cmd >> 8) & 0xFF;
  addr_buf[1] = (temp_addr_cmd) & 0xFF;

  // Select ESC (active low)
  ESC_SPI_CS_Enable();

  // Transmit address/command; simultaneously read out ALEvent (first two bytes)
  if (HAL_SPI_TransmitReceive(&ESC_SPI_HANDLE, addr_buf, ale_buf, 2, ESC_SPI_TIMEOUT) != HAL_OK) {
    printf("SPI read addr phase failed\n");
  }
  EscALEvent.Byte[0] = ale_buf[0];
  EscALEvent.Byte[1] = ale_buf[1];

  // Read data bytes one-by-one
  for (UINT16 i = Len; i > 0; i--) {
    uint8_t tx = (i == 1) ? 0xFF : 0x00; // last byte: DI shall be 1
    uint8_t rx = 0;
    if (HAL_SPI_TransmitReceive(&ESC_SPI_HANDLE, &tx, &rx, 1, ESC_SPI_TIMEOUT) != HAL_OK) {
      printf("SPI read data failed at index %u\n", (unsigned)(Len - i));
      break;
    }
    *pTmpData++ = rx;
  }

  // Small delay after transmission
  __NOP();

  // Only a single byte was transmitted => wait for ~250ns (9 NOPs as in reference)
  if (Len == 1) {
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
  }

  // Keep CS low for at least 15ns + CLK/2 after transmission, then deselect
  __NOP();
  ESC_SPI_CS_Disable();
  return;
}

/**
 * ET9800 ch5.2.3 Write Access
 *
 * @see Section III-ET1100 Hardware Description ch6.3.5 ch6.3.7
 * @see mcp2515_cmd_write_reg
 * @details
 * compared to HW_EscWriteIsr, it may additionally disable global interrupt
 */
void
HW_EscWrite(MEM_ADDR* pData, UINT16 Address, UINT16 Len)
{
  DISABLE_ESC_INT();
  HW_EscWriteIsr(pData, Address, Len);
  ENABLE_ESC_INT();
  return;
}
void
HW_EscWriteIsr(MEM_ADDR* pData, UINT16 Address, UINT16 Len)
{

  // Byte-by-byte write as requested
  UINT8* pTmpData = (UINT8*)pData;
  VARVOLATILE UINT16 dummy = 0;

  // Address + command (2-byte addressing)
  uint16_t temp_addr_cmd = (Address << 3) | ESC_WR;
  uint8_t addr_buf[2];
  uint8_t ale_buf[2] = {0};
  addr_buf[0] = (temp_addr_cmd >> 8) & 0xFF;
  addr_buf[1] = (temp_addr_cmd) & 0xFF;

  // Select ESC (active low)
  ESC_SPI_CS_Enable();

  // Transmit address/command; simultaneously read out ALEvent (first two bytes)
  if (HAL_SPI_TransmitReceive(&ESC_SPI_HANDLE, addr_buf, ale_buf, 2, ESC_SPI_TIMEOUT) != HAL_OK) {
    printf("SPI write addr phase failed\n");
  }
  EscALEvent.Byte[0] = ale_buf[0];
  EscALEvent.Byte[1] = ale_buf[1];

  // Write data bytes one-by-one
  for (UINT16 i = 0; i < Len; i++) {
    uint8_t rx_byte = 0;
    if (HAL_SPI_TransmitReceive(&ESC_SPI_HANDLE, &pTmpData[i], &rx_byte, 1, ESC_SPI_TIMEOUT) != HAL_OK) {
      printf("SPI write data failed at index %u\n", (unsigned)i);
      break;
    }
    dummy = rx_byte; // consume read-back to avoid unused warning
  }
  (void)dummy;

  // Small delay after transmission
  __NOP();

  // Only a single byte was transmitted => wait for ~250ns (9 NOPs as in reference)
  if (Len == 1) {
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
  }

  // Keep CS low for at least 15ns + CLK/2 after transmission, then deselect
  __NOP();
  ESC_SPI_CS_Disable();
  return;
}
