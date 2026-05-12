/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "modbus_task.h"
#include "spi.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENABLE_UART_TEST_TASK   0
#define ENABLE_SPI_TEST_TASK    0
#define ENABLE_MODBUS_TCP_TASK  1
#define ENABLE_MODBUS_RTU1_TASK 0
#define ENABLE_MODBUS_RTU2_TASK 0
#define ENABLE_MODBUS_RTU1_MASTER_TASK 0
#define ENABLE_RS485_1_HELLO_TASK 0
#define ENABLE_RS485_2_HELLO_TASK 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
const osThreadAttr_t modbusTCP_attributes = {
    .name = "ModbusTCP",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal,
  };

const osThreadAttr_t modbusRTU1_attributes = {
    .name = "ModbusRTU1",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal,
  };

const osThreadAttr_t modbusRTU2_attributes = {
    .name = "ModbusRTU2",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal,
  };

const osThreadAttr_t rs4851Hello_attributes = {
    .name = "RS485_1_TX",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };

const osThreadAttr_t modbusRTU1Master_attributes = {
    .name = "RTU1Master",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };

const osThreadAttr_t rs4852Hello_attributes = {
    .name = "RS485_2_TX",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };

const osThreadAttr_t uartTx_attributes = {
    .name = "UartTx",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityLow,
  };

const osThreadAttr_t spiTest_attributes = {
    .name = "SpiTest",
    .stack_size = 768 * 4,
    .priority = (osPriority_t) osPriorityLow,
  };

static volatile uint8_t uart8_rx_buffer[128];
static volatile uint16_t uart8_rx_len;
static volatile uint8_t uart8_rx_ready;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void UartTxTask(void *argument);
static void SpiTestTask(void *argument);
uint8_t *MB_UART8_TestRxBuffer(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
#if ENABLE_MODBUS_TCP_TASK
  osThreadNew(ModbusTCP_Task, NULL, &modbusTCP_attributes);
#endif

#if ENABLE_MODBUS_RTU1_TASK
  osThreadNew(ModbusRTU1_Task, NULL, &modbusRTU1_attributes);
#endif

#if ENABLE_MODBUS_RTU2_TASK
  osThreadNew(ModbusRTU2_Task, NULL, &modbusRTU2_attributes);
#endif

#if ENABLE_MODBUS_RTU1_MASTER_TASK
  osThreadNew(ModbusRTU1_MasterTask, NULL, &modbusRTU1Master_attributes);
#endif

#if ENABLE_RS485_1_HELLO_TASK
  osThreadNew(RS485_1_HelloTask, NULL, &rs4851Hello_attributes);
#endif

#if ENABLE_RS485_2_HELLO_TASK
  osThreadNew(RS485_2_HelloTask, NULL, &rs4852Hello_attributes);
#endif

#if ENABLE_UART_TEST_TASK
  HAL_UART_Receive_IT(&huart8, MB_UART8_TestRxBuffer(), 1);
  osThreadNew(UartTxTask, NULL, &uartTx_attributes);
#endif

#if ENABLE_SPI_TEST_TASK
  osThreadNew(SpiTestTask, NULL, &spiTest_attributes);
#endif

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void UartTxTask(void *argument)
{
  const uint8_t message[] = "UART hello\r\n";

  (void)argument;

  for (;;)
  {
    HAL_UART_Transmit(&huart8, (uint8_t *)message, sizeof(message) - 1U, HAL_MAX_DELAY);
    osDelay(5000);
  }
}

static void SpiTestTask(void *argument)
{
  const uint8_t tx[] = "SPI hello";
  uint8_t rx[sizeof(tx)] = {0};

  (void)argument;

  for (;;)
  {
    memset(rx, 0, sizeof(rx));

    HAL_GPIO_WritePin(ESP_SPI_CS_GPIO_Port, ESP_SPI_CS_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, sizeof(tx) - 1U, 100);
    HAL_GPIO_WritePin(ESP_SPI_CS_GPIO_Port, ESP_SPI_CS_Pin, GPIO_PIN_SET);

    if (uart8_rx_ready != 0U)
    {
      taskENTER_CRITICAL();
      uart8_rx_len = 0U;
      uart8_rx_ready = 0U;
      taskEXIT_CRITICAL();
    }

    osDelay(3000);
  }
}

uint8_t *MB_UART8_TestRxBuffer(void)
{
  static uint8_t uart8_rx_byte;

  return &uart8_rx_byte;
}

void MB_UART8_TestRxCallback(void)
{
  uint8_t uart8_rx_byte = *MB_UART8_TestRxBuffer();

  if (uart8_rx_len < sizeof(uart8_rx_buffer))
  {
    uart8_rx_buffer[uart8_rx_len++] = uart8_rx_byte;
  }

  if ((uart8_rx_byte == '\n') || (uart8_rx_len >= sizeof(uart8_rx_buffer)))
  {
    uart8_rx_ready = 1U;
  }

  HAL_UART_Receive_IT(&huart8, MB_UART8_TestRxBuffer(), 1);
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
