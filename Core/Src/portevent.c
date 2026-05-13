/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    portevent.c
  * @brief   FreeModbus event port for FreeRTOS
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "mb.h"
#include "mbport.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "stm32f7xx_hal.h"

/* Private define ------------------------------------------------------------*/
#define MB_EVENT_QUEUE_LENGTH    16U

/* Private variables ---------------------------------------------------------*/
static QueueHandle_t xMBEventQueue = NULL;

/* Private function prototypes -----------------------------------------------*/
static BaseType_t prvMBPortIsInISR(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  判断当前是否处于中断上下文
  */
static BaseType_t prvMBPortIsInISR(void)
{
    return (__get_IPSR() != 0U) ? pdTRUE : pdFALSE;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 Modbus 事件队列
  */
BOOL xMBPortEventInit(void)
{
    if (xMBEventQueue != NULL)
    {
        vQueueDelete(xMBEventQueue);
        xMBEventQueue = NULL;
    }

    xMBEventQueue = xQueueCreate(
        MB_EVENT_QUEUE_LENGTH,
        sizeof(eMBEventType)
    );

    return (xMBEventQueue != NULL) ? TRUE : FALSE;
}

/**
  * @brief  关闭 Modbus 事件队列
  */
void vMBPortEventClose(void)
{
    if (xMBEventQueue != NULL)
    {
        vQueueDelete(xMBEventQueue);
        xMBEventQueue = NULL;
    }
}

/**
  * @brief  投递 Modbus 事件
  *
  * RTU 接收过程中，事件可能从 USART/TIM 中断路径触发，
  * 所以这里必须区分中断上下文和任务上下文。
  */
BOOL xMBPortEventPost(eMBEventType eEvent)
{
    BaseType_t xResult;

    if (xMBEventQueue == NULL)
    {
        return FALSE;
    }

    if (prvMBPortIsInISR() == pdTRUE)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xResult = xQueueSendFromISR(
            xMBEventQueue,
            &eEvent,
            &xHigherPriorityTaskWoken
        );

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        return (xResult == pdPASS) ? TRUE : FALSE;
    }
    else
    {
        /*
         * 任务上下文不要用 portMAX_DELAY。
         * TCP 回调路径里如果阻塞，会导致网络卡住。
         */
        xResult = xQueueSend(
            xMBEventQueue,
            &eEvent,
            0
        );

        return (xResult == pdPASS) ? TRUE : FALSE;
    }
}

/**
  * @brief  获取 Modbus 事件
  *
  * eMBPoll() 会调用这个函数等待事件。
  */
BOOL xMBPortEventGet(eMBEventType *eEvent)
{
    BaseType_t xResult;

    if ((xMBEventQueue == NULL) || (eEvent == NULL))
    {
        return FALSE;
    }

    xResult = xQueueReceive(
        xMBEventQueue,
        eEvent,
        portMAX_DELAY
    );

    return (xResult == pdPASS) ? TRUE : FALSE;
}
