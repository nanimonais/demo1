/*
 * FreeModbus event queue port for FreeRTOS.
 */

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "stm32f7xx_hal.h"
#include "mb.h"

#define MB_EVENT_QUEUE_LENGTH 16U

static QueueHandle_t xMBEventQueue;

BOOL xMBPortEventInit(void)
{
    if (xMBEventQueue == NULL)
    {
        xMBEventQueue = xQueueCreate(MB_EVENT_QUEUE_LENGTH, sizeof(eMBEventType));
    }

    return (xMBEventQueue != NULL) ? TRUE : FALSE;
}

void vMBPortEventClose(void)
{
    if (xMBEventQueue != NULL)
    {
        vQueueDelete(xMBEventQueue);
        xMBEventQueue = NULL;
    }
}

BOOL xMBPortEventPost(eMBEventType eEvent)
{
    if (xMBEventQueue == NULL)
    {
        return FALSE;
    }

    if (__get_IPSR() != 0U)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        BaseType_t xResult;

        xResult = xQueueSendFromISR(xMBEventQueue, &eEvent, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        return (xResult == pdPASS) ? TRUE : FALSE;
    }

    return (xQueueSend(xMBEventQueue, &eEvent, portMAX_DELAY) == pdPASS) ? TRUE : FALSE;
}

BOOL xMBPortEventGet(eMBEventType *eEvent)
{
    if (xMBEventQueue == NULL)
    {
        return FALSE;
    }

    return (xQueueReceive(xMBEventQueue, eEvent, portMAX_DELAY) == pdPASS) ? TRUE : FALSE;
}
