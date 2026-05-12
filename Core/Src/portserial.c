/*
 * FreeModbus RTU serial port for STM32 HAL.
 * Port 3 maps to USART3 / RS485_1.
 */

#include "mb.h"
#include "mbport.h"
#include "main.h"
#include "usart.h"

static UART_HandleTypeDef *pxMBUart;
static GPIO_TypeDef *pxMBRs485EnPort;
static uint16_t usMBRs485EnPin;
static uint8_t ucRxByte;
static uint8_t ucTxByte;
static volatile BOOL xRxEnabled;
static volatile BOOL xTxEnabled;

extern void MB_UART8_TestRxCallback(void);

static void prvSetRS485Tx(BOOL xEnable)
{
    if (pxMBRs485EnPort != NULL)
    {
        HAL_GPIO_WritePin(pxMBRs485EnPort, usMBRs485EnPin,
                          xEnable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

BOOL xMBPortSerialInit(UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits,
                       eMBParity eParity, UCHAR ucStopBits)
{
    (void)ulBaudRate;
    (void)ucDataBits;
    (void)eParity;
    (void)ucStopBits;

    if (ucPort == 3U)
    {
        pxMBUart = &huart3;
        pxMBRs485EnPort = RS485_1_EN_GPIO_Port;
        usMBRs485EnPin = RS485_1_EN_Pin;
    }
    else if (ucPort == 1U)
    {
        pxMBUart = &huart1;
        pxMBRs485EnPort = RS485_2_DE_GPIO_Port;
        usMBRs485EnPin = RS485_2_DE_Pin;
    }
    else
    {
        pxMBUart = NULL;
        pxMBRs485EnPort = NULL;
        usMBRs485EnPin = 0U;
        return FALSE;
    }

    xRxEnabled = FALSE;
    xTxEnabled = FALSE;
    prvSetRS485Tx(FALSE);

    return TRUE;
}

void vMBPortClose(void)
{
    xMBPortSerialClose();
}

void xMBPortSerialClose(void)
{
    if (pxMBUart != NULL)
    {
        (void)HAL_UART_AbortReceive_IT(pxMBUart);
        (void)HAL_UART_AbortTransmit_IT(pxMBUart);
    }

    xRxEnabled = FALSE;
    xTxEnabled = FALSE;
    prvSetRS485Tx(FALSE);
}

void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
    xRxEnabled = xRxEnable;
    xTxEnabled = xTxEnable;

    if (pxMBUart == NULL)
    {
        return;
    }

    if (xTxEnable)
    {
        (void)HAL_UART_AbortReceive_IT(pxMBUart);
        prvSetRS485Tx(TRUE);

        if (pxMBFrameCBTransmitterEmpty != NULL)
        {
            (void)pxMBFrameCBTransmitterEmpty();
        }
    }
    else
    {
        prvSetRS485Tx(FALSE);
    }

    if (xRxEnable)
    {
        (void)HAL_UART_Receive_IT(pxMBUart, &ucRxByte, 1U);
    }
    else
    {
        (void)HAL_UART_AbortReceive_IT(pxMBUart);
    }
}

BOOL xMBPortSerialGetByte(CHAR *pucByte)
{
    *pucByte = (CHAR)ucRxByte;
    return TRUE;
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
    if ((pxMBUart == NULL) || (xTxEnabled == FALSE))
    {
        return FALSE;
    }

    ucTxByte = (uint8_t)ucByte;
    return (HAL_UART_Transmit_IT(pxMBUart, &ucTxByte, 1U) == HAL_OK) ? TRUE : FALSE;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((pxMBUart != NULL) && (huart->Instance == pxMBUart->Instance))
    {
        if ((xRxEnabled == TRUE) && (pxMBFrameCBByteReceived != NULL))
        {
            (void)pxMBFrameCBByteReceived();
            (void)HAL_UART_Receive_IT(pxMBUart, &ucRxByte, 1U);
        }
    }
    else if (huart->Instance == UART8)
    {
        MB_UART8_TestRxCallback();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((pxMBUart != NULL) && (huart->Instance == pxMBUart->Instance))
    {
        if ((xTxEnabled == TRUE) && (pxMBFrameCBTransmitterEmpty != NULL))
        {
            (void)pxMBFrameCBTransmitterEmpty();
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((pxMBUart != NULL) && (huart->Instance == pxMBUart->Instance) && (xRxEnabled == TRUE))
    {
        (void)HAL_UART_Receive_IT(pxMBUart, &ucRxByte, 1U);
    }
}
