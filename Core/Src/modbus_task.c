/*
 * modbus_task.c
 *
 *  Created on: 2025年11月29日
 *      Author: 12543
 */

#include <stdio.h>
#include "mb.h"
#include "mbtcp.h"
#include "mbport.h"
#include "cmsis_os.h"   // 用到了 vTaskDelay / osDelay，视你工程风格而定
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usart.h"

#define REG_HOLDING_START  1
#define REG_HOLDING_NREGS  100
#define MODBUS_RTU_SLAVE_ID 1U
#define MODBUS_RTU_BAUDRATE 115200UL
#define MODBUS_RTU1_MASTER_PERIOD_MS 1000U
#define MODBUS_RTU1_MASTER_READ_ADDR 0U
#define MODBUS_RTU1_MASTER_READ_QTY  10U
#define RS485_1_HELLO_PERIOD_MS 1000U
#define RS485_2_HELLO_PERIOD_MS 1000U

USHORT usHoldingBuf[REG_HOLDING_NREGS];
volatile int g_rtu1_master_last_status;
volatile uint32_t g_rtu1_master_request_count;
volatile uint32_t g_rtu1_master_ok_count;
volatile uint32_t g_rtu1_master_error_count;
volatile uint32_t g_rs485_1_hello_tx_count;
volatile int g_rs485_1_hello_last_status;
volatile uint32_t g_rs485_2_hello_tx_count;
volatile int g_rs485_2_hello_last_status;

static uint16_t ModbusCRC16(const uint8_t *data, uint16_t length);
static void ModbusHoldingInit(void);
static void RS485_1_SetTx(uint8_t enable);
static void RS485_2_SetTx(uint8_t enable);
static int ModbusRTU1_ReadHolding(uint8_t slaveId, uint16_t startAddr, uint16_t quantity, uint16_t *pRegs);

void ModbusTCP_Task(void *argument)
{
    eMBErrorCode eStatus;

    eStatus = eMBTCPInit(502);
    if (eStatus != MB_ENOERR)
    {
        printf("eMBTCPInit FAILED, err = %d\r\n", eStatus);
        for(;;) vTaskDelay(1000);
    }
    printf("eMBTCPInit OK\r\n");

    eStatus = eMBEnable();
    if (eStatus != MB_ENOERR)
    {
        printf("eMBEnable FAILED, err = %d\r\n", eStatus);
        for(;;) vTaskDelay(1000);
    }
    printf("eMBEnable OK\r\n");

    for (;;)
    {
        eMBPoll();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void ModbusRTU_TaskWithPort(UCHAR ucPort)
{
    eMBErrorCode eStatus;

    ModbusHoldingInit();

    eStatus = eMBInit(MB_RTU, MODBUS_RTU_SLAVE_ID, ucPort, MODBUS_RTU_BAUDRATE, MB_PAR_NONE, 1);
    if (eStatus != MB_ENOERR)
    {
        for(;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    eStatus = eMBEnable();
    if (eStatus != MB_ENOERR)
    {
        for(;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    for (;;)
    {
        (void)eMBPoll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void ModbusRTU1_Task(void *argument)
{
    (void)argument;
    ModbusRTU_TaskWithPort(3);
}

void ModbusRTU2_Task(void *argument)
{
    (void)argument;
    ModbusRTU_TaskWithPort(1);
}

void ModbusRTU1_MasterTask(void *argument)
{
    TickType_t xLastWakeTime;
    uint16_t masterBuf[MODBUS_RTU1_MASTER_READ_QTY];

    (void)argument;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        g_rtu1_master_request_count++;
        g_rtu1_master_last_status = ModbusRTU1_ReadHolding(MODBUS_RTU_SLAVE_ID,
                                                           MODBUS_RTU1_MASTER_READ_ADDR,
                                                           MODBUS_RTU1_MASTER_READ_QTY,
														   masterBuf);
        if (g_rtu1_master_last_status == 0)
        {
            g_rtu1_master_ok_count++;
        }
        else
        {
            g_rtu1_master_error_count++;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MODBUS_RTU1_MASTER_PERIOD_MS));
    }
}

void RS485_1_HelloTask(void *argument)
{
    TickType_t xLastWakeTime;
    const uint8_t message[] = "RTU1 hello\r\n";
    HAL_StatusTypeDef status;

    (void)argument;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        RS485_1_SetTx(1U);
        status = HAL_UART_Transmit(&huart3, (uint8_t *)message, sizeof(message) - 1U, 100U);
        while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET)
        {
        }
        RS485_1_SetTx(0U);

        g_rs485_1_hello_tx_count++;
        g_rs485_1_hello_last_status = (int)status;

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(RS485_1_HELLO_PERIOD_MS));
    }
}

void RS485_2_HelloTask(void *argument)
{
    TickType_t xLastWakeTime;
    const uint8_t message[] = "RTU2 hello\r\n";
    HAL_StatusTypeDef status;

    (void)argument;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        RS485_2_SetTx(1U);
        status = HAL_UART_Transmit(&huart1, (uint8_t *)message, sizeof(message) - 1U, 100U);
        while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
        {
        }
        RS485_2_SetTx(0U);

        g_rs485_2_hello_tx_count++;
        g_rs485_2_hello_last_status = (int)status;

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(RS485_2_HELLO_PERIOD_MS));
    }
}


eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress,
                 USHORT usNRegs, eMBRegisterMode eMode )
{
    USHORT i;
    USHORT index;

    if ((usAddress >= REG_HOLDING_START) &&
        ((usAddress + usNRegs - 1) <= (REG_HOLDING_START + REG_HOLDING_NREGS - 1)))
    {
        index = usAddress - REG_HOLDING_START;

        if (eMode == MB_REG_READ)
        {
            for (i = 0; i < usNRegs; i++)
            {
                USHORT val = usHoldingBuf[index + i];
                *pucRegBuffer++ = (UCHAR)(val >> 8);
                *pucRegBuffer++ = (UCHAR)(val & 0xFF);
            }
        }
        else if (eMode == MB_REG_WRITE)
        {
            for (i = 0; i < usNRegs; i++)
            {
                usHoldingBuf[index + i] =
                    (((USHORT)pucRegBuffer[0] << 8) | (USHORT)pucRegBuffer[1]);
                pucRegBuffer += 2;
            }
        }
        return MB_ENOERR;
    }
    else
    {
        return MB_ENOREG;
    }
}

static uint16_t ModbusCRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t pos = 0; pos < length; pos++)
    {
        crc ^= data[pos];

        for (uint8_t i = 0; i < 8U; i++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc >>= 1U;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static void ModbusHoldingInit(void)
{
    static uint8_t initialized;

    if (initialized != 0U)
    {
        return;
    }

    for (uint16_t i = 0U; i < 10U; i++)
    {
        usHoldingBuf[i] = (USHORT)(i + 1U);
    }

    initialized = 1U;
}

static void RS485_1_SetTx(uint8_t enable)
{
    HAL_GPIO_WritePin(RS485_1_EN_GPIO_Port, RS485_1_EN_Pin,
                      (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void RS485_2_SetTx(uint8_t enable)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,
                      (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int ModbusRTU1_ReadHolding(uint8_t slaveId, uint16_t startAddr, uint16_t quantity, uint16_t *pRegs)
{
    uint8_t txFrame[8];
    uint8_t rxFrame[5 + (MODBUS_RTU1_MASTER_READ_QTY * 2U)];
    uint16_t crc;
    uint16_t expectedLen;
    HAL_StatusTypeDef status;

    if ((quantity == 0U) || (quantity > MODBUS_RTU1_MASTER_READ_QTY))
    {
        return -1;
    }

    txFrame[0] = slaveId;
    txFrame[1] = 0x03U;
    txFrame[2] = (uint8_t)(startAddr >> 8);
    txFrame[3] = (uint8_t)(startAddr & 0xFFU);
    txFrame[4] = (uint8_t)(quantity >> 8);
    txFrame[5] = (uint8_t)(quantity & 0xFFU);

    crc = ModbusCRC16(txFrame, 6U);
    txFrame[6] = (uint8_t)(crc & 0xFFU);
    txFrame[7] = (uint8_t)(crc >> 8);

    (void)HAL_UART_AbortReceive(&huart3);
    RS485_1_SetTx(1U);
    status = HAL_UART_Transmit(&huart3, txFrame, sizeof(txFrame), 100U);
    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET)
    {
    }
    RS485_1_SetTx(0U);

    if (status != HAL_OK)
    {
        return -2;
    }

    expectedLen = (uint16_t)(5U + (quantity * 2U));
    status = HAL_UART_Receive(&huart3, rxFrame, expectedLen, 200U);
    if (status != HAL_OK)
    {
        return -3;
    }

    crc = ModbusCRC16(rxFrame, expectedLen);
    if (crc != 0U)
    {
        return -4;
    }

    if ((rxFrame[0] != slaveId) || (rxFrame[1] != 0x03U) || (rxFrame[2] != (uint8_t)(quantity * 2U)))
    {
        return -5;
    }

    for(int i=0; i<quantity; i++)
    {
        pRegs[i] = (rxFrame[3+i*2] << 8) | rxFrame[4+i*2];
    }

    return 0;
}
