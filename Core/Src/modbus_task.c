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
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"

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

volatile uint8_t  g_rs485_2_echo_rx_byte;
volatile uint8_t  g_rs485_2_echo_rx_buf[256];
volatile uint16_t g_rs485_2_echo_rx_len;

volatile uint32_t g_rs485_2_echo_rx_count;
volatile uint32_t g_rs485_2_echo_tx_count;
volatile uint32_t g_rs485_2_echo_timeout_count;
volatile uint32_t g_rs485_2_echo_error_count;

volatile int      g_rs485_2_echo_rx_status;
volatile int      g_rs485_2_echo_tx_status;
volatile uint32_t g_rs485_2_echo_uart_error;
volatile uint32_t g_rs485_2_echo_last_tick;

static uint16_t ModbusCRC16(const uint8_t *data, uint16_t length);
static void ModbusHoldingInit(void);
static void RS485_1_SetTx(uint8_t enable);
static void RS485_2_SetTx(uint8_t enable);
static int ModbusRTU1_ReadHolding(uint8_t slaveId, uint16_t startAddr, uint16_t quantity, uint16_t *pRegs);

static

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
        osDelay(1);
    }
}

void ModbusTCP_MasterTask(void *argument)
{
    uint8_t value = 0;       // 当前要发送的值 0-9
    uint8_t sendBuf[12];     // Modbus 请求报文
    uint8_t recvBuf[256];    // Modbus 响应
    int len, sock;
    struct sockaddr_in server_addr;
    ip_addr_t server_ip;
    uint16_t port = 502;

    IP4_ADDR(&server_ip, 192,168,1,14); // 从站 IP

    for (;;)
    {
        // 创建 TCP Socket
        sock = lwip_socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) { osDelay(1000); continue; }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr.s_addr = server_ip.addr;

        if (lwip_connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
        {
            lwip_close(sock);
            osDelay(1000);
            continue;
        }

        // 循环发送寄存器值 0-9 +1
        for (;;)
        {
            // 构造 Modbus 写单寄存器请求（示例）
            // MBAP header: TransactionID=0x0001, ProtocolID=0x0000, Length=0x0006, UnitID=1
            sendBuf[0] = 0x00; sendBuf[1] = 0x01;  // 事务 ID
            sendBuf[2] = 0x00; sendBuf[3] = 0x00;  // 协议 ID
            sendBuf[4] = 0x00; sendBuf[5] = 0x06;  // 长度
            sendBuf[6] = 0x01;                     // Unit ID
            sendBuf[7] = 0x06;                     // 功能码 0x06 写单寄存器
            sendBuf[8] = 0x00; sendBuf[9] = 0x00;  // 地址 0
            sendBuf[10]= 0x00; sendBuf[11]= value; // 数据: 当前 value

            // 发送请求
            len = lwip_send(sock, sendBuf, sizeof(sendBuf), 0);
            if (len <= 0) break; // 断开重连

            // 接收响应
            len = lwip_recv(sock, recvBuf, sizeof(recvBuf), 0);
            if (len <= 0) break; // 断开重连

            // 打印或调试
            printf("Sent value: %d, Response length: %d\n", value, len);

            // 更新 value
            value++;
            if (value > 9) value = 0;

            osDelay(500); // 每 500ms 发送一次
        }

        lwip_close(sock);
        osDelay(1000); // 重连延时
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
    HAL_StatusTypeDef status;
    HAL_StatusTypeDef txStatus;

    uint8_t rx;
    uint8_t frame[256];
    uint16_t frameLen;

    (void)argument;

    /*
     * RS485_2 = USART1
     * TX = PA9
     * RX = PA10
     * DE = PB0
     *
     * DE = 0: 接收模式
     * DE = 1: 发送模式
     */

    RS485_2_SetTx(0U);

    /*
     * 防止之前 Modbus RTU 或其他接收状态残留。
     */
    (void)HAL_UART_Abort(&huart1);
    (void)HAL_UART_AbortReceive(&huart1);
    (void)HAL_UART_AbortTransmit(&huart1);

    g_rs485_2_echo_rx_byte = 0U;
    g_rs485_2_echo_rx_len = 0U;

    g_rs485_2_echo_rx_count = 0U;
    g_rs485_2_echo_tx_count = 0U;
    g_rs485_2_echo_timeout_count = 0U;
    g_rs485_2_echo_error_count = 0U;

    g_rs485_2_echo_rx_status = 0;
    g_rs485_2_echo_tx_status = 0;
    g_rs485_2_echo_uart_error = 0U;
    g_rs485_2_echo_last_tick = 0U;

    for (;;)
    {
        frameLen = 0U;

        /*
         * 一直保持接收模式。
         */
        RS485_2_SetTx(0U);

        /*
         * 等待第 1 个字节。
         * 100ms 超时一次，方便任务持续运行。
         */
        status = HAL_UART_Receive(&huart1, &rx, 1U, 100U);
        g_rs485_2_echo_rx_status = (int)status;

        if (status == HAL_TIMEOUT)
        {
            g_rs485_2_echo_timeout_count++;
            osDelay(1);
            continue;
        }

        if (status != HAL_OK)
        {
            g_rs485_2_echo_error_count++;
            g_rs485_2_echo_uart_error = huart1.ErrorCode;

            (void)HAL_UART_Abort(&huart1);
            (void)HAL_UART_AbortReceive(&huart1);

            RS485_2_SetTx(0U);
            osDelay(1);
            continue;
        }

        /*
         * 收到第 1 个字节。
         */
        frame[frameLen] = rx;
        g_rs485_2_echo_rx_buf[frameLen] = rx;
        frameLen++;

        g_rs485_2_echo_rx_byte = rx;
        g_rs485_2_echo_rx_count++;
        g_rs485_2_echo_last_tick = HAL_GetTick();

        /*
         * 继续接收后续字节。
         * 如果 10ms 内没有新字节，就认为这一帧结束。
         *
         * 这样上位机发送 "123456"，STM32 会收到完整一帧后再回传，
         * 不会收到 1 个字节就立刻抢占总线发送，避免 RS485 半双工冲突。
         */
        while (frameLen < sizeof(frame))
        {
            status = HAL_UART_Receive(&huart1, &rx, 1U, 10U);
            g_rs485_2_echo_rx_status = (int)status;

            if (status == HAL_OK)
            {
                frame[frameLen] = rx;
                g_rs485_2_echo_rx_buf[frameLen] = rx;
                frameLen++;

                g_rs485_2_echo_rx_byte = rx;
                g_rs485_2_echo_rx_count++;
                g_rs485_2_echo_last_tick = HAL_GetTick();

                if ((rx == '\n') || (rx == '\r'))
                {
                    break;
                }
            }
            else if (status == HAL_TIMEOUT)
            {
                break;
            }
            else
            {
                g_rs485_2_echo_error_count++;
                g_rs485_2_echo_uart_error = huart1.ErrorCode;

                (void)HAL_UART_Abort(&huart1);
                (void)HAL_UART_AbortReceive(&huart1);

                break;
            }
        }

        g_rs485_2_echo_rx_len = frameLen;

        /*
         * 收到多少，回传多少。
         */
        if (frameLen > 0U)
        {
            /*
             * 稍微延时，给 USB-RS485 转接器一点方向切换时间。
             */
            osDelay(2);

            RS485_2_SetTx(1U);

            txStatus = HAL_UART_Transmit(&huart1, frame, frameLen, 200U);
            g_rs485_2_echo_tx_status = (int)txStatus;

            /*
             * 等待最后一个字节真正发完，再切回接收。
             */
            while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
            {
            }

            RS485_2_SetTx(0U);

            if (txStatus == HAL_OK)
            {
                g_rs485_2_echo_tx_count++;
            }
            else
            {
                g_rs485_2_echo_error_count++;
                g_rs485_2_echo_uart_error = huart1.ErrorCode;

                (void)HAL_UART_Abort(&huart1);
                RS485_2_SetTx(0U);
            }
        }

        osDelay(1);
    }
}

void RS485_2_RxTask(void *argument)
{

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
