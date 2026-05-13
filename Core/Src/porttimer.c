/*
 * FreeModbus RTU t3.5 timer for STM32 HAL.
 * TIM2 is used as a one-shot timer with 50 us ticks.
 */

#include "mb.h"
#include "mbport.h"
#include "tim.h"

/*
 * Debug 观察变量
 */
volatile uint32_t g_mb_t35_init_count;
volatile uint32_t g_mb_t35_enable_count;
volatile uint32_t g_mb_t35_disable_count;
volatile uint32_t g_mb_t35_expired_count;
volatile int      g_mb_t35_start_status;
volatile int      g_mb_t35_stop_status;

static USHORT usTimerReload50us;

static uint32_t prvGetTIM2ClockHz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_HCLK_DIV1)
    {
        pclk1 *= 2U;
    }

    return pclk1;
}

BOOL xMBPortTimersInit(USHORT usTimeOut50us)
{
    uint32_t timerClockHz;
    uint32_t prescaler;
    HAL_StatusTypeDef status;

    if (usTimeOut50us == 0U)
    {
        return FALSE;
    }

    timerClockHz = prvGetTIM2ClockHz();

    /*
     * 50us 一个 tick = 20kHz
     */
    prescaler = timerClockHz / 20000U;

    if (prescaler == 0U)
    {
        return FALSE;
    }

    usTimerReload50us = usTimeOut50us;

    htim2.Init.Prescaler = prescaler - 1U;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = usTimerReload50us - 1U;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    status = HAL_TIM_Base_Init(&htim2);

    g_mb_t35_init_count++;

    return (status == HAL_OK) ? TRUE : FALSE;
}

void xMBPortTimersClose(void)
{
    (void)HAL_TIM_Base_Stop_IT(&htim2);
}

void vMBPortTimersEnable(void)
{
    HAL_StatusTypeDef stopStatus;
    HAL_StatusTypeDef startStatus;

    g_mb_t35_enable_count++;

    /*
     * 关键点：
     * 不能只用 __HAL_TIM_DISABLE()。
     * 必须 HAL_TIM_Base_Stop_IT()，让 htim2.State 回到 READY。
     * 否则连续收到多个字节时，HAL_TIM_Base_Start_IT() 会因为 State=BUSY 失败。
     */
    stopStatus = HAL_TIM_Base_Stop_IT(&htim2);
    g_mb_t35_stop_status = (int)stopStatus;

    __HAL_TIM_SET_AUTORELOAD(&htim2, (uint32_t)(usTimerReload50us - 1U));
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    startStatus = HAL_TIM_Base_Start_IT(&htim2);
    g_mb_t35_start_status = (int)startStatus;
}

void vMBPortTimersDisable(void)
{
    HAL_StatusTypeDef stopStatus;

    g_mb_t35_disable_count++;

    stopStatus = HAL_TIM_Base_Stop_IT(&htim2);
    g_mb_t35_stop_status = (int)stopStatus;

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
}

void vMBPortTimersDelay(USHORT usTimeOutMS)
{
    HAL_Delay(usTimeOutMS);
}

void vMBPortTimerExpiredCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM2) && (pxMBPortCBTimerExpired != NULL))
    {
        g_mb_t35_expired_count++;

        (void)pxMBPortCBTimerExpired();
    }
}
