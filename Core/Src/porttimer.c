/*
 * FreeModbus RTU t3.5 timer for STM32 HAL.
 * TIM2 is used as a one-shot timer with 50 us ticks.
 */

#include "mb.h"
#include "mbport.h"
#include "tim.h"

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
    uint32_t timerClockHz = prvGetTIM2ClockHz();
    uint32_t prescaler = (timerClockHz / 20000U);

    if ((usTimeOut50us == 0U) || (prescaler == 0U))
    {
        return FALSE;
    }

    usTimerReload50us = usTimeOut50us;

    htim2.Init.Prescaler = (uint32_t)(prescaler - 1U);
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = (uint32_t)(usTimerReload50us - 1U);
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    return (HAL_TIM_Base_Init(&htim2) == HAL_OK) ? TRUE : FALSE;
}

void xMBPortTimersClose(void)
{
    (void)HAL_TIM_Base_Stop_IT(&htim2);
}

void vMBPortTimersEnable(void)
{
    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_AUTORELOAD(&htim2, (uint32_t)(usTimerReload50us - 1U));
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    (void)HAL_TIM_Base_Start_IT(&htim2);
}

void vMBPortTimersDisable(void)
{
    (void)HAL_TIM_Base_Stop_IT(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
}

void vMBPortTimersDelay(USHORT usTimeOutMS)
{
    HAL_Delay(usTimeOutMS);
}

void vMBPortTimerExpiredCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM2) && (pxMBPortCBTimerExpired != NULL))
    {
        (void)pxMBPortCBTimerExpired();
    }
}
