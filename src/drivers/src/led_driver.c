/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * led_driver.c - led_driver/Buzzer driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 */

#include <stdbool.h>

/* ST includes */
#include "stm32fxxx.h"

#include "led_driver.h"

//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

// HW defines
#define led_driver_TIM_PERIF       RCC_APB1Periph_TIM5
#define led_driver_TIM             TIM5
#define led_driver_TIM_DBG         DBGMCU_TIM5_STOP
#define led_driver_TIM_SETCOMPARE  TIM_SetCompare2
#define led_driver_TIM_GETCAPTURE  TIM_GetCapture2

#define led_driver_GPIO_POS_PERIF         RCC_AHB1Periph_GPIOA
#define led_driver_GPIO_POS_PORT          GPIOA
#define led_driver_GPIO_POS_PIN           GPIO_Pin_2 // TIM5_CH3
#define led_driver_GPIO_AF_POS_PIN        GPIO_PinSource2
#define led_driver_GPIO_AF_POS            GPIO_AF_TIM5

#define led_driver_GPIO_NEG_PERIF         RCC_AHB1Periph_GPIOA
#define led_driver_GPIO_NEG_PORT          GPIOA
#define led_driver_GPIO_NEG_PIN           GPIO_Pin_3 // TIM5_CH4
#define led_driver_GPIO_AF_NEG_PIN        GPIO_PinSource3
#define led_driver_GPIO_AF_NEG            GPIO_AF_TIM5

#define led_driver_PWM_BITS      (8)
#define led_driver_PWM_PERIOD    ((1<<led_driver_PWM_BITS) - 1)
#define led_driver_PWM_PRESCALE  (0)

/* This should be calculated.. */
#define led_driver_BASE_FREQ (329500)

static bool isInit = false;

/* Public functions */

void led_driverInit()
{
  if (isInit)
    return;

  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(led_driver_GPIO_POS_PERIF | led_driver_GPIO_NEG_PERIF, ENABLE);
  RCC_APB1PeriphClockCmd(led_driver_TIM_PERIF, ENABLE);

  // Configure the GPIO for the timer output
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = led_driver_GPIO_POS_PIN;
  GPIO_Init(led_driver_GPIO_POS_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = led_driver_GPIO_NEG_PIN;
  GPIO_Init(led_driver_GPIO_NEG_PORT, &GPIO_InitStructure);

  //Map timers to alternate functions
  GPIO_PinAFConfig(led_driver_GPIO_POS_PORT, led_driver_GPIO_AF_POS_PIN, led_driver_GPIO_AF_POS);
  GPIO_PinAFConfig(led_driver_GPIO_NEG_PORT, led_driver_GPIO_AF_NEG_PIN, led_driver_GPIO_AF_NEG);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = led_driver_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = led_driver_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(led_driver_TIM, &TIM_TimeBaseStructure);

  // PWM channels configuration
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure OC3
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC3Init(led_driver_TIM, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(led_driver_TIM, TIM_OCPreload_Enable);

  // Configure OC4 inverted
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC4Init(led_driver_TIM, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(led_driver_TIM, TIM_OCPreload_Enable);

  //Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(led_driver_TIM, ENABLE);
  TIM_SetCompare3(led_driver_TIM, 0x00);
  TIM_SetCompare4(led_driver_TIM, 0x00);

  //Enable the timer
  TIM_Cmd(led_driver_TIM, ENABLE);

  isInit = true;
}

bool led_driverTest(void)
{
  return isInit;
}

void ledDriverSetBrightness(uint8_t ratio1, uint8_t ratio2,)
{
  TIM_SetCompare3(led_driver_TIM, ratio1);
  TIM_SetCompare4(led_driver_TIM, ratio2);
}

void led_driverSetFreq(uint16_t freq)
{
  TIM_PrescalerConfig(led_driver_TIM, (led_driver_BASE_FREQ/freq), TIM_PSCReloadMode_Update);
}
