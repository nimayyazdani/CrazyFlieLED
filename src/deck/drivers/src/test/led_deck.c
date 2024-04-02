/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 BitCraze AB
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
 * buzzdeck.c - Deck driver for the buzzer deck
 */
#include <stdint.h>
#include <stdlib.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "deck.h"
#include "param.h"

#include "led_driver.h"  

// Defines for LED states
#define LED_ON  HIGH
#define LED_OFF LOW

// Assuming LEDs are connected to the same pins used for motor control
#define LED_1_PIN  DECK_GPIO_IO1
#define LED_2_PIN  DECK_GPIO_IO2

// Enumeration for setting different LED states
typedef enum {
	off = 0,
    on = 1,
    dim = 2  
} LEDState;
static LEDState ledState;

// Params for controlling the brightness of LEDs 1 and 2 (0-255 for PWM)
static uint8_t brightness1;
static uint8_t brightness2;

// Function to adjust LED brightness using PWM
static void ledControlPWM(uint8_t power1, uint8_t power2)
{
	// Assume ledDriverSetBrightness is modified to control individual LEDs
	ledDriverSetBrightness(power1, power2);
  led_driverSetFreq(10000)
}

static void ledsOff()
{
	// Turn both LEDs off
	ledDriverSetBrightness(0, 0);
}

/* Timer loop and handle */
static xTimerHandle timer;
static void ledsTimer(xTimerHandle timer)

{
  switch (leds) {
     case off:
      ledsOff();
      break;
    case bright:
      wheelDeckPWM(brightness1,brightness2);
      digitalWrite(LED_1_PIN, LED_ON);
      digitalWrite(LED_2_PIN, LED_ON);
      break;
    case dim:
      wheelDeckPWM(brightness1,brightness2);
      digitalWrite(LED_1_PIN, LED_OFF);
      digitalWrite(LED_2_PIN, LED_OFF);
      break;
    default:
      /* Keep all LEDs off */
      break;
  }
}

static void ledDeckInit(DeckInfo *info)
{
	ledDriverInit();  // Initialize the LED driver

  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  digitalWrite(LED_1_PIN, LED_ON);
  digitalWrite(LED_2_PIN, LED_ON);




	timer = xTimerCreate("ledsTimer", M2T(10), pdTRUE, NULL, ledsTimer);
	xTimerStart(timer, 100);
}

PARAM_GROUP_START(leds)
PARAM_ADD(PARAM_UINT8, state, &ledState)  // Might not be needed if controlling brightness directly
PARAM_ADD(PARAM_UINT8, brightness_1, &brightness1)
PARAM_ADD(PARAM_UINT8, brightness_2, &brightness2)
PARAM_GROUP_STOP(leds)

static const DeckDriver led_deck = {
	.vid = 0,
	.pid = 0,
	.name = "ledDeck",
	// Assume using the timer and GPIO pins for PWM control of LEDs
	.usedPeriph = DECK_USING_TIMER5,
	.usedGpio = DECK_USING_TX2 | DECK_USING_RX2 | DECK_USING_PB5 | DECK_USING_PB8,
	.init = ledDeckInit,
};

DECK_DRIVER(led_deck);

