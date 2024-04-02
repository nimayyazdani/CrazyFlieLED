#include <stdint.h>
#include <stdlib.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "deck.h"
#include "param.h"

/* Diodes are connected between MCU pin and VCC
 * so polarity for switching on is reversed.
 */
#define LED_ON LOW
#define LED_OFF HIGH

/* Define which IOs on the expansion deck is used */
#define RED_LED     DECK_GPIO_IO1
#define YELLOW_LED  DECK_GPIO_IO2
#define GREEN_LED   DECK_GPIO_IO3

/* My definitions for PB4, PB5, PB8*/

#define DECK_USING_PB4 (1<<19)
#define DECK_USING_PB5 (1<<20)
#define DECK_USING_PB8 (1<<21)


/* Enumeration for setting different states */
typedef enum {
    tf_off = 0,
    tf_red = 1,
    tf_yellow = 2,
    tf_green = 3,
    tf_all = 4
} TF;
static TF tf;

/* Timer loop and handle */
static xTimerHandle timer;
static void tfTimer(xTimerHandle timer)
{
  digitalWrite(RED_LED, LED_OFF);
  digitalWrite(YELLOW_LED, LED_OFF);
  digitalWrite(GREEN_LED, LED_OFF);
  switch (tf) {
    case tf_green:
      digitalWrite(GREEN_LED, LED_ON);
      break;
    case tf_yellow:
      digitalWrite(YELLOW_LED, LED_ON);
      break;
    case tf_red:
      digitalWrite(RED_LED, LED_ON);
      break;
    case tf_all:
      digitalWrite(RED_LED, LED_ON);
      digitalWrite(YELLOW_LED, LED_ON);
      digitalWrite(GREEN_LED, LED_ON);
      break;
    default:
      /* Keep all LEDs off */
      break;
  }
}

/* Main initialization */
static void tfInit(DeckInfo *info)
{
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  digitalWrite(RED_LED, LED_OFF);
  digitalWrite(YELLOW_LED, LED_OFF);
  digitalWrite(GREEN_LED, LED_OFF);

  timer = xTimerCreate( "tfTimer", M2T(10),
                         pdTRUE, NULL, tfTimer );
  xTimerStart(timer, 100);
}

PARAM_GROUP_START(tf)
PARAM_ADD(PARAM_UINT32, state, &tf)
PARAM_GROUP_STOP(tf)

static const DeckDriver tf_deck = {
  .vid = 0,
  .pid = 0,
  .name = "bcTF",
  .usedGpio = DECK_USING_PB4 | DECK_USING_PB5 | DECK_USING_PB8,
  .init = tfInit,
};

DECK_DRIVER(tf_deck);