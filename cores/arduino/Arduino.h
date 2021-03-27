/*
  Arduino.h - Main include file for the Arduino SDK
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef Arduino_h
#define Arduino_h

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "binary.h"
#include "itoa.h"

#ifdef __cplusplus
extern "C"{
#endif

#include <chip.h>
#include "wiring_constants.h"

#define clockCyclesPerMicrosecond() (SystemCoreClock / 1000000L)
#define clockCyclesToMicroseconds(a) (((a) * 1000L) / (SystemCoreClock / 1000L))
#define microsecondsToClockCycles(a) ((a) * (SystemCoreClock / 1000000L))

void yield(void);

extern void setup( void ) ;
extern void loop( void ) ;
extern void TimeTick_Increment( void ) ;
extern uint32_t GetTickCount(void);
extern void writeBootFlag(uint8_t flag);
extern uint8_t readBootFlag(void);
extern void jumpToBootloader(void);
extern void bootloader(void);


#define NOT_A_PORT			0
#define NOT_AN_INTERRUPT	(-1)

typedef enum _EExt_Interrupts
{
  EXTERNAL_INT_0=0,
  EXTERNAL_INT_1=1,
  EXTERNAL_INT_2=2,
  EXTERNAL_INT_3=3,
  EXTERNAL_INT_4=4,
  EXTERNAL_INT_5=5,
  EXTERNAL_INT_6=6,
  EXTERNAL_INT_7=7,
  EXTERNAL_NUM_INTERRUPTS
} EExt_Interrupts ;

typedef void (*voidFuncPtr)(void) ;

/* Define attribute */
#if defined   ( __CC_ARM   ) /* Keil uVision 4 */
    #define WEAK (__attribute__ ((weak)))
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
    #define WEAK __weak
#elif defined (  __GNUC__  ) /* GCC CS */
    #define WEAK __attribute__ ((weak))
#endif

/**
 * Pin Attributes to be OR-ed
 */
#define PIN_ATTR_COMBO         (1UL<<0)
#define PIN_ATTR_ANALOG        (1UL<<1)
#define PIN_ATTR_DIGITAL       (1UL<<2)
#define PIN_ATTR_PWM           (1UL<<3)
#define PIN_ATTR_TIMER         (1UL<<4)

/* Types used for the tables below */
typedef struct _PinDescription
{
  GPIO_TypeDef* pPort ;
  uint32_t ulPin ;
  uint32_t ulPeripheral ;
  uint8_t ulPinOType;
  uint8_t ulPinPuPd;
  uint8_t ulPinSpeed;
  uint8_t ulPinAFMode;
} PinDescription ;

/* Pins table to be instanciated into variant.cpp */
extern const PinDescription g_APinDescription[] ;

#ifdef __cplusplus
} // extern "C"

#include "WCharacter.h"
#include "WString.h"
#include "Tone.h"
#include "WMath.h"
#include "HardwareSerial.h"
#include "wiring_pulse.h"

#endif // __cplusplus

// Include board variant
#include "variant.h"

#include "wiring.h"
#include "wiring_digital.h"
#include "wiring_analog.h"
#include "wiring_shift.h"
#include "WInterrupts.h"

// USB Device
#define USB_VID            0x2341 // arduino LLC vid
#define USB_PID_LEONARDO   0x0034
#define USB_PID_MICRO      0x0035
#define USB_PID_DUE        0x003E
#include "USB/USBDesc.h"
#include "USB/USBCore.h"
#include "USB/USBAPI.h"

#endif // Arduino_h
