/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_GROVE_TWO_
#define _VARIANT_GROVE_TWO_

// Frequency of the board main oscillator
#define VARIANT_MAINOSC		8000000
// Master clock frequency
#define VARIANT_MCK			48000000

#include "Arduino.h"
#ifdef __cplusplus
#include "USARTClass.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif

// Libc porting layers
#if defined (  __GNUC__  ) // GCC CS3
#include <syscalls.h> // RedHat Newlib minimal stub
#endif

#define NONE ((uint8_t)0xFF)
 
// Number of pins defined in PinDescription array
#define PINS_COUNT           (15u)

#define digitalPinToInterrupt(p)  p

#define PA0  	4
#define PA1		5
#define PA2  	1
#define PA3  	0
#define PA4  	10
#define PA5  	11
#define PA6  	12
#define PA7  	13
#define PA9  	2
#define PA10 	3
#define PA13 	6
#define PA14 	7
#define PB1  	14
#define PF0  	8
#define PF1  	9

static const uint8_t RX = 0;
static const uint8_t TX = 1;

static const uint8_t SCL = 2;
static const uint8_t SDA = 3;

static const uint8_t SS   = 10;
static const uint8_t SCK  = 11;
static const uint8_t MISO = 12;
static const uint8_t MOSI = 13;

static const uint8_t A0 = 4;
static const uint8_t A1 = 5;
static const uint8_t A2 = 11;
static const uint8_t A3 = 12;
static const uint8_t A4 = 13;
static const uint8_t A5 = 14;


extern const unsigned int pin_ADC_Channel[];
extern const TIM_TypeDef* pin_TIM[];
extern const uint16_t pin_TIM_Channel[];


#define ADC_RESOLUTION		12
#define PWM_FREQUENCY		40000
#define PWM_RESOLUTION		8
#define WIRE_ISR_HANDLER	I2C1_IRQHandler


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

extern USARTClass Serial;

#endif

#endif
