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

#include "Arduino.h"

#ifdef __cplusplus
 extern "C" {
#endif

void pinMode( uint32_t ulPin, uint32_t ulMode )
{
	if(g_APinDescription[ulPin].pPort == NULL)
	{
		return;
	}
	GPIO_TypeDef *gpio_port = g_APinDescription[ulPin].pPort;
	uint16_t gpio_pin = 1 << (g_APinDescription[ulPin].ulPin & 0xF);

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(g_APinDescription[ulPin].ulPeripheral,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = gpio_pin;
	GPIO_InitStructure.GPIO_Mode = g_APinDescription[ulPin].ulPinPuPd;
	
	switch(ulMode)
	{
		case INPUT:
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			break ;
		
		case INPUT_PULLUP:
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
			break;
			
		case INPUT_PULLDOWN:
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
			break;
			
		case OUTPUT:
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
			break ;
		
		case ALTERNATE:
			GPIO_PinAFConfig(gpio_port, g_APinDescription[ulPin].ulPin & 0xF, g_APinDescription[ulPin].ulPinAFMode);
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			break;
		case ANALOG:
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
			break;

        default:
        	break ;
	}

	if(ulMode != ANALOG)
	{
		//GPIO_InitStructure.GPIO_Mode = g_APinDescription[ulPin].ulPinPuPd;
		GPIO_InitStructure.GPIO_Speed = g_APinDescription[ulPin].ulPinSpeed;
	}
	
	GPIO_Init(gpio_port, &GPIO_InitStructure);
}

void digitalWrite(uint32_t ulPin, uint32_t ulVal)
{
	if(g_APinDescription[ulPin].pPort == NULL)
	{
		return;
	}
	if(ulVal == HIGH)
	{
		g_APinDescription[ulPin].pPort->BSRR = 1 << (g_APinDescription[ulPin].ulPin & 0xF);
	}
	else
	{
		g_APinDescription[ulPin].pPort->BRR = 1 << (g_APinDescription[ulPin].ulPin & 0xF);
	}
}

int digitalRead(uint32_t ulPin)
{
	if(g_APinDescription[ulPin].pPort == NULL)
	{
		return;
	}
	if(g_APinDescription[ulPin].pPort->IDR & (1 << (g_APinDescription[ulPin].ulPin & 0xF)))
	{
		return HIGH;
	}
	return LOW;
}

#ifdef __cplusplus
}
#endif

