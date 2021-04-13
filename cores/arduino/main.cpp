/*
  main.cpp - Main loop for Arduino sketches
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

#define ARDUINO_MAIN
#include "Arduino.h"

__IO uint32_t TimingMillis;

void TimeTick_Increment(void)
{
  TimingMillis++;
}

uint32_t GetTickCount(void)
{
  return TimingMillis;
}

/*
 * \brief Main entry point of Arduino application
 */

int main( void )
{

	init();

	delay(1);

#if defined(USBCON)
//	USBDevice.attach();
#endif

	bootloader();

	setup();

	for (;;)
	{
		loop();
		if (serialEventRun) serialEventRun();
	}

	return 0;
}

/*****************************************************************************************************************/
/*****************************************************************************************************************/
/*****************************************************************************************************************/
/*****************************************************************************************************************/

// void UartInit_Loop(void);
// void UartSendGroup(u8 *buf, u16 len);
// void Uart1RxTest(UART_TypeDef *UARTx);
// unsigned char inbyte(UART_TypeDef *UARTx);
// int KKK;
// char BBB;
// #define LIGHT_LED_PIN_NUM		PA1
// char printBuf[100];

// int main(void)
// {
//   UartInit_Loop();
//   while (1)
//   {
//     Uart1RxTest(UART1);
//   }
// }

// void UartInit_Loop(void)
// {

//   //GPIO�˿�����
//   GPIO_InitTypeDef GPIO_InitStructure;
//   UART_InitTypeDef UART_InitStructure;

//   RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE); //ʹ��UART1ʱ��
//   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);   //����GPIOAʱ��
//   //UART ��ʼ������
//   GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
//   GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

//   UART_InitStructure.UART_BaudRate = 115200;                                   //���ڲ�����
//   UART_InitStructure.UART_WordLength = UART_WordLength_8b;                     //�ֳ�Ϊ8λ���ݸ�ʽ
//   UART_InitStructure.UART_StopBits = UART_StopBits_1;                          //һ��ֹͣλ
//   UART_InitStructure.UART_Parity = UART_Parity_No;                             //����żУ��λ
//   UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None; //��Ӳ������������
//   UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;                  //�շ�ģʽ

//   UART_Init(UART1, &UART_InitStructure); //��ʼ������1
//   UART_Cmd(UART1, ENABLE);               //ʹ�ܴ���1

//   //UART1_TX   GPIOA.9
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
//   GPIO_Init(GPIOA, &GPIO_InitStructure);          //��ʼ��GPIOA.9

//   //UART1_RX	  GPIOA.10��ʼ��
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;            //PA10
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
//   GPIO_Init(GPIOA, &GPIO_InitStructure);                //��ʼ��GPIOA.10

//   UartSendGroup((u8 *)printBuf, sprintf(printBuf, "UART OK!\r\n"));
// }

// void Uart1RxTest(UART_TypeDef *UARTx)
// {
//   unsigned char temp;
//   BBB = analogRead(PA4);
//   KKK = analogRead(~PA4);
//   temp = inbyte(UARTx);
  
//   if (temp != 0)
//   {
    
//     // UartSendGroup((u8 *)printBuf, sprintf(printBuf, "goodgood:%c\r\n", temp));
//     // delay(1000);
//     UartSendGroup((u8 *)printBuf, sprintf(printBuf, "bbbbbb:%d\r\n", BBB));
//     sprintf(printBuf, "bbbbbb:%d\r\n", BBB);
//     // delay(1000);
//     UartSendGroup((u8 *)printBuf, sprintf(printBuf, "KKKK:%d\r\n", KKK));
//     sprintf(printBuf, "KKKK:%d\r\n", KKK);

//     // delay(1000);

//   }
// }

// unsigned char inbyte(UART_TypeDef *UARTx)
// {
//   unsigned char temp;

//   for(int i = 0; i < 10; i++)
//   {
//     if (UART_GetITStatus(UARTx, UART_IT_RXIEN))
//     {
//       UART_ClearITPendingBit(UARTx, UART_IT_RXIEN); //��������ж�λ
//       break;
//     }
//   }
//   temp = (uint8_t)UART_ReceiveData(UARTx);
//   if (temp == 0xd)
//   { //���������յ�����
//     return 0;
//   }
//   return temp;
// }

// void UartSendByte(u8 dat)
// {
//   UART_SendData(UART1, dat);
//   while (!UART_GetFlagStatus(UART1, UART_FLAG_TXEPT))
//     ;
// }

// void UartSendGroup(u8 *buf, u16 len)
// {
//   while (len--)
//     UartSendByte(*buf++);
// }