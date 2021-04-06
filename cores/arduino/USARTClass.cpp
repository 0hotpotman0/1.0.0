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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "USARTClass.h"
#include "Arduino.h"


USARTClass::USARTClass( UART_TypeDef* pUsart, IRQn_Type dwIrq, uint32_t dwId, RingBuffer* pRx_buffer )
{
	_rx_buffer = pRx_buffer;
	_pUsart = pUsart;
	_dwIrq = dwIrq;
	_dwId = dwId;
}

void USARTClass::begin( const uint32_t dwBaudRate )
{
	// Enable USART Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);

	// Configure USART Rx as input floating
	pinMode(RX, ALTERNATE);

	// Configure USART Tx as alternate function push-pull
	pinMode(TX, ALTERNATE);
  
	// USART default configuration
	// USART configured as follow:
	// - BaudRate = (set baudRate as 9600 baud)
	// - Word Length = 8 Bits
	// - One Stop Bit
	// - No parity
	// - Hardware flow control disabled (RTS and CTS signals)
	// - Receive and transmit enabled
	USART_InitStructure.UART_BaudRate = dwBaudRate;
	USART_InitStructure.UART_WordLength = UART_WordLength_8b;
	USART_InitStructure.UART_StopBits = UART_StopBits_1;
	USART_InitStructure.UART_Parity = UART_Parity_No ;
	USART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
	USART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;

	// Configure USART
	UART_Init(_pUsart, &USART_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
  
	// Enable the USARTy Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = _dwIrq;
	NVIC_InitStructure.NVIC_IRQChannelPriority = _dwId;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable USART Receive interrupts
	UART_ITConfig(_pUsart, UART_IT_RXIEN, ENABLE);  //USART_ITConfig(_pUsart, USART_IT_RXNE, ENABLE);
	// Enable UART interrupt in NVIC
	NVIC_EnableIRQ( _dwIrq ) ;

	// Enable the USART
	UART_Cmd(_pUsart, ENABLE);
}


void USARTClass::end( void )
{
	// clear any received data
	_rx_buffer->_iHead = _rx_buffer->_iTail ;

	// Disable UART interrupt in NVIC
	NVIC_DisableIRQ( _dwIrq ) ;

	// Wait for any outstanding data to be sent
	flush();
  
	UART_Cmd(_pUsart, DISABLE);

	// Disable USART Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, DISABLE);
}

int USARTClass::available( void )
{
	return (uint32_t)(SERIAL_BUFFER_SIZE + _rx_buffer->_iHead - _rx_buffer->_iTail) % SERIAL_BUFFER_SIZE ;
}

int USARTClass::peek( void )
{
	if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
	{
		return -1 ;
	}
	return _rx_buffer->_aucBuffer[_rx_buffer->_iTail] ;
}

int USARTClass::read( void )
{
	// if the head isn't ahead of the tail, we don't have any characters
	if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
	{
		return -1 ;
	}

	uint8_t uc = _rx_buffer->_aucBuffer[_rx_buffer->_iTail] ;
	_rx_buffer->_iTail = (unsigned int)(_rx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE ;
	return uc ;
}

void USARTClass::flush( void )
{

}

size_t USARTClass::write( const uint8_t uc_data )
{
	// Send one byte from USART
	UART_SendData(_pUsart, uc_data);

	// Loop until USART DR register is empty
	while(UART_GetFlagStatus(_pUsart, UART_FLAG_TXEMPTY) == RESET);    // MARK     UART_FLAG_TXEPT  (while(USART_GetFlagStatus(_pUsart, USART_FLAG_TXE) == RESET);)

	return 1;
}

void USARTClass::IrqHandler( void )
{
	if(UART_GetITStatus(_pUsart, UART_IT_RXIEN) != RESET)  //if(USART_GetITStatus(_pUsart, USART_IT_RXNE) != RESET)
	{
		// Read one byte from the receive data register
		uint8_t RxBuffer;
	
		RxBuffer = UART_ReceiveData(_pUsart);
		_rx_buffer->store_char( RxBuffer ) ;
	}
}
