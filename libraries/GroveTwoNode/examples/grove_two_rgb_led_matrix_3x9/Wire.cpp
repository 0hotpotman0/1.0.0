/*
 * TwoWire.h
 * Copyright (c) 2011 Cristian Maglie <c.maglie@bug.st>.
 * All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

extern "C" {
#include <string.h>
}

#include "Wire.h"

#define EVENT_TIMEOUT       100

#ifdef GPIO_I2C
#define I2C_BUS_CHECK
#define WIRE_I2C_DELAY			1
#define WIRE_I2C_WAIT()			delayMicroseconds(WIRE_I2C_DELAY)
#define WIRE_I2C_SDA_IN()		pinMode(SDA, INPUT)
#define WIRE_I2C_SDA_OUT()		pinMode(SDA, OUTPUT)
#define WIRE_I2C_SDA_READ()		digitalRead(SDA)
#define WIRE_I2C_SCL_READ()		digitalRead(SCL)
#define WIRE_I2C_SCL_LOW()		digitalWrite(SCL, LOW)
#define WIRE_I2C_SCL_HIGH()		digitalWrite(SCL, HIGH)
#define WIRE_I2C_SDA_LOW()		digitalWrite(SDA, LOW)
#define WIRE_I2C_SDA_HIGH()		digitalWrite(SDA, HIGH)

#define	START_DELAY				20

#endif


TwoWire::TwoWire(I2C_TypeDef *_twi) :
	twi(_twi), rxBufferIndex(0), rxBufferLength(0), txAddress(0),
			txBufferLength(0), srvBufferIndex(0), srvBufferLength(0), status(
					UNINITIALIZED){
	// Empty
}

void TwoWire::begin(void) {

#ifndef GPIO_I2C

	if (onBeginCallback)
		onBeginCallback();

	if (twi==I2C1)
	{
		RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK); // 48 MHz
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

		pinMode(SDA, ALTERNATE);
		pinMode(SCL, ALTERNATE);
	}

	I2C_DeInit(twi);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	if (status == UNINITIALIZED)
	{
		I2C_InitStructure.I2C_OwnAddress1 = 0x00;
		status = MASTER_IDLE;
	}
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	// I2C_InitStructure.I2C_Timing = 0xb0420f13; // 100 KHz
	// I2C_InitStructure.I2C_Timing = 0x50300707; // 400 KHz
	 I2C_InitStructure.I2C_Timing = 0x50330309; // 400 KHz
	// I2C_InitStructure.I2C_Timing = 0x50100103; // 1000 KHz
	I2C_Init(twi, &I2C_InitStructure);

	I2C_Cmd(twi, ENABLE);
#endif

}

void TwoWire::begin(uint8_t address) {
	if (onBeginCallback)
		onBeginCallback();
	status = SLAVE_IDLE;
	if (twi==I2C1)
	{
		RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK); // 48 MHz
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

		pinMode(SDA, ALTERNATE);
		pinMode(SCL, ALTERNATE);
	}

	I2C_DeInit(twi);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x0A;
	I2C_InitStructure.I2C_OwnAddress1 = address;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	// I2C_InitStructure.I2C_Timing = 0xb0420f13; // 100 KHz
	I2C_InitStructure.I2C_Timing = 0x50330309; // 400 KHz
	//I2C_InitStructure.I2C_Timing = 0x50100103; // 1000 KHz
	I2C_Init(twi, &I2C_InitStructure);
	
//	I2C_StretchClockCmd(twi, DISABLE);

	NVIC_SetPriority(I2C1_IRQn, 1);
	NVIC_EnableIRQ(I2C1_IRQn);
	I2C_Cmd(twi, ENABLE);
	I2C_ITConfig(twi, I2C_IT_ADDRI|I2C_IT_ERRI, ENABLE);
}

void TwoWire::begin(int address) {
	address <<= 1;
	begin((uint8_t) address);
}

void TwoWire::end(void)
{
#ifdef GPIO_I2C
	GPIO_TypeDef *gpio_port = g_APinDescription[SCL].pPort;
	uint16_t gpio_pin = 1 << (g_APinDescription[SCL].ulPin & 0xF);
	GPIO_InitTypeDef GPIO_InitStructure;
	int value;

//	if(status == SLAVE_IDLE)return;
	
	if(WIRE_I2C_SCL_READ() && WIRE_I2C_SDA_READ())return;
	
	I2C_DeInit(twi);
	onReceiveCallback = NULL;
	onRequestCallback = NULL;
	status = UNINITIALIZED;

	WIRE_I2C_SDA_HIGH();
	WIRE_I2C_SCL_HIGH();

	RCC_AHBPeriphClockCmd(g_APinDescription[SCL].ulPeripheral,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = gpio_pin;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(gpio_port, &GPIO_InitStructure);

	gpio_port = g_APinDescription[SDA].pPort;
	gpio_pin = 1 << (g_APinDescription[SDA].ulPin & 0xF);

	RCC_AHBPeriphClockCmd(g_APinDescription[SDA].ulPeripheral,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = gpio_pin;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(gpio_port, &GPIO_InitStructure);

	WIRE_I2C_SDA_HIGH();
	WIRE_I2C_SCL_HIGH();
	delay(1);
	WIRE_I2C_SDA_LOW();
	delay(1);
	WIRE_I2C_SDA_HIGH();
	delay(10);

#endif

}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
	address <<= 1;
	if (quantity > BUFFER_LENGTH)
		quantity = BUFFER_LENGTH;
	uint32_t _millis;

	_millis = millis();
	while(I2C_GetFlagStatus(twi, I2C_FLAG_BUSY) != RESET)
	{
		if(EVENT_TIMEOUT < (millis() - _millis)) return 0;
	}

	I2C_TransferHandling(twi, address, quantity, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	uint8_t *pBuffer = rxBuffer;
	uint8_t numByteToRead = quantity;
	uint8_t bytesRead = 0;
	/* While there is data to be read */
	while(numByteToRead)
	{
		_millis = millis();
		while(I2C_GetFlagStatus(twi, I2C_FLAG_RXNE) == RESET)
		{
			if(EVENT_TIMEOUT < (millis() - _millis)) return 0;
		}
		/* Read a byte from the Slave */
		*pBuffer = I2C_ReceiveData(twi);

		bytesRead++;

		/* Point to the next location where the byte read will be saved */
		pBuffer++;

		/* Decrement the read bytes counter */
		numByteToRead--;
	}

	_millis = millis();
	while(I2C_GetFlagStatus(twi, I2C_FLAG_STOPF) == RESET)
	{
		if(EVENT_TIMEOUT < (millis() - _millis)) return 0;
	}

	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = bytesRead;

	return bytesRead;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) sendStop);
}

void TwoWire::beginTransmission(uint8_t address) {
	status = MASTER_SEND;

	// save address of target and empty buffer
	txAddress = address;
	txBufferLength = 0;
}

void TwoWire::beginTransmission(int address) {
	address <<= 1;
	beginTransmission((uint8_t) address);
}


#ifdef GPIO_I2C


int i2cPioSendByte(uint8_t txd)
{
	int i, ack, value;

	for(i = 0; i < 8; i ++)
	{
		WIRE_I2C_SCL_LOW();
		WIRE_I2C_WAIT();
		if(txd & 0x80)
		{
			WIRE_I2C_SDA_HIGH();
			WIRE_I2C_WAIT();
#ifdef I2C_BUS_CHECK
			value = WIRE_I2C_SDA_READ();
			if (value == LOW)
			{// Operation conflict, release bus(already done) and quit
				return (2);
			}
#endif
		}else  WIRE_I2C_SDA_LOW();
		WIRE_I2C_WAIT();

		WIRE_I2C_SCL_HIGH();
		WIRE_I2C_WAIT();
#ifdef I2C_BUS_CHECK
		value = WIRE_I2C_SCL_READ();
		if (value == LOW)
		{// Operation conflict, release bus and quit
			WIRE_I2C_SDA_HIGH();
			return (2);
		}
#endif
		txd <<= 1;
		WIRE_I2C_WAIT();
	}
	WIRE_I2C_SCL_LOW();
	WIRE_I2C_WAIT();
	WIRE_I2C_SDA_HIGH();
	WIRE_I2C_SCL_HIGH();
	value = WIRE_I2C_SCL_READ();
	i = 0;
	while(value == LOW)
	{
		WIRE_I2C_WAIT();
		value = WIRE_I2C_SCL_READ();
		i++;
		if(i > 5)return HIGH;
	}
	
	ack = WIRE_I2C_SDA_READ();
	WIRE_I2C_SCL_LOW();
	WIRE_I2C_WAIT();
	WIRE_I2C_SDA_HIGH();
	return ack;
}

int TwoWire::MasterGPIOTransmission(uint8_t *send, uint8_t sizeofsend)
{
	int i = 0, ack;

	while((I2C_GetITStatus(twi, I2C_ISR_BUSY) == SET))
	{
		WIRE_I2C_WAIT();
	}
#ifdef I2C_BUS_CHECK
	while(WIRE_I2C_SCL_READ() && WIRE_I2C_SDA_READ() && (i<(START_DELAY*WIRE_I2C_DELAY*5)))
	{
		delayMicroseconds(1);
		i++;
	}
	if (i < (START_DELAY*WIRE_I2C_DELAY))
	{
		return (0);
	}
	I2C_Cmd(twi, DISABLE);
#endif

#ifndef	SWITCH_I2C_GPIO
	WIRE_I2C_SDA_HIGH();
	WIRE_I2C_SCL_HIGH();
	pinMode(SCL, OUTPUT);
	pinMode(SDA, OUTPUT);
	WIRE_I2C_SDA_HIGH();
	WIRE_I2C_SCL_HIGH();
	ack = WIRE_I2C_SCL_READ();
	if (ack == LOW)
	{// Operation conflict, release bus(already done) and quit
		pinMode(SCL, ALTERNATE);
		pinMode(SDA, ALTERNATE);
		I2C_Cmd(twi, ENABLE);
		return (0);
	}
	ack = WIRE_I2C_SDA_READ();
	if (ack == LOW)
	{// Operation conflict, release bus(already done) and quit
		pinMode(SCL, ALTERNATE);
		pinMode(SDA, ALTERNATE);
		I2C_Cmd(twi, ENABLE);
		return (0);
	}
#endif
	WIRE_I2C_SDA_LOW();
	WIRE_I2C_WAIT();
	WIRE_I2C_WAIT();
	WIRE_I2C_WAIT();
	WIRE_I2C_WAIT();

	ack = i2cPioSendByte(MAINBOARD_BLE_I2C_ADDRESS<<1);
	if(ack > 0)
	{// No ACK, STOP and return;
#ifndef	SWITCH_I2C_GPIO
		WIRE_I2C_SDA_LOW();
		WIRE_I2C_WAIT();
		WIRE_I2C_SCL_HIGH();
		WIRE_I2C_WAIT();
		WIRE_I2C_WAIT();
		WIRE_I2C_SDA_HIGH();
		pinMode(SCL, ALTERNATE);
		pinMode(SDA, ALTERNATE);
		I2C_Cmd(twi, ENABLE);
#endif
		return 0;
	}

	for(i=0;i<sizeofsend;i++)
	{
		ack = i2cPioSendByte(send[i]);
		if(ack > 0)
		{
			break;
		}
	}

	WIRE_I2C_SDA_LOW();
	WIRE_I2C_WAIT();
	WIRE_I2C_SCL_HIGH();
	WIRE_I2C_WAIT();
	WIRE_I2C_WAIT();
	WIRE_I2C_SDA_HIGH();

#ifndef	SWITCH_I2C_GPIO
	pinMode(SCL, ALTERNATE);
	pinMode(SDA, ALTERNATE);
#endif
	I2C_Cmd(twi, ENABLE);

	return 1;
}


int TwoWire::GPIOTransmission(uint8_t *send, uint8_t sizeofsend)
{
	int i = 0, ack;

	while((I2C_GetITStatus(twi, I2C_ISR_BUSY) == SET))
	{
		WIRE_I2C_WAIT();
	}
#ifdef I2C_BUS_CHECK
	while(WIRE_I2C_SCL_READ() && WIRE_I2C_SDA_READ() && (i<(START_DELAY*WIRE_I2C_DELAY)))
	{
		delayMicroseconds(1);
		i++;
	}
	if (i < (START_DELAY*WIRE_I2C_DELAY))
	{
		return (0);
	}
	I2C_Cmd(twi, DISABLE);
#endif

#ifndef	SWITCH_I2C_GPIO
	WIRE_I2C_SDA_HIGH();
	WIRE_I2C_SCL_HIGH();
	pinMode(SCL, OUTPUT);
	pinMode(SDA, OUTPUT);
#endif
	WIRE_I2C_SDA_LOW();
	WIRE_I2C_WAIT();
	WIRE_I2C_WAIT();
	WIRE_I2C_WAIT();
	WIRE_I2C_WAIT();

	for(i=0;i<sizeofsend;i++)
	{
		ack = i2cPioSendByte(send[i]);
		if(ack > 0)
		{
			break;
		}
	}

	WIRE_I2C_SDA_LOW();
	WIRE_I2C_WAIT();
	WIRE_I2C_SCL_HIGH();
	WIRE_I2C_WAIT();
	WIRE_I2C_WAIT();
	WIRE_I2C_SDA_HIGH();

#ifndef	SWITCH_I2C_GPIO
	pinMode(SCL, ALTERNATE);
	pinMode(SDA, ALTERNATE);
#endif
	I2C_Cmd(twi, ENABLE);

	return 1;
}

#endif
//
//	Originally, 'endTransmission' was an f(void) function.
//	It has been modified to take one parameter indicating
//	whether or not a STOP should be performed on the bus.
//	Calling endTransmission(false) allows a sketch to
//	perform a repeated start.
//
//	WARNING: Nothing in the library keeps track of whether
//	the bus tenure has been properly ended with a STOP. It
//	is very possible to leave the bus in a hung state if
//	no call to endTransmission(true) is made. Some I2C
//	devices will behave oddly if they do not see a STOP.
//
uint8_t TwoWire::endTransmission(uint8_t sendStop) {
	uint32_t _millis;

	_millis = millis();
	while(I2C_GetFlagStatus(twi, I2C_FLAG_BUSY) != RESET)
	{
		if(EVENT_TIMEOUT < (millis() - _millis)) return 0;
	}
	if (sendStop == true)
	{
		I2C_TransferHandling(twi, txAddress, txBufferLength, I2C_AutoEnd_Mode, I2C_Generate_Start_Write);
	}
	else
	{
		I2C_TransferHandling(twi, txAddress, txBufferLength, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
	}

	uint8_t *pBuffer = txBuffer;
	uint8_t NumByteToWrite = txBufferLength;
	/* While there is data to be read */
	while(NumByteToWrite--)
	{
		_millis = millis();
		while(I2C_GetFlagStatus(twi, I2C_FLAG_TXIS) == RESET)
		{
			if(EVENT_TIMEOUT < (millis() - _millis)) return 0;
		}
		if (I2C_GetFlagStatus(twi, I2C_FLAG_NACKF) == SET) return 0;
		/* Send the current byte to slave */
		I2C_SendData(twi, *pBuffer++);
	}

	_millis = millis();
	if (sendStop == true)
	{
		while(I2C_GetFlagStatus(twi, I2C_FLAG_STOPF) == RESET)
		{
			if(EVENT_TIMEOUT < (millis() - _millis)) return 0;
		}
	}
	else
	{
		while(I2C_GetFlagStatus(twi, I2C_FLAG_STOPF) == RESET)
		{
			if(EVENT_TIMEOUT < (millis() - _millis)) return 0;
		}
	}

	// reset tx buffer iterator vars
	txBufferLength = 0;
	status = MASTER_IDLE;
	return 0;
}

//	This provides backwards compatibility with the original
//	definition, and expected behaviour, of endTransmission
//
uint8_t TwoWire::endTransmission(void)
{
	return endTransmission(true);
}

size_t TwoWire::write(uint8_t data) {
	if (status == MASTER_SEND) {
		if (txBufferLength >= BUFFER_LENGTH)
			return 0;
		txBuffer[txBufferLength++] = data;
		return 1;
	} else {
		if (srvBufferLength >= BUFFER_LENGTH)
			return 0;
		srvBuffer[srvBufferLength++] = data;
		return 1;
	}
}

size_t TwoWire::write(const uint8_t *data, size_t quantity) {
	if (status == MASTER_SEND) {
		for (size_t i = 0; i < quantity; ++i) {
			if (txBufferLength >= BUFFER_LENGTH)
				return i;
			txBuffer[txBufferLength++] = data[i];
		}
	} else {
		for (size_t i = 0; i < quantity; ++i) {
			if (srvBufferLength >= BUFFER_LENGTH)
				return i;
			srvBuffer[srvBufferLength++] = data[i];
		}
	}
	return quantity;
}

int TwoWire::available(void) {
	return rxBufferLength - rxBufferIndex;
}

int TwoWire::read(void) {
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex++];
	return -1;
}

int TwoWire::peek(void) {
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex];
	return -1;
}

void TwoWire::flush(void) {
	// Do nothing, use endTransmission(..) to force
	// data transfer.
	// reset.

}
#ifdef GPIO_I2C

int TwoWire::isbusidle(void) {
	// Do nothing, use endTransmission(..) to force
	// data transfer.
	// reset.
	if(WIRE_I2C_SCL_READ() && WIRE_I2C_SDA_READ())return 1;

	return 0;
}
#endif
void TwoWire::reset(void) {
	// Do nothing, use endTransmission(..) to force
	// data transfer.
	// reset.
	I2C_Cmd(twi, DISABLE);
	status = SLAVE_IDLE;
	I2C_Cmd(twi, ENABLE);
	I2C_ITConfig(twi, I2C_IT_ADDRI|I2C_IT_ERRI, ENABLE);
}

void TwoWire::onReceive(void(*function)(int)) {
	onReceiveCallback = function;
}

void TwoWire::onRequest(void(*function)(void)) {
	onRequestCallback = function;
}

void TwoWire::onService(void)
{
	if (I2C_GetITStatus(twi, I2C_ISR_BERR) == SET)
	{
		reset();
	}
	if (I2C_GetITStatus(twi, I2C_IT_ADDR) == SET)
	{
		I2C_ITConfig(twi, I2C_IT_RXI | I2C_IT_TXI | I2C_IT_STOPI, ENABLE);
		srvBufferLength = 0;
		srvBufferIndex = 0;
		if (twi->ISR & (1 << 16))
		{
			status = SLAVE_SEND;
			if (onRequestCallback)
				onRequestCallback();
		}
		else
		{
			status = SLAVE_RECV;
		}

		I2C_ClearITPendingBit(twi, I2C_IT_ADDR);
	}
	if (I2C_GetITStatus(twi, I2C_IT_TXIS) == SET)
	{
		if(status == MASTER_SEND || status == SLAVE_SEND)
		{
			if (srvBufferIndex < srvBufferLength)
			{
				I2C_SendData(twi, srvBuffer[srvBufferIndex++]);
			}
		}

		// I2C_ClearITPendingBit(twi, I2C_IT_TXIS);
	}
	if (I2C_GetITStatus(twi, I2C_IT_RXNE) == SET)
	{
		if (srvBufferLength < BUFFER_LENGTH)
		{
			srvBuffer[srvBufferLength++] = I2C_ReceiveData(twi);
		}

		// I2C_ClearITPendingBit(twi, I2C_IT_RXNE);
	}
	if (I2C_GetITStatus(twi, I2C_IT_STOPF) == SET)
	{
		if (status == SLAVE_RECV && onReceiveCallback)
		{
			// Copy data into rxBuffer
			// (allows to receive another packet while the
			// user program reads actual data)
			for (uint8_t i = 0; i < srvBufferLength; ++i)
			{
				rxBuffer[i] = srvBuffer[i];
			}
			rxBufferIndex = 0;
			rxBufferLength = srvBufferLength;

			// Alert calling program
			onReceiveCallback(rxBufferLength);
		}

		I2C_ClearITPendingBit(twi, I2C_IT_STOPF);
		I2C_ITConfig(twi, I2C_IT_RXI | I2C_IT_TXI | I2C_IT_STOPI, DISABLE);
		I2C_ITConfig(twi, I2C_IT_ADDRI, ENABLE);
	}
}


TwoWire Wire = TwoWire(I2C1);

#ifdef __cplusplus
extern "C" {
#endif
void WIRE_ISR_HANDLER(void)
{
	Wire.onService();
}
#ifdef __cplusplus
}
#endif
