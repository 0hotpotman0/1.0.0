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

#ifndef TwoWire_h
#define TwoWire_h

// Include Atmel CMSIS driver
//#include <include/twi.h>

#include "Stream.h"
#include "variant.h"

#define BUFFER_LENGTH 128

#define BUTTON_V_3

#define MAINBOARD_BLE_I2C_DATALEN	0x10
#define LED_FLASH_COUNT				3
#define BLE_SUPPORT

#ifdef BLE_SUPPORT

#define GPIO_I2C

#define I2C_CMD_RAW_DATA_TIME		200
#define I2C_CMD_SYS_READY_TIME		500

#define MAINBOARD_BLE_I2C_ADDRESS	0x20

#define MAINBOARD_BLE_COMMAND_LOW	0x40
#define MAINBOARD_BLE_COMMAND_HIGH	0x80

#define ATMEL_I2C_TIME 200

#define I2C_CMD_ATR				0x55
#define I2C_CMD_LED_FLASH		0xb0

#define I2C_CMD_NOTIFY_EVENT	0x0E //
#define I2C_CMD_GET_RAW_DATA	0xD0 //
#define I2C_CMD_NOTIFY_ATR		0xFB

#define I2C_CMD_BLE_SET_THD		0x85

#define I2C_CMD_CONTINUE_DATA	0x41

#define	CORE_BLE_MODE			0x01
#define	CORE_ATMEL_MODE			0x02

#endif

class TwoWire : public Stream {
public:
	TwoWire(I2C_TypeDef *twi);
	void begin();
	void begin(uint8_t);
	void begin(int);
	void end(void);

    int MasterGPIOTransmission(uint8_t *, uint8_t);
#ifdef GPIO_I2C
    int GPIOTransmission(uint8_t *, uint8_t);
#endif

	void beginTransmission(uint8_t);
	void beginTransmission(int);
	uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
	uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
	uint8_t requestFrom(int, int);
    uint8_t requestFrom(int, int, int);
	int isbusidle(void);
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t *, size_t);
	virtual int available(void);
	virtual int read(void);
	virtual int peek(void);
	virtual void flush(void);
	void reset(void);
	void onReceive(void(*)(int));
	void onRequest(void(*)(void));

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;

	void onService(void);


	// RX Buffer
	uint8_t rxBuffer[BUFFER_LENGTH];
	uint8_t rxBufferIndex;
	uint8_t rxBufferLength;

	// TX Buffer
	uint8_t txAddress;
	uint8_t txBuffer[BUFFER_LENGTH];
	uint8_t txBufferLength;

	// Service buffer
	uint8_t srvBuffer[BUFFER_LENGTH];
	uint8_t srvBufferIndex;
	uint8_t srvBufferLength;
private:
	// Callback user functions
	void (*onRequestCallback)(void);
	void (*onReceiveCallback)(int);

	// Called before initialization
	void (*onBeginCallback)(void);

	// TWI instance
	I2C_TypeDef *twi;
	I2C_InitTypeDef I2C_InitStructure;

	// TWI state
	enum TwoWireStatus {
		UNINITIALIZED = 0,
		MASTER_IDLE,
		MASTER_SEND,
		MASTER_RECV,
		SLAVE_IDLE,
		SLAVE_RECV,
		SLAVE_SEND
	};
	TwoWireStatus status;

	// TWI clock frequency
	static const uint32_t TWI_CLOCK = 100000;

	// Timeouts (
	static const uint32_t RECV_TIMEOUT = 100000;
	static const uint32_t XMIT_TIMEOUT = 100000;
};

extern TwoWire Wire;


#endif

