
#ifndef _FLASH_H_INCLUDED
#define _FLASH_H_INCLUDED

#include "arduino.h"

#define FLASH_SIZE			    64   		 /* Flash Size in bytes */

#if defined(STM32F030)

    #define FLASH_USER_START_ADDR   0x08003C00   /* Flash Start Address */

#elif defined(STM32F031)

    #define FLASH_USER_START_ADDR   0x08007C00   /* Flash Start Address */

#endif


class FlashClass {
public:
  FlashClass() {
    memset((uint8_t *)blockdata, 0, FLASH_SIZE);
  }
	void clear();

	void write32(uint16_t address, uint32_t data);
	void write16(uint16_t address, uint16_t data);
	void write8(uint16_t address, uint8_t data);

    void writeAll(void);
    void readAll(void);

	uint32_t read32(uint16_t addr);
	uint16_t read16(uint16_t addr);
	uint8_t read8(uint16_t addr);

	uint32_t *getChipUniqueID(void);

    uint32_t blockdata[FLASH_SIZE >> 2];

private:

	uint32_t chipUniqueID[3];

};


extern FlashClass Flash;


#endif
