
#include "arduino.h"
#include "Flash.h"

void FlashClass::clear()
{
	/* Unlock the Flash to enable the flash control register access */
	FLASH_Unlock();

	/* Clear pending flags */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

	/* Erase the FLASH pages */
	FLASH_ErasePage(FLASH_USER_START_ADDR);

	/* Lock the Flash to disable the flash control register access */
	FLASH_Lock();
}

void FlashClass::writeAll(void)
{
	// erase all
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	FLASH_ErasePage(FLASH_USER_START_ADDR);
	// write buffer
	for(int i = 0; i < (FLASH_SIZE >> 2); i++)
	{
		FLASH_ProgramWord(FLASH_USER_START_ADDR + (i << 2), blockdata[i]);
	}
	FLASH_Lock();
}

void FlashClass::write32(uint16_t address, uint32_t data)
{
	if(address >= (FLASH_SIZE >> 2))return;

	uint32_t currentValue = *(__IO uint32_t *)(FLASH_USER_START_ADDR + (address << 2));
	if(currentValue == data)return;

	if(currentValue != 0xFFFFFFFF)
	{
		for(int i = 0; i < (FLASH_SIZE >> 2); i++)
		{
			blockdata[i] = *(__IO uint32_t *)(FLASH_USER_START_ADDR + (i << 2));
		}

		blockdata[address] = data;

		FLASH_Unlock();
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
		FLASH_ErasePage(FLASH_USER_START_ADDR);
		for(int i = 0; i < (FLASH_SIZE >> 2); i++)
		{
			FLASH_ProgramWord(FLASH_USER_START_ADDR + (i << 2), blockdata[i]);
		}
		FLASH_Lock();
	}
	else
	{
		FLASH_Unlock();
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
		FLASH_ProgramWord(FLASH_USER_START_ADDR + (address << 2), data);
		FLASH_Lock();
	}
}

void FlashClass::write16(uint16_t address, uint16_t data)
{
	if(address >= (FLASH_SIZE >> 1))return;

	uint16_t currentValue = *(__IO uint16_t *)(FLASH_USER_START_ADDR + (address << 1));
	if(currentValue == data)return;

	if(currentValue != 0xFFFF)
	{
		for(int i = 0; i < (FLASH_SIZE >> 2); i++)
		{
			blockdata[i] = *(__IO uint32_t *)(FLASH_USER_START_ADDR + (i << 2));
		}

		int offset = address % 2;
		int blockIndex = address / 2;
		uint32_t data32 = blockdata[blockIndex];
		uint16_t data_array[2];
		memcpy(data_array, &data32, 4);
		data_array[offset] = data;
		memcpy(&data32, data_array, 4);
		blockdata[blockIndex] = data32;

		FLASH_Unlock();
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
		FLASH_ErasePage(FLASH_USER_START_ADDR);
		for(int i = 0; i < (FLASH_SIZE >> 2); i++)
		{
			FLASH_ProgramWord(FLASH_USER_START_ADDR + (i << 2), blockdata[i]);
		}
		FLASH_Lock();
	}
	else
	{
		FLASH_Unlock();
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
		FLASH_ProgramHalfWord(FLASH_USER_START_ADDR + (address << 1), data);
		FLASH_Lock();
	}
}

void FlashClass::write8(uint16_t address, uint8_t data)
{
	if(address >= FLASH_SIZE)return;

	uint8_t currentValue = *(__IO uint8_t *)(FLASH_USER_START_ADDR + address);
	if(currentValue == data)return;

	for(int i = 0; i < (FLASH_SIZE >> 2); i++)
	{
		blockdata[i] = *(__IO uint32_t *)(FLASH_USER_START_ADDR + (i << 2));
	}

	int offset = address % 4;
	int blockIndex = address / 4;
	uint32_t data32 = blockdata[blockIndex];
	uint8_t data_array[4];
	memcpy(data_array, &data32, 4);
	data_array[offset] = data;
	memcpy(&data32, data_array, 4);
	blockdata[blockIndex] = data32;

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	FLASH_ErasePage(FLASH_USER_START_ADDR);
	for(int i = 0; i < (FLASH_SIZE >> 2); i++)
	{
		FLASH_ProgramWord(FLASH_USER_START_ADDR + (i << 2), blockdata[i]);
	}
	FLASH_Lock();
}

void FlashClass::readAll(void)
{
	for (uint16_t i = 0; i < (FLASH_SIZE >> 2); i++)
	{
		blockdata[i] = *(__IO uint32_t *)(FLASH_USER_START_ADDR + (i << 2));
	}
}


uint32_t FlashClass::read32(uint16_t addr)
{
	uint32_t data = 0;

	if(addr < (FLASH_SIZE >> 2))
	data = *(__IO uint32_t *)(FLASH_USER_START_ADDR + (addr << 2));

	return data;
}

uint16_t FlashClass::read16(uint16_t addr)
{
	uint16_t data = 0;

	if(addr < (FLASH_SIZE >> 1))
	data = *(__IO uint16_t *)(FLASH_USER_START_ADDR + (addr << 1));

	return data;
}

uint8_t FlashClass::read8(uint16_t addr)
{
	uint8_t data = 0;

	if(addr < FLASH_SIZE)
	data = *(__IO uint8_t *)(FLASH_USER_START_ADDR + addr);

	return data;
}

uint32_t *FlashClass::getChipUniqueID(void)
{
	chipUniqueID[0] = *(__IO uint32_t *)(0x1FFFF7AC); // Low word
	chipUniqueID[1] = *(__IO uint32_t *)(0X1FFFF7B0); // Mid word
	chipUniqueID[2] = *(__IO uint32_t *)(0x1FFFF7B4); // High word

	return chipUniqueID;
}


FlashClass Flash;
