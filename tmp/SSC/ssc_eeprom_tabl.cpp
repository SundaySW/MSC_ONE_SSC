#include "ssc_eeprom_table.h"
#include "SSC_Base.h"
#include "../eepromMap.h"
#include <avr/eeprom.h>

namespace SSCTable
{
	enum { NODE_SIZE = 0x9 };
		
	bool FindParamByROMID(uint64_t romId, char& id)
	{
		for (unsigned char i = 0; i < SSC_CHANNELS_MAX_NUMBER; i++)
		{
			char buffer[8];
			uint8_t* nodeAddr = (uint8_t*)EEPROM_SSC_ID_TABLE_ADDR + NODE_SIZE * i;
			
			eeprom_read_block(buffer, (void*)nodeAddr, 8);
			if (*(uint64_t*)buffer == romId)
			{
				id = eeprom_read_byte(nodeAddr + 8);
				return true;
			}
		}
		return false;
	}

	void Write(int row, uint64_t romId, char paramId)
	{
		uint8_t* blockAddr = (uint8_t*)EEPROM_SSC_ID_TABLE_ADDR + row * NODE_SIZE;
		eeprom_write_block(&romId, (void*)blockAddr, sizeof(romId));
		blockAddr += sizeof(romId);
		eeprom_write_byte(blockAddr, (uint8_t)paramId);
	}

}