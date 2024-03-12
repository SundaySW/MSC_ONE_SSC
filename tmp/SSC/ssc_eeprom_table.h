#ifndef SSC_EEPROM_TABLE_H_
#define SSC_EEPROM_TABLE_H_

#include <stdbool.h>
#include <stdint.h>

namespace SSCTable
{
	extern bool FindParamByROMID(uint64_t romId, char& id);
	extern void Write(int row, uint64_t romId, char paramId);
}

#endif /* SSC_EEPROM_TABLE_H_ */