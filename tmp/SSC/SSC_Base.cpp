#include "../onewire/OWDriverSW.h"
#include "../onewire/OWSpecific.h"
#include "SSC_Base.h"
#include "../adc/ad7792.h"
#include "../eepromMap.h"
#include "../parameters/parameters.h"

#include <avr/eeprom.h>
#include <stdint.h>



//
//extern SSCParam SSC_1;
//extern SSCParam SSC_2;
//extern SSCParam SSC_3;

//void DisableSSCChannel(char channel)
//{
	//switch (channel)
	//{
	//case 3:
		//SSC_1.UpdateRate = 0;
		//SSC_1.SendRate = 0;
	//break;
	//case 4:
		//SSC_2.UpdateRate = 0;
		//SSC_2.SendRate = 0;
	//break;
	//case 5:
		//SSC_3.UpdateRate = 0;
		//SSC_3.SendRate = 0;
	//break;
	//}
//}

//char EnableSSCChannel(char channel)
//{
	//char _return = 22;
	//
	//switch (channel)
	//{
	//case 3:
		//_return = SSC_ConfigureChannel(&SSC_1);
	//break;
	//case 4:
		//_return = SSC_ConfigureChannel(&SSC_2);
	//break;
	//case 5:
		//_return = SSC_ConfigureChannel(&SSC_3);
	//break;
	//}
	//return _return;
//}

//char SSC_ConfigureChannel (SSCParam * pSSCParam)
//{
	//char _return = 56;
	//
	//OWI_Init(pSSCParam->OWID_channel);
	//pSSCParam->SSC_type = DS2431_ReadByte(0x00, pSSCParam->OWID_channel);
	////char buffer[8] = {0};
	//OWI_ReadRom(pSSCParam->SSC_ROMID, pSSCParam->OWID_channel);
	////pSSCParam->SSC_ROMID[0] = buffer[0];
	////pSSCParam->SSC_ROMID[1] = buffer[1];
	////pSSCParam->SSC_ROMID[2] = buffer[2];
	////pSSCParam->SSC_ROMID[3] = buffer[3];
	////pSSCParam->SSC_ROMID[4] = buffer[4];
	////pSSCParam->SSC_ROMID[5] = buffer[5];
	////pSSCParam->SSC_ROMID[4] = buffer[6];
	////pSSCParam->SSC_ROMID[5] = buffer[7];
	//
	////check ID in EEPROM TABLE
	//char id_temp = 0;
	//for (unsigned char i = 0; i < SSC_CHANNELS_MAX_NUMBER; i++)
	//{
		//char buffer[8];
		//
		//eeprom_read_block(buffer, EEPROM_SSC_ID_TABLE_1 + 9 * i, 8);
		//if (*(uint64_t*)buffer == *(uint64_t*)(&pSSCParam->SSC_ROMID))
		//{
			////exists in table
			//id_temp = eeprom_read_byte(EEPROM_SSC_ID_TABLE_1 + 9 * i + 8);
			//break;
		//}
	//}
	//
	//if (id_temp != 0)
	//{
		//if (BaseParamFindByID(id_temp) == 0)
		//pSSCParam->ID = id_temp;
	//}
	//else 
	//{
		//for (id_temp = SSC_DYNAMIC_ID_FIRST; id_temp < SSC_DYNAMIC_ID_LAST; id_temp++)
		//{
			//if (BaseParamFindByID(id_temp) == 0)
			//{
				//pSSCParam->ID = id_temp;
				//break;
			//}
		//}
	//}
	//
	//pSSCParam->conversionType = DS2431_ReadByte(16, pSSCParam->OWID_channel);
	//
	//switch (pSSCParam->conversionType)
	//{
		//case 0:
		//break;
		//case 10:
		//{
			//char a[4] = {0};
			//DS2431_ReadBlock_1_4B(pSSCParam->OWID_channel, 24, 4, a);
			//pSSCParam->Offset = *(float*)a;
			//DS2431_ReadBlock_1_4B(pSSCParam->OWID_channel, 28, 4, a);
			//pSSCParam->Multiplier = *(float*)a;
		//}
		//break;
		//case 20:
		//{
			//char a[4] = {0};
			//DS2431_ReadBlock_1_4B(pSSCParam->OWID_channel, 24, 4, a);
			//pSSCParam->Offset = *(float*)a;
			//DS2431_ReadBlock_1_4B(pSSCParam->OWID_channel, 28, 4, a);
			//pSSCParam->Multiplier = *(float*)a;
			//
			//for (char i = 0; i< 16; i++)
			//{
				//DS2431_ReadBlock_1_4B(pSSCParam->OWID_channel, (32 + i * 4), 4, a);
				//pSSCParam->LUT[2*i] = *(int*)(&a[0]);
				//pSSCParam->LUT[2*i+1] = *(int*)(&a[2]);
			//}
			//DS2431_ReadBlock_1_4B(pSSCParam->OWID_channel, 96, 2, a);
			//pSSCParam->LUT[32] = *(int*)(&a[0]);
			//
			//SendProtosMsg(44,MSGTYPE_PARAM_ERROR,a,4);
		//}
		//
		//break;
	//}
	//
	//switch(pSSCParam->SSC_type)
	//{
		//case SSCtype_A0_5Vin_uCADC:
		//
		//break;
		//case SSCtype_A0_5Vin:
		//{
			//ad7792_init( pSSCParam->AD7792_Channel, NTC_onAIN2, currentDisabled , fADC_16_7Hz, gain_1, external, AIN1);
			////pSSCParam->SSC_status = 1;
			//pSSCParam->UpdateRate = 1000;
			//pSSCParam->SendRate = 2000;
			//_return  = 0;
		//}
		//break;
		//case SSCtype_3WRTD:
		//{
			//ad7792_init( pSSCParam->AD7792_Channel,RTD_2currentSources, current_210uA, fADC_16_7Hz, gain_1, external, AIN1);
			////pSSCParam->SSC_status = 1;
			//pSSCParam->UpdateRate = 1000;
			//pSSCParam->SendRate = 2000;
			//char H = eeprom_read_byte((uint8_t*)(256+2*pSSCParam->AD7792_Channel));
			//char L = eeprom_read_byte((uint8_t*)(257+2*pSSCParam->AD7792_Channel));
			//if (H != 0 && H != 255)
			//ad7792_WRCalibRegister(pSSCParam->AD7792_Channel, H ,L);
			//_return = 0;
		//}
		//break;
		//case SSCtype_RES:
		//{
			//ad7792_init( pSSCParam->AD7792_Channel, NTC_onAIN2, current_10uA , fADC_16_7Hz, gain_1, external, AIN1);
			////pSSCParam->SSC_status = 1;
			//pSSCParam->UpdateRate = 1000;
			//pSSCParam->SendRate = 2000;
			//_return = 0;
		//}
		//break;
		//case SSCtype_TC:
		//
		//break;
		//case SSCtype_FREQ:
		//
		//break;
		//default:
		//{
			////pSSCParam->SSC_status = 0;
			//pSSCParam->UpdateRate = 0;
			//pSSCParam->SendRate =0;
			//_return = 5;
		//}
		//break;
	//}
	//
	//return _return;	
//}