/*
 * ssc_param.cpp
 *
 * Created: 06.02.2016 16:03:12
 *  Author: stremyakov
 */ 

#include "ssc_param.h"
#include <avr/eeprom.h>
#include "ssc_descriptor.h"

SSCParam::SSCParam(char id,
		float value,
		char readOnly,
		int sendRate,
		int updateRate,
		char OWIDChannel,
		char uCADCnum,
		char AD7792Channel) : FloatParam(id, value, readOnly, sendRate)	
{
	UpdateRate = updateRate;
	TypeAttr |= (1<<4); //defines that param is updatable
	
	OWID_channel = OWIDChannel;
	uC_ADC_num = uCADCnum;
	AD7792_Channel = AD7792Channel;
	
	SSC_type = 0;
	ConversionType = 0;
	Multiplier = 1;
	Offset = 0;
	//LUT members zero before initializtion
}

void SSCParam::Disable()
{
	UpdateRate = SendRate = 0;
	ID = 0;
}

void SSCParam::Enable(const SSCDescriptor& desc)
{
	SSC_type = desc.SSCType;
	ConversionType = desc.ConversionType;
	
	switch (ConversionType)
	{
	case SSCDescriptor::RAW:
	break;
	case SSCDescriptor::LINEAR:
	{
		Offset = desc.Offset;
		Multiplier = desc.Multiplier;
	}
	break;
	case SSCDescriptor::LUT_DS2431:
	{
		Offset = desc.Offset;
		Multiplier = desc.Multiplier;
		for (unsigned char i = 0; i < 33; i++)
		{
			LUT[i] = desc.LUT[i];
		}
	}
	break;
	default:
		
	break;
	}
	
	switch(SSC_type)
	{
	case SSCDescriptor::A0_5VIN_UCADC:
	break;
	case SSCDescriptor::A0_5VIN:
	{
		ad7792_init(AD7792_Channel, NTC_onAIN2, currentDisabled , fADC_16_7Hz, gain_1, external, AIN1);
	}
	break;
	case SSCDescriptor::WRTD:
	{
		ad7792_init(AD7792_Channel,RTD_2currentSources, current_210uA, fADC_16_7Hz, gain_1, external, AIN1);
		
		char H = eeprom_read_byte((uint8_t*)(256+2*AD7792_Channel));
		char L = eeprom_read_byte((uint8_t*)(257+2*AD7792_Channel));
		if (H != 0 && H != 255)
		ad7792_WRCalibRegister(AD7792_Channel, H ,L);
	}
	break;
	case SSCDescriptor::RES:
		ad7792_init(AD7792_Channel, NTC_onAIN2, current_10uA , fADC_16_7Hz, gain_1, external, AIN1);
	break;
	case SSCDescriptor::TC:
	break;
	case SSCDescriptor::FREQ:
	break;
	default:
		Disable();
		return;
	break;
	}
	
	UpdateRate = 1000;
	UpdateTime = 0;
	SendRate = 2000;
	SendTime = 1000;
}

void SSCParam::Update (void)
{
	if (SSC_type != 0)
	{
		switch(SSC_type)
		{
			case SSCDescriptor::A0_5VIN_UCADC:
			break;
			case SSCDescriptor::A0_5VIN:
			{
				Convert((unsigned)ad7792_GetRawValue(AD7792_Channel));
			}
			break;
			case SSCDescriptor::WRTD:
			{
				Convert((unsigned)ad7792_GetRawValue(AD7792_Channel));
			}
			break;
			case SSCDescriptor::RES:
			{
				Value = (float)((unsigned)ad7792_GetRawValue(AD7792_Channel)) * 7.7362;
			}
			break;
			case SSCDescriptor::TC:
			break;
			case SSCDescriptor::FREQ:
			break;
		}
	}
}

int SSCParam::GetUpdateRate()
{
	return UpdateRate;
}

void SSCParam::Convert(unsigned int Input)
{
	if (ConversionType == 20)
	{
		Input = Input/2;
		//float Output =0;
		//2048 for 32 LUT indexes, 33 - value at full scale
		unsigned char index = Input/1024;
		//if (index>32)
		//	return;
		//?int or float
		int remainder = Input % 1024;
		Value = (float)(LUT[index]) + (((float)(LUT[index+1]-LUT[index])*remainder))/1024;
		
		Value = (Value*Multiplier)-Offset;
	}
	else if (ConversionType == 10)
	{
		Value = (((float)Input * 0.000077362)-Offset) * Multiplier;
	}
}