#include "ad7792.h"
#include "../misc/misc.h"
#include "../spi/spi.h"
#include <util/delay.h>


#define AD7792_COMMUNICATION_REGISTER 0
#define AD7792_STATUS_REGISTER 0
#define AD7792_MODE_REGISTER 1
#define AD7792_CONFIGURATION_REGISTER 2
#define AD7792_DATA_REGISTER 3
#define AD7792_ID_REGISTER 4
#define AD7792_IO_REGISTER 5
#define AD7792_OFFSET_REGISTER 6
#define AD7792_FULLSCALE_REGISTER 7

#define RS0 3
#define RW 6
#define WEN 7

void Select_AD7792 (char channel)
{
	if (channel == 0)
	{
		SBR(PORTC,0);
		SBR(PORTD,0);
		SBR(PORTB,6);
		
		CBR(DDRC,0);
		CBR(DDRD,0);
		CBR(DDRB,6);
	}
	
	else if (channel == 1)
	{
		SBR(PORTD,0);
		SBR(PORTB,6);
		CBR(DDRD,0);
		CBR(DDRB,6);
		
		SBR(DDRC,0);
		CBR(PORTC,0);
	}
	
	else if (channel == 2)
	{
		SBR(PORTC,0);
		SBR(PORTB,6);
		CBR(DDRC,0);
		CBR(DDRB,6);
		
		SBR(DDRD,0);
		CBR(PORTD,0);
	}
	
	else if (channel == 3)
	{
		SBR(PORTC,0);
		SBR(PORTD,0);
		CBR(DDRC,0);
		CBR(DDRD,0);
		
		SBR(DDRB,6);
		CBR(PORTB,6);
	}
}

void Reset_AD7792 ()
{
	SPI_Transmit(0xff);
	SPI_Transmit(0xff);
	SPI_Transmit(0xff);
	SPI_Transmit(0xff);
}

char ad7792_status ()
{
	SPI_Transmit((0<<WEN)|(1<<RW)|(AD7792_STATUS_REGISTER<<RS0));
	return SPI_Read();
}

int ad7792_mode ()
{
	SPI_Transmit((0<<WEN)|(1<<RW)|(AD7792_MODE_REGISTER<<RS0));
	char m1 = SPI_Read();
	char m2 = SPI_Read();
	return ((m1<<8)|m2);
}

void ad7792_mode_set (char H, char L)
{
	SPI_Transmit((0<<WEN)|(0<<RW)|(AD7792_MODE_REGISTER<<RS0));
	SPI_Transmit(H);
	SPI_Transmit(L);
}

int ad7792_data ()
{
	SPI_Transmit((0<<WEN)|(1<<RW)|(AD7792_DATA_REGISTER<<RS0));
	char m1 = SPI_Read();
	char m2 = SPI_Read();
	return ((m1<<8)|m2);
}
void ad7792_io_set (char io)
{
	SPI_Transmit((0<<WEN)|(0<<RW)|(AD7792_IO_REGISTER<<RS0));
	SPI_Transmit(io);
}

char ad7792_io ()
{
	SPI_Transmit((0<<WEN)|(1<<RW)|(AD7792_IO_REGISTER<<RS0));
	return SPI_Read();
}

int ad7792_config ()
{
	SPI_Transmit((0<<WEN)|(1<<RW)|(AD7792_CONFIGURATION_REGISTER<<RS0));
	char m1 = SPI_Read();
	char m2 = SPI_Read();
	return ((m1<<8)|m2);
}
void ad7792_config_set (char H, char L)
{
	SPI_Transmit((0<<WEN)|(0<<RW)|(AD7792_CONFIGURATION_REGISTER<<RS0));
	SPI_Transmit(H);
	SPI_Transmit(L);
}

int ad7792_offset()
{
	SPI_Transmit((0<<WEN)|(1<<RW)|(AD7792_OFFSET_REGISTER<<RS0));
	char m1 = SPI_Read();
	char m2 = SPI_Read();
	return ((m1<<8)|m2);
}
int ad7792_fullscale ()
{
	SPI_Transmit((0<<WEN)|(1<<RW)|(AD7792_FULLSCALE_REGISTER<<RS0));
	char m1 = SPI_Read();
	char m2 = SPI_Read();
	return ((m1<<8)|m2);
}

void ad7792_fullscale_write (char H, char L)
{
	SPI_Transmit((0<<WEN)|(0<<RW)|(AD7792_FULLSCALE_REGISTER<<RS0));
	SPI_Transmit(H);
	SPI_Transmit(L);
}

void ad7792_init(char deviceChannel, char sensorType, char ioCurrent, char updateRate, char adcGain, char adcReferenceSource, char adcChannel)
{
	if (sensorType == RTD_2currentSources)
	{
		Select_AD7792(deviceChannel);
		
		Reset_AD7792();
		
		ad7792_io_set(ioCurrent);
			
		ad7792_config_set((1<<4)|(adcGain & 0x07),(adcReferenceSource<<7)|(1<<4)|(adcChannel & 0x07));
		ad7792_mode_set(0, (updateRate & 0x0f));
	}
	
	else if (sensorType == NTC_onAIN2)
	{
		Select_AD7792(deviceChannel);
		
		Reset_AD7792();
		
		ad7792_io_set(ioCurrent);
				
		ad7792_config_set((1<<4)|(adcGain & 0x07),(adcReferenceSource<<7)|(adcChannel & 0x07));
		ad7792_mode_set(0, (updateRate & 0x0f));
	}
	
	Select_AD7792(0);
}

int ad7792_GetRawValue(char deviceChannel)
{
	Select_AD7792(deviceChannel);
	
	SPI_Transmit((0<<WEN)|(1<<RW)|(AD7792_DATA_REGISTER<<RS0));
	char m1 = SPI_Read();
	char m2 = SPI_Read();
	
	Select_AD7792(0);
	
	return ((m1<<8)|m2);
}

float ad7792_GetResistanceFormCode(int code, char gain, float resReference)
{
		return ((code*2*resReference)/(65535 * gain));
}

char ad7792_calibration(char deviceChannel, char * ret_buf)
{
	Select_AD7792(deviceChannel);
	
	//Reset_AD7792();
	
	int modeBefore = ad7792_mode();
	
	ad7792_mode_set((1<<7)|(0<<6)|(0<<5), (fADC_16_7Hz & 0x0f)); //internal zero offest calib
	_delay_ms(10);
	ad7792_mode_set((1<<7)|(0<<6)|(1<<5), (fADC_16_7Hz & 0x0f)); //internal full scale calib
	_delay_ms(10);
	
	ad7792_mode_set(modeBefore>>8, modeBefore&0x00FF);
	
	*(int*)&ret_buf[0] = ad7792_offset();
	*(int*)&ret_buf[2] = ad7792_fullscale();
	Select_AD7792(0);
	return 0;
}
char ad7792_ReadCalibration(char deviceChannel, char * ret_buf)
{
	Select_AD7792(deviceChannel);
	
    *(int*)&ret_buf[0] = ad7792_offset();
    *(int*)&ret_buf[2] = ad7792_fullscale();
    Select_AD7792(0);
	return 0;
}
//for debug with answer - remove usage in device
char ad7792_WRFullCalibration(char deviceChannel, char * ret_buf, char H, char L)
{
	Select_AD7792(deviceChannel);
	
	int modeBefore = ad7792_mode();
	
	ad7792_mode_set((0<<7)|(1<<6)|(0<<5), (fADC_16_7Hz & 0x0f));
	_delay_ms(10);
	
	ad7792_fullscale_write(H,L);
	
	*(int*)&ret_buf[0] = ad7792_offset();
	*(int*)&ret_buf[2] = ad7792_fullscale();
	
	ad7792_mode_set(modeBefore>>8, modeBefore&0x00FF);
	
	Select_AD7792(0);
	return 0;
}

void ad7792_WRCalibRegister(char deviceChannel, char H, char L)
{
	Select_AD7792(deviceChannel);
	
	int modeBefore = ad7792_mode();
	
	ad7792_mode_set((0<<7)|(1<<6)|(0<<5), (fADC_16_7Hz & 0x0f));
	
	ad7792_fullscale_write(H,L);
	
	ad7792_mode_set(modeBefore>>8, modeBefore&0x00FF);
	
	Select_AD7792(0);
}