#ifndef AD7792_H_
#define AD7792_H_

void Select_AD7792 (char channel);
void ad7792_io_set(char ioCurrent);
void ad7792_config_set (char H, char L);
void Reset_AD7792 ();

extern void ad7792_init(char deviceChannel, char sensorType, char ioCurrent, char updateRate, char adcGain, char adcReferenceSource, char adcChannel);
extern int ad7792_GetRawValue(char deviceChannel);
extern char ad7792_calibration(char deviceChannel, char* ret_buf);
extern char ad7792_ReadCalibration(char deviceChannel, char * ret_buf);
extern char ad7792_WRFullCalibration(char deviceChannel, char * ret_buf, char H, char L);
extern void ad7792_WRCalibRegister(char deviceChannel, char H, char L);

float ad7792_GetResistanceFormCode(int code, char gain, float resReference);

char ad7792_status ();
int ad7792_mode ();
int ad7792_data ();
char ad7792_io ();
int ad7792_fullscale ();
int ad7792_offset();
int ad7792_config ();

enum sensorTypes
{
	RTD_2currentSources =1,
	RTD_1currentSource =2,
	TC_plus_RTDtype2 =3,
	NTC_onAIN2 = 4
};

enum ioCurrents
{
	currentDisabled		=0b00000000,
	current_10uA		=0b00000001,
	current_210uA		=0b00000010,
	current_1mA			=0b00000011,
	current_20uA_ch1	=0b00001001,
	current_20uA_ch2	=0b00001101,
	current_420uA_ch1	=0b00001010,
	current_420uA_ch2	=0b00001110
};

enum adcGains
{
	gain_1 = 0,
	gain_2 = 1,
	gain_4 = 2,
	gain_8 = 3,
	gain_16 = 4,
	gain_32 = 5,
	gain_64 = 6,
	gain_128 = 7
};

enum updateRates
{
	fADC_X = 0,
	fADC_470Hz,
	fADC_242Hz,
	fADC_123Hz,
	fADC_62Hz,
	fADC_50Hz,
	fADC_39Hz,
	fADC_33_2Hz,
	fADC_19_6Hz,
	fADC_16_7Hz_50HzRejection,
	fADC_16_7Hz,
	fADC_12_5Hz,
	fADC_10Hz,
	fADC_8_33Hz,
	fADC_6_25Hz,
	fADC_4_17Hz
};

enum adcReferenceSources
{
	external = 0,
	Internal = 1
};

enum adcChannels
{
	AIN1 = 0,
	AIN2 = 1,
	AIN3 = 2,
	AIN1_ =3,
	TempSensor = 6,
	AVdd = 7
};

#endif