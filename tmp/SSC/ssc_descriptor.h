#ifndef SSC_DESCRIPTOR_H_
#define SSC_DESCRIPTOR_H_



struct SSCDescriptor
{	
	enum CONVERSION_TYPE
	{
		RAW = 0,
		LINEAR = 10,
		LUT_DS2431 = 20,
		LUT_uC = 30,		
	};
	enum SSC_TYPE
	{
		UNKNOWN = 0, FREQ, RES, TC, WRTD, A0_5VIN, A0_5VIN_UCADC,
	};
	enum OUTPUT_UNIT
	{
		VOID = 0,
		
		T_CELS = 10,
		T_KELV = 11,
		T_FAHR = 12,
		
		P_Pa = 20,
		P_kPa = 21,
		P_MPa = 22,
		P_PSI = 23,
		
		U_VOLT = 30,
		U_mV = 31,
		
		I_AMP = 40,
		I_mA = 41,
		
		DIST_mm = 50,
		DIST_m = 51,
		DIST_km = 52,
		
		TIME_SEC = 60,
		TIME_mSEC = 61,
		
		FREQ_HZ = 70,
		FREQ_kHZ = 71,
		
		SPEED_METERS_PER_SEC = 80,
		SPEED_KILOMETERS_PER_HOUR = 81,
		SPEED_MILES_PER_HOUR = 82,
		
		//ACCEL_m/s^2 = 90,
		ACCEL_G = 91,
		
		LUM_LUX = 100,
		LUM_Cd = 101,
	};
	
	char Rom[8];
	char SSCType;
	char SensType;
	char SensName[6];
	char OutputUnitType;
	char Reserved[7];
	char ConversionType;
	char MeasSettings[7];
	float Offset;
	float Multiplier;
	unsigned int LUT[33];
	char reserved_2[14];
	long ManufactureDate;
	long CalibrationDate;
};



#endif /* SSC_DESCRIPTOR_H_ */