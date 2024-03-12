#ifndef SSC_PARAM_H_
#define SSC_PARAM_H_

#include "SSC_Base.h"
#include "../parameters/parameters.h"
struct SSCDescriptor;

class SSCParam : public ParamUpdate, public FloatParam
{
public:
	
	
	SSCParam (char id, 
		float value, 
		char readOnly, 
		int sendRate, 
		int updateRate, 
		char OWIDChannel, 
		char uCADCnum, 
		char AD7792Channel);

	void Convert(unsigned int input);
	void Disable();
	void Enable(const SSCDescriptor& desc);
	int GetUpdateRate();
	void Update();
	
	
	char OWID_channel;
	char uC_ADC_num;
	char AD7792_Channel;
	
	char SSC_type;
	uint64_t SSC_ROMID;
	
	char ConversionType;
	signed int LUT[33];
	
	float Offset;
	float Multiplier;
};



#endif /* SSC_PARAM_H_ */