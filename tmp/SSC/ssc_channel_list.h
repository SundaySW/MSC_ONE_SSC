#ifndef SSC_CHANNEL_LIST_H_
#define SSC_CHANNEL_LIST_H_

class SSCChannelList
{
public:
	typedef void (*callback_t)(char, char);

	SSCChannelList(callback_t signal);
	void OnDebounceTimer();
	void Poll();
	
private:
	callback_t ChannelStateChangeSignal;
	
	// channels state bit flag
	// 1 - channel used, 0 - channel free
	// bit position in flag is channel number 	
	char CurrentState;	
	
	int DebounceTimeout;
	
	struct Channel
	{
		char Number, State;
	}
	TargetChannel;
};

#endif /* SSC_CHANNEL_H_ */