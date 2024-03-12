/*
 * ssc_channel_list.cpp
 *
 * Created: 06.02.2016 17:15:49
 *  Author: stremyakov
 */ 

#include <avr/io.h>
#include "../misc/misc.h"
#include "ssc_channel_list.h"
#include "SSC_Base.h"

SSCChannelList::SSCChannelList(callback_t signal)
	: ChannelStateChangeSignal(signal)
	, CurrentState(0)
	, DebounceTimeout(0)
{
	//has not target channel
	TargetChannel.Number = 0xFF;
	TargetChannel.State = 0;
}

void SSCChannelList::OnDebounceTimer()
{
	if (DebounceTimeout)
		DebounceTimeout--;
}

void SSCChannelList::Poll()
{
	if (DebounceTimeout) return;
	
	if (TargetChannel.Number != 0xFF)
	{
		// save channel state
		CurrentState ^= (1 << TargetChannel.Number);
		
		if (ChannelStateChangeSignal)
			(*ChannelStateChangeSignal)(TargetChannel.Number, TargetChannel.State);
		
		TargetChannel.Number = 0xFF;
	}
	
	struct
	{
		volatile uint8_t &Ddr, &Port;
		char Bit;
		volatile uint8_t &Pin;
	}
	c[] = {
		{ch1_DDR, ch1_PORT, ch1_BIT, ch1_PIN},
		{ch2_DDR, ch2_PORT, ch2_BIT, ch2_PIN},
		{ch3_DDR, ch3_PORT, ch3_BIT, ch3_PIN},
		{ch4_DDR, ch4_PORT, ch4_BIT, ch4_PIN},
		{ch5_DDR, ch5_PORT, ch5_BIT, ch5_PIN},
	};
	
	char ddrState  = 0;
	char portState = 0;
	
	for (int i = 0; i < sizeof(c) / sizeof(c[0]); i++)
	{
		ddrState =  (c[i].Ddr  & (1 << c[i].Bit));
		portState = (c[i].Port & (1 << c[i].Bit));
		
		CBR(c[i].Port, c[i].Bit);
		CBR(c[i].Ddr,  c[i].Bit);
		
		char curState = (c[i].Pin & (1 << c[i].Bit)) >> c[i].Bit;
		//get previous state for channel
		char prevState = (CurrentState & (1 << i)) ? 1 : 0;
		
		if (curState != prevState)
		{
			// init debouncing timer
			DebounceTimeout = 200;
			// remember channel and state
			TargetChannel.Number = i;
			TargetChannel.State  = curState;
		}
		
		c[i].Ddr  |= ddrState;
		c[i].Port |= portState;
		
		if (curState != prevState)
		break;
	}
}