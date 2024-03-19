#pragma once

#include "ad7792_specs.hpp"

#include <utility>
#include <cstdint>

namespace AD7792_adc{

struct AD7792{
    using HardWareTransmitT = std::function<void(uint8_t*, uint8_t)>;

    explicit AD7792(HardWareTransmitT transmit_f)
        : transmit_f_(std::move(transmit_f))
    {}

    void IO_Set(uint8_t ioCurrent) {
        tx_data_[0] = (0<<WEN) | (0<<RW) | (AD7792_IO_REGISTER<<RS0);
        tx_data_[1] = ioCurrent;
        Transmit(2);
    }

    void Config_Set (uint8_t H, uint8_t L) {
        tx_data_[0] = (0<<WEN) | (0<<RW) | (AD7792_CONFIGURATION_REGISTER<<RS0);
        tx_data_[1] = H;
        tx_data_[2] = L;
        Transmit(3);
    }

    void Mode_Set (uint8_t H, uint8_t L)
    {
        tx_data_[0] = (0<<WEN)|(0<<RW)|(AD7792_MODE_REGISTER<<RS0);
        tx_data_[1] = H;
        tx_data_[2] = L;
        Transmit(3);
    }

    void Reset (){
        tx_data_[0] = 0xff;
        tx_data_[1] = 0xff;
        tx_data_[2] = 0xff;
        tx_data_[3] = 0xff;
        Transmit(4);
    }

    void Init(uint8_t sensorType, uint8_t ioCurrent, uint8_t updateRate, uint8_t adcGain, uint8_t adcReferenceSource, uint8_t adcChannel) {
        if (sensorType == RTD_2currentSources)
        {
            Reset();
            IO_Set(ioCurrent);
            Config_Set( ( (1<<4) | (adcGain & 0x07) ), ( (adcReferenceSource<<7) | (1<<4) | (adcChannel & 0x07) ) );
            Mode_Set(0, updateRate & 0x0f);
        }

        else if (sensorType == NTC_onAIN2)
        {
            Reset();
            IO_Set(ioCurrent);
            Config_Set ( (1<<4) | (adcGain & 0x07),(adcReferenceSource<<7) | (adcChannel & 0x07) );
            Mode_Set(0, updateRate & 0x0f);
        }
    }

    int GetRawValue(){
        Transmit( (0<<WEN) | (1<<RW) | (AD7792_DATA_REGISTER<<RS0) );
        uint8_t m1 = Read();
        uint8_t m2 = Read();
        return ( (m1<<8) | m2 );
    }

    void RequestData(){
        tx_data_[0] = (0<<WEN) | (1<<RW) | (AD7792_DATA_REGISTER<<RS0);
        Transmit(1);
    }

    std::pair<uint8_t, uint8_t> Calibration() {
        //Reset();
        auto modeBefore = Get_Mode();

        Mode_Set((1<<7)|(0<<6)|(0<<5), (fADC_16_7Hz & 0x0f)); //internal zero offest calib
        Mode_Set((1<<7)|(0<<6)|(1<<5), (fADC_16_7Hz & 0x0f)); //internal full scale calib
        Mode_Set(modeBefore >> 8, modeBefore & 0x00FF);

        return {Get_Offset(), Get_FullScale()};
    }

    std::pair<uint8_t, uint8_t> ReadCalibration() {
        return {Get_Offset(), Get_FullScale()};
    }

    auto WRFullCalibration(uint8_t H, uint8_t L) {
        auto modeBefore = Get_Mode();
        Mode_Set( (0<<7)|(1<<6)|(0<<5), (fADC_16_7Hz & 0x0f) );
        FullScale_Write(H,L);
        std::pair<uint8_t, uint8_t> retV {Get_Offset(), Get_FullScale()};
        Mode_Set(modeBefore >> 8, modeBefore & 0x00FF);
        return retV;
    }

    void WRCalibRegister(uint8_t H, uint8_t L) {
        auto modeBefore = Get_Mode();
        Mode_Set( (0<<7)|(1<<6)|(0<<5), (fADC_16_7Hz & 0x0f) );
        FullScale_Write(H,L);
        Mode_Set(modeBefore >> 8, modeBefore & 0x00FF);
    }

    float GetResistanceFormCode(uint32_t code, uint8_t gain, float resReference) {
        return ( (code * 2 * resReference) / (65535 * gain) );
    }

    uint8_t Get_Status(){
        Transmit( (0<<WEN) | (1<<RW) | (AD7792_STATUS_REGISTER<<RS0) );
        return Read();
    }

    uint32_t Get_Mode(){
        Transmit( (0<<WEN) | (1<<RW) | (AD7792_MODE_REGISTER<<RS0) );
        auto m1 = Read();
        auto m2 = Read();
        return ( (m1<<8) | m2 );
    }

    uint32_t Get_Data(){
        Transmit( (0<<WEN) | (1<<RW) | (AD7792_DATA_REGISTER<<RS0) );
        auto m1 = Read();
        auto m2 = Read();
        return ( (m1<<8) | m2 );
    }

    auto Get_IO(){
        Transmit( (0<<WEN) | (1<<RW) | (AD7792_IO_REGISTER<<RS0) );
        return Read();
    }

    int Get_FullScale(){
        Transmit( (0<<WEN) | (1<<RW) | (AD7792_FULLSCALE_REGISTER<<RS0) );
        auto m1 = Read();
        auto m2 = Read();
        return ( (m1<<8) | m2 );
    }

    int Get_Offset(){
        Transmit((0<<WEN)|(1<<RW)|(AD7792_OFFSET_REGISTER<<RS0));
        auto m1 = Read();
        auto m2 = Read();
        return ( (m1<<8) | m2 );
    }

    int Get_Config () {
        Transmit( (0<<WEN) | (1<<RW) | (AD7792_CONFIGURATION_REGISTER<<RS0) );
        auto m1 = Read();
        auto m2 = Read();
        return ( (m1<<8) | m2 );
    }

private:
    HardWareTransmitT transmit_f_;
    uint8_t tx_data_[8];

//    template<typename T, std::size_t Size>
//    void Transmit (T(&v)[Size]){
//        transmit_f_(v, Size);
//    }

    void Transmit (uint8_t size){
        transmit_f_(tx_data_, size);
    }

    uint8_t Read(){
        return {};
    }
    void FullScale_Write (uint8_t H, uint8_t L)
    {
        tx_data_[0] = (0<<WEN) | (0<<RW) | (AD7792_FULLSCALE_REGISTER<<RS0);
        tx_data_[1] = H;
        tx_data_[2] = L;
        Transmit(3);
    }
};

}// namespace AD7792_adc