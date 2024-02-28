#pragma once

#include "array"

#include "stm32g4xx_hal.h"
#include "protos_core/base_param.h"
#include "HW/IO/pin.hpp"

class IAdc
{
public:
    virtual bool GetValue(float& value) = 0;
};

class SpiADC : public IAdc
{
public:
    struct CalibData{
        CalibData() = default;
        CalibData(const std::array<uint8_t, 8>& data){

        }
        float c1;
        float c2;
    };

    SpiADC() = delete;
    SpiADC(SPI_HandleTypeDef* _hspi, PIN_BOARD::PIN<PIN_BOARD::PinWriteable> _ss_pin)
        :hspi_(_hspi)
        , cs_pin_(_ss_pin)
    {}

    void Start()
    {
        cs_pin_.setValue(PIN_BOARD::LOW);
//        HAL_TIM_IC_Start_DMA(htim, tim_channel, (uint32_t*)&capture_result, 1);
        HAL_SPI_TransmitReceive_DMA(hspi_, (uint8_t*)(src_buf), (uint8_t *)(dst_buf), 128);
    }

    void OnCallBack() {
        cs_pin_.setValue(PIN_BOARD::HIGH);
    }

    float CalcValue()
    {
        float v = 1;
        auto res = (v * calib_data_.c1) + calib_data_.c2;
        return res;
    }

    void StoreCalibData(CalibData data){
        calib_data_ = std::move(data);
    }

    bool GetValue(float& value) override
    {
        value = CalcValue();
        return true;
    }

    SPI_HandleTypeDef * getHSpi(){
        return hspi_;
    }

private:
    SPI_HandleTypeDef* hspi_;
    PIN_BOARD::PIN<PIN_BOARD::PinWriteable> cs_pin_;
    CalibData calib_data_;
    uint16_t src_buf[1024] = {0};
    uint16_t dst_buf[1024] = {0};
};

template<typename Func>
class SpiADCParam : public UpdateParam, public FloatParam, public CalibrParam
{
public:
    SpiADCParam() = default;
    SpiADCParam(IAdc* _tim, Func f)
        :Tim(_tim)
        , saveToEEPROM(f)
    {}

    bool QueryInterface(INTERFACE_ID iid, void*& p) override
    {
        switch (iid)
        {
            case IID_UPDATABLE:
                p = (UpdateParam*)this;
                return true;
            case IID_CALIBRATEABLE:
                p = (CalibrParam*)this;
                return true;
            default:
                return false;
        }
    }

    void SaveToEEPROM() override {
        float data[2] = {Offset, Mult};
        saveToEEPROM(Id, data);
    }

    bool UpdateValue() override
    {
        float value;
        if (Tim->GetValue(value)){
            Value = Calibrate(value);
            return true;
        }
        return false;
    }

private:
    Func saveToEEPROM = nullptr;
    IAdc* Tim = nullptr;
};
