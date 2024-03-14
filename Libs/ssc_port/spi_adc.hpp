#pragma once

#include "array"

#include "stm32g4xx_hal.h"
#include "protos_core/base_param.h"
#include "HW/IO/pin.hpp"

#define spi_buffer_size     8
#define ow_eeprom_size      144

class SpiADC
{
public:
    using buffer_v_t = uint16_t;
    struct CalibData{
        CalibData() = default;
        CalibData(const std::array<uint8_t, 8>& data){

        }
        float c1{1};
        float c2{1};
    };

    SpiADC() = default;
    SpiADC(PIN_BOARD::PIN<PIN_BOARD::PinWriteable> _ss_pin)
        : cs_pin_(_ss_pin)
    {}

    void SetSPI(SPI_HandleTypeDef* hspi){
        hspi_ = hspi;
    }

    void Start()
    {
        src_buf[0] = 0xff;
        src_buf[1] = 0xff;
        src_buf[2] = 0xff;
        src_buf[3] = 0xff;

        cs_pin_.setValue(PIN_BOARD::LOW);
        HAL_SPI_TransmitReceive_DMA(hspi_, (uint8_t*)(src_buf), (uint8_t*)(dst_buf), (spi_buffer_size * sizeof(buffer_v_t)));
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

    bool GetValue(float& value)
    {
        value = CalcValue();
        return true;
    }

    SPI_HandleTypeDef * getHSpi(){
        return hspi_;
    }

    void RecalcCalibration(){

    }

    void Enable(){
        RecalcCalibration();
    }

    void Disable(){
    }

    char* GetTablePtr(){
        return table_.data();
    }

private:
    SPI_HandleTypeDef* hspi_;
    PIN_BOARD::PIN<PIN_BOARD::PinWriteable> cs_pin_;
    CalibData calib_data_;

    buffer_v_t src_buf[spi_buffer_size]{0};
    buffer_v_t dst_buf[spi_buffer_size]{0};
    std::array<char, ow_eeprom_size> table_;
};