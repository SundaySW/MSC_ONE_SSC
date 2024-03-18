#pragma once

#include "array"

#include "stm32g4xx_hal.h"
#include "protos_core/base_param.h"
#include "HW/IO/pin.hpp"

#include "ad7792.hpp"

#include "ad7792_specs.hpp"

#define spi_buffer_size     8
#define ow_eeprom_size      144

using namespace AD7792_adc;

class SpiADC
{
public:
    using buffer_v_t = uint16_t;

    SpiADC(PIN_BOARD::PIN<PIN_BOARD::PinWriteable> _ss_pin)
        : cs_pin_(_ss_pin)
    {}

    void SetSPI(SPI_HandleTypeDef* hspi){
        hspi_ = hspi;
    }

    void Start()
    {
        ad7792.Init(RTD_2currentSources, current_210uA, fADC_16_7Hz, gain_1, external, AIN1);
    }

    void RxCallBack() {
//        cs_pin_.setValue(PIN_BOARD::HIGH);
    }
    void TxCallBack() {
//        cs_pin_.setValue(PIN_BOARD::HIGH);
    }

    float CalcValue()
    {
        float v = 1;
        return v;
    }

    bool GetValue(float& value)
    {
        value = CalcValue();
        return true;
    }

    SPI_HandleTypeDef * getSpiHandler(){
        return hspi_;
    }

    void Enable(){
    }

    void Disable(){
    }

    char* GetTablePtr(){
        return table_.data();
    }

private:
    SPI_HandleTypeDef* hspi_;
    PIN_BOARD::PIN<PIN_BOARD::PinWriteable> cs_pin_;

    buffer_v_t src_buf[spi_buffer_size]{0};
    buffer_v_t dst_buf[spi_buffer_size]{0};
    std::array<char, ow_eeprom_size> table_;

    AD7792_adc::AD7792 ad7792{
        [&](uint8_t* ptr, uint8_t size){
            cs_pin_.setValue(PIN_BOARD::LOW);
            HAL_SPI_Transmit_DMA(hspi_, ptr, size);
        }
    };
};