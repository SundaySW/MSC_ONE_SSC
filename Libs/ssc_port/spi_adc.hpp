#pragma once

#include "array"

#include "stm32g4xx_hal.h"
#include "protos_core/base_param.h"
#include "HW/IO/pin.hpp"

#include "ad7792.hpp"

#include "ad7792_specs.hpp"

#include "async_tim_tasks.hpp"

#define spi_buffer_size     8
#define ow_eeprom_size      144

using namespace AD7792_adc;

class SpiADC
{
public:
    using buffer_v_t = uint8_t;

    enum WaitingFor{
        data
    };

    SpiADC(PIN_BOARD::PIN<PIN_BOARD::PinWriteable> _ss_pin)
        : cs_pin_(_ss_pin)
    {}

    void SetSPI(SPI_HandleTypeDef* hspi){
        hspi_ = hspi;
    }

    void Start()
    {
        cs_pin_.setValue(PIN_BOARD::LOW);
        ad7792.Init(RTD_2currentSources, current_210uA, fADC_16_7Hz, gain_1, external, AIN1);
    }

    void RxCallBack() {
        switch (waitingFor) {
            case data:
                average_value_.PlaceToStorage( (rx_buf[0]<<8) | rx_buf[1] );
                break;
        }
    }
    void TxCallBack() {
    }

    float CalcValue()
    {
        return LookUpItTable(average_value_.GetAverageValue());
    }

    float LookUpItTable(uint16_t adc_v){
        return 1;
    }

    void RequestADCValue(){
        waitingFor = data;
        tx_buf[0] = (0<<WEN) | (1<<RW) | (AD7792_DATA_REGISTER<<RS0);
        HAL_SPI_Transmit_DMA(hspi_, tx_buf, 1);
        HAL_SPI_Receive_DMA(hspi_, rx_buf, 2);
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
        task_n_ = PLACE_ASYNC_TASK([&]{
                RequestADCValue();
            }, 100);
    }

    void Disable() const{
        if(task_n_ != -1)
            REMOVE_TASK(task_n_);
    }

    char* GetTablePtr(){
        return table_.data();
    }

private:
    SPI_HandleTypeDef* hspi_;
    PIN_BOARD::PIN<PIN_BOARD::PinWriteable> cs_pin_;
    WaitingFor waitingFor;
    int task_n_{-1};
    struct {
        long long values{0};
        std::size_t count{0};
        uint16_t GetAverageValue(){
            if(count){
                uint16_t v = values / count;
                values = 0;
                count = 0;
                return v;
            }
            else return 0;
        }
        void PlaceToStorage(uint16_t v){
            values += v;
            count++;
        };
    }average_value_;

    buffer_v_t tx_buf[spi_buffer_size]{0};
    buffer_v_t rx_buf[spi_buffer_size]{0};
    std::array<char, ow_eeprom_size> table_;

    AD7792_adc::AD7792 ad7792{
        [&](uint8_t* ptr, uint8_t size){
            HAL_SPI_Transmit_DMA(hspi_, ptr, size);
        }
    };
};