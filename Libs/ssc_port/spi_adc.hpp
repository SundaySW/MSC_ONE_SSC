#pragma once

#include <cmath>

#include "stm32g4xx_hal.h"

#include "protos_core/base_param.h"
#include "HW/IO/pin.hpp"

#include "ad7792.hpp"
#include "ad7792_specs.hpp"
#include "async_tim_tasks.hpp"
#include "spi_driver.hpp"

#define ow_eeprom_size      144

using namespace AD7792_adc;

class SpiADC
{
public:
    enum WaitingFor{
        data
    };

    SpiADC(PIN_BOARD::PIN<PIN_BOARD::PinWriteable> _ss_pin)
        : cs_pin_(_ss_pin)
    {}

    void SetSPI(SPI_HandleTypeDef* hspi){
        hspi_ = hspi;
        SPI_Driver::global().SetHandler(hspi);
    }

    void Start()
    {
        InitADCChip();
    }

    void InitADCChip(){
        ad7792.Init(RTD_2currentSources, current_1mA, fADC_16_7Hz, gain_1, external, AIN1);
    }

    float CalcValue()
    {
        return CalcT(average_value_.GetAverageValue());
    }

    float LookUpItTable(uint16_t adc_v){
        return adc_v;
    }

    [[nodiscard]] float CalcT2(float adc_v) const{
        float Rt = ntc_calib_.R_ref * (adc_v / UINT16_MAX),
              C = 1 - Rt / ntc_calib_.R0,
              D = ntc_calib_.b_sq - (4 * ntc_calib_.a * C);
        return (-ntc_calib_.b + sqrtf(D)) / (2 * ntc_calib_.a);
    }

    [[nodiscard]] float CalcT(float adc_v) const{
        auto Rt = ntc_calib_.R_ref * (adc_v / UINT16_MAX),
             c = 1 - Rt / ntc_calib_.R0,
             D = ntc_calib_.b_d2sq - ntc_calib_.a * c;
        return ntc_calib_.r + (sqrtf(D) / ntc_calib_.a);
    }

    void RequestADCValue(){
        auto pair_ptr_size = ad7792.RequestDataCmd();
        SPI_TRANSMIT_RECEIVE_(pair_ptr_size.first, pair_ptr_size.second, 2, &cs_pin_, [&](const uint8_t* data){
            average_value_.PlaceToStorage( (data[0]<<8) | data[1] );
        });
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
        average_value_.Reset();
//        PLACE_ASYNC_TASK([&]{
//            SPI_POLL();
//        }, 500);
        task_n_ = PLACE_ASYNC_TASK([&]{
            RequestADCValue();
        }, 1000);
    }

    void Disable(){
        if(task_n_ != -1)
            REMOVE_TASK(task_n_);
    }

    char* GetTablePtr(){
        return table_.data();
    }

private:
    SPI_HandleTypeDef* hspi_;
    PIN_BOARD::PIN<PIN_BOARD::PinWriteable> cs_pin_;
    int task_n_{-1};

    struct{
        float R_ref = 270,
              R0 = 100;
        float A = 3.9083e-3,
              B = -5.775e-7,
              C = -4.183e-12;
        float& a = B,
               b = A;
        float  b_d2 = b/2,
               r = - (b_d2 / a),
               b_sq = b * b,
               b_d2sq = b_d2 * b_d2;
    }ntc_calib_;

    struct{
        long long values{0};
        std::size_t count{0};
        void Reset(){
            values = 0;
            count = 0;
        }
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
    std::array<char, ow_eeprom_size> table_;
    bool enabled {false};
    AD7792_adc::AD7792 ad7792{
        [&](uint8_t* ptr, uint8_t size){
            SPI_TRANSMIT_(ptr, size, &cs_pin_, [&](uint8_t*){});
        }
    };
};