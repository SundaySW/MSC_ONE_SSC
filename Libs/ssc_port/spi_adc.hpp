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

    SpiADC(PIN_BOARD::PIN<PIN_BOARD::PinWriteable> _ss_pin, PIN_BOARD::PIN<PIN_BOARD::PinSwitchable> _miso_rdy_pin)
        :cs_pin_(_ss_pin)
        ,miso_rdy_pin_(_miso_rdy_pin)
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
        SPI_DRIVER_.PlaceTask(ad7792.RequestDataCmd(), 2, &cs_pin_, [&](const uint8_t* data){
            average_value_.PlaceToStorage( (data[0]<<8) | data[1] );
        });
    }

    bool GetValue(float& value)
    {
        if(average_value_.Ready()){
            value = CalcValue();
            return true;
        }
        return false;
    }

    SPI_HandleTypeDef* GetSpiHandler(){
        return hspi_;
    }

    void Enable(){
        average_value_.Reset();
//        PLACE_ASYNC_TASK([&]{
//            SPI_POLL();
//        }, 100);
        task_n_ = PLACE_ASYNC_TASK([&]{
            RequestADCValue();
//            single_conv_coro.Resume();
        }, 100);
    }

    void Disable(){
        if(task_n_ != -1)
            REMOVE_TASK(task_n_);
    }

    char* GetTablePtr(){
        return table_.data();
    }

    Event pin_ev_;
    CoroTask<> single_conv_coro = SingleConversion(single_conv_coro);
    CoroTask<> SingleConversion(decltype(single_conv_coro)& this_coro_){
        int pin_wait_task_n;
        bool state;
        while (true){
            cs_pin_.setValue(PIN_BOARD::LOW);
            SPI_DRIVER_.PlaceTask(ad7792.SingleConversionCmd1());
            SPI_DRIVER_.PlaceTask(ad7792.SingleConversionCmd2(), [&](uint8_t*){
                miso_rdy_pin_.setAsInput();
                pin_wait_task_n = PLACE_ASYNC_TASK([&]{
                    state = miso_rdy_pin_.getState();
                    if(!miso_rdy_pin_.getState()){
                        miso_rdy_pin_.SetAFSpi1();
                        pin_ev_.Notify();
                    }
                },1);
            });
            co_await pin_ev_;
            REMOVE_TASK(pin_wait_task_n);
            SPI_DRIVER_.PlaceTask(ad7792.SingleConversionCmd3(), 2, [&](const uint8_t* data){
                average_value_.PlaceToStorage( (data[0]<<8) | data[1] );
            });
            cs_pin_.setValue(PIN_BOARD::HIGH);
            co_yield {};
        }
    }

private:
    SPI_HandleTypeDef* hspi_;
    PIN_BOARD::PIN<PIN_BOARD::PinWriteable> cs_pin_;
    PIN_BOARD::PIN<PIN_BOARD::PinSwitchable> miso_rdy_pin_;
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
        [[nodiscard]] bool Ready() const{
            return count;
        }
    }average_value_;

    std::array<char, ow_eeprom_size> table_;
    bool enabled {false};

    AD7792_adc::AD7792 ad7792{
        [&](auto pair_ptr_size){
            SPI_DRIVER_.PlaceTask(pair_ptr_size, &cs_pin_);
        }
    };
};