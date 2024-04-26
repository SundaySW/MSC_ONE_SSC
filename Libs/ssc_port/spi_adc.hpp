#pragma once

#include "stm32g4xx_hal.h"

#include "protos_core/base_param.h"
#include "embedded_hw_utils/IO/pin.hpp"

#include "ad7792.hpp"
#include "ad7792_specs.hpp"
#include "async_tim_tasks.hpp"
#include "embedded_hw_utils/connectivity/spi/spi_driver.hpp"

#include "Coro/coro_mutex.hpp"

#include "ow_table.hpp"
#include "platinum_thermistor.hpp"

using namespace AD7792_adc;
using namespace connectivity;

struct SpiADC
{
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
        task_n_ = PLACE_ASYNC_TASK_SUSPENDED([&]{
//            cont_conv_coro.Resume();
            RequestADCValue();
        }, 5);
    }

    SPI_HandleTypeDef* GetSpiHandler(){
        return hspi_;
    }

    void Enable(){
        LoadOWData();
        average_value_.Reset();
        RESUME_TASK(task_n_);
    }

    void Disable() const{
        STOP_TASK(task_n_);
    }

    char* GetTablePtr(){
        return ow_table_.data();
    }

    bool GetValue(float& value)
    {
        if(average_value_.Ready()){
            value = CalcValue();
            return true;
        }
        return false;
    }

private:
    AD7792_adc::AD7792 ad7792{
        [&](std::pair<uint8_t*, std::size_t> ptr_size){
            SPI_DRIVER_.PlaceTask(&cs_pin_, ptr_size);
        }
    };
    static inline CoroMutex coro_mutex_;
    SPI_HandleTypeDef* hspi_;
    PIN_BOARD::PIN<PIN_BOARD::PinWriteable> cs_pin_;
    PIN_BOARD::PIN<PIN_BOARD::PinSwitchable> miso_rdy_pin_;
    int task_n_{-1};
    bool enabled {false};
    PRTD ntc_;
    OWTable ow_table_;

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


    void LoadOWData(){
//        ntc_.StoreRref(ow_table_.GetRref());
//        ntc_.StoreR0(ow_table_.GetR0());
//        ntc_.StorePlatinumData(ow_table_.GetPlatinumData());
    }

    void InitADCChip(){
        ad7792.Init(current_1mA, fADC_16_7Hz, gain_1, external, AIN1);
    }

    float CalcValue()
    {
        return ntc_.CalcT(average_value_.GetAverageValue());
    }

    float LookUpItTable(uint16_t adc_v){
        return adc_v;
    }

    void RequestADCValue(){
        SPI_DRIVER_.PlaceTask(&cs_pin_, ad7792.RequestDataCmd(), 2, [&](const uint8_t* data){
            average_value_.PlaceToStorage( (data[0]<<8) | data[1] );
        });
    }

    Event pin_ev_;
    CoroTask<> single_conv_coro = SingleConversion();
    CoroTask<> SingleConversion(){
        int pin_wait_task_n = PLACE_ASYNC_TASK_SUSPENDED_QUICKEST([&]{
            if(!miso_rdy_pin_.getState()){
                SPI_DRIVER_.PlaceTask(ad7792.RequestDataCmd(), 2, [&](const uint8_t* data){
                    average_value_.PlaceToStorage( (data[0]<<8) | data[1] );
                    cs_pin_.setValue(PIN_BOARD::HIGH);
                });
                STOP_TASK(pin_wait_task_n);
                pin_ev_.Notify();
            }
        });
        while (true){
            if(coro_mutex_.TryLock()) {
                cs_pin_.setValue(PIN_BOARD::LOW);
                SPI_DRIVER_.PlaceTask(ad7792.SingleConversionCmd(), [&](uint8_t *) {});
                RESUME_TASK(pin_wait_task_n);
                co_await pin_ev_;
                coro_mutex_.UnLock();
                co_yield {};
            }
            co_yield {};
        }
    }

    CoroTask<> cont_conv_coro = ContConversion();
    CoroTask<> ContConversion(){
        int pin_wait_task_n = PLACE_ASYNC_TASK_SUSPENDED_QUICKEST([&]{
            if(!miso_rdy_pin_.getState()){
                SPI_DRIVER_.PlaceTask(ad7792.RequestDataCmd(), 2, [&](const uint8_t* data){
                    average_value_.PlaceToStorage( (data[0]<<8) | data[1] );
                    cs_pin_.setValue(PIN_BOARD::HIGH);
                });
                STOP_TASK(pin_wait_task_n);
                pin_ev_.Notify();
            }
        });
        while (true){
            if(coro_mutex_.TryLock()){
                cs_pin_.setValue(PIN_BOARD::LOW);
                RESUME_TASK(pin_wait_task_n);
                co_await pin_ev_;
                coro_mutex_.UnLock();
                co_yield {};
            }
            co_yield {};
        }
    }
};