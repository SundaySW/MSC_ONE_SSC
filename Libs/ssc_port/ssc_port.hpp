#pragma once
#include "OneWire/one_wire.hpp"
#include "spi_adc.hpp"
#include "ssc_port_param.hpp"
#include "HW/IO/input_signal.hpp"
#include "async_tim_tasks.hpp"

#define param_default_send_rate     (2000)
#define param_default_update_rate   (1000)

struct SSCPort{
    SSCPort() = default;
    SSCPort(PIN_BOARD::PIN<PIN_BOARD::PinWriteable> cs_pin,
            PIN_BOARD::PIN<PIN_BOARD::PinSwitchable> ow_pin,
            SSCPortParam& param)
        : ow_port_(ow_pin)
        , adc_(cs_pin)
        , param_(param)
        , connection_pin_(ow_pin.getPort(), ow_pin.getPin(), 100)
    {}

    void Init(SPI_HandleTypeDef* spi, TIM_HandleTypeDef* ow_tim){
        ow_port_.SetTim(ow_tim);
        adc_.SetSPI(spi);
        PLACE_ASYNC_TASK([&]
        {
            if(!ow_port_.IsInProcess())
                connection_pin_.Update();

        }, 10);
    }

    SpiADC* GetADC(){
        return &adc_;
    }

    void Update(){
        if(!ow_port_.IsInProcess()){
            switch (connection_pin_.GetPinConnectionState()) {
                case InputSignal::new_connection:
                    CoroTaskReadCalibData();
                    break;
                case InputSignal::last_disconnected:
                    DisableParam();
                    break;
                case InputSignal::connected:
                case InputSignal::no_device:
                default:
                    break;
            }
            ow_port_.Poll();
        }
    }

    void CoroTaskReadCalibData(){
        ow_port_.PlaceTask(OneW_Coro::read_memory_full, adc_.GetTablePtr(), [&](void* ret_val_ptr){
            EnableParam();
        });
    }

    void SPIHandler(){
        adc_.OnCallBack();
    }

    void Start(){
        adc_.Start();
    }

    void EnableParam(){
        adc_.Enable();
        param_.SetSendRate(param_send_rate_);
        param_.SetUpdateRate(param_update_rate_);
    }

    void DisableParam(){
        param_send_rate_ = param_.GetSendRate();
        param_update_rate_ = param_.GetUpdateRate();
        param_.SetSendRate(0);
        param_.SetUpdateRate(0);
        adc_.Disable();
    }
    SSCPortParam& param_;
    OneWirePort ow_port_;
    SpiADC adc_;
    InputSignal connection_pin_;
    unsigned short param_send_rate_ = param_default_send_rate;
    unsigned short param_update_rate_ = param_default_update_rate;
};