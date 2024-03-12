#pragma once

#include "one_wire.hpp"
#include "spi_adc.hpp"

#define CastArg(void_ptr, type)  std::launder(static_cast<std::optional<type>*>(void_ptr))

#define ow_port_update_rate     (5)

struct SSCPort{
    SSCPort(SPI_HandleTypeDef* spi, TIM_HandleTypeDef* ow_tim,
                        PIN_BOARD::PIN<PIN_BOARD::PinWriteable> cs_pin,
                        PIN_BOARD::PIN<PIN_BOARD::PinSwitchable> ow_pin,
                        BaseParam* _param)
        : param_(_param)
        , ow_port_(ow_pin, ow_tim)
        , adc_(spi, cs_pin)
    {
        send_rate_ = param_->GetSendRate();
    }

    void Update(){
        if(static uint8_t ow_port_update_count_; ow_port_update_count_++ == ow_port_update_rate){
            ow_port_update_count_ = 0;
            if(ow_port_.IsNewConnected())
                CoroTaskReadCalibData();
        }
        ow_port_.Poll();
    }

    void CoroTaskReadCalibData(){
        uint16_t ow_page_offset_addr = 0;
        uint8_t position = 0;
        for(std::size_t i = 0; i < 32; i++){
            ow_port_.PlaceTask(OneW_Coro::read_memory, offset, [&, position](void* ret_val_ptr){
                using d = typename std::array<uint8_t, 8>;
                auto* ret_val = CastArg(ret_val_ptr, d);
                if(ret_val->has_value()){
                    auto& v = ret_val->value();
                    adc_.PlaceInTable(position, v);
                }
            });
            ow_page_offset_addr += 8;
            position++;
        }
    }

    void SPIHandler(){
        adc_.OnCallBack();
    }

    void Start(){

    }

    void EnableParam(){
        param_->SetSendRate(send_rate_);
    }

    void DisableParam(){
        param_->SetSendRate(0);
    }
private:
    OneWirePort ow_port_;
    SpiADC adc_;
    BaseParam* param_;
    unsigned short send_rate_;
};
