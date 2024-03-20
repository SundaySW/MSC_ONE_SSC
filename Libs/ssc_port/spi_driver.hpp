#pragma once

#include <utility>

#include "optional"
#include "functional"

#include "stm32g4xx_hal.h"

#include "HW/IO/pin.hpp"

//#include "function2/function2.hpp"
//#define call_back_capacity (32U)

struct SPI_Driver {
    using CallBackT = std::function<void()>;
//    using CallBackT = fu2::function_base<true, false, fu2::capacity_fixed<call_back_capacity>,
//            false, false, void()>;

    SPI_Driver(SPI_HandleTypeDef* hspi)
    {
        spi_handler_ = hspi;
    }

    void TxHandler(){
        current_task_.FinishTask();
    }
    void RxHandler(){
        current_task_.FinishTask();
    }

    static SPI_Driver& global(){
        static auto instance = SPI_Driver(spi_handler_);
        return instance;
    }

    template<typename T, std::size_t Size>
    void Transmit(T(&data)[Size], PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* cs_pin, const CallBackT& callBack){
        current_task_.Set(cs_pin, callBack);
        current_task_.ActivateCSPin();
        HAL_SPI_Transmit_DMA(spi_handler_, data, Size);
    }

private:
    struct{
        void Set(PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* pin, CallBackT call_back){
            pin_ = pin;
            call_back_ = std::move(call_back);
            is_set_ = true;
        }
        void FinishTask(){
            DeactiveCSPin();
            call_back_();
            is_set_ = false;
        }
        void ActivateCSPin(){ pin_->setValue(PIN_BOARD::LOW);}
        void DeactiveCSPin(){ pin_->setValue(PIN_BOARD::HIGH); }
        bool IsSet()const{ return is_set_; }
    private:
        PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* pin_ {nullptr};
        CallBackT call_back_;
        bool is_set_ {false};
    }current_task_;

    static inline SPI_HandleTypeDef* spi_handler_;
};

extern "C"
{
    void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
        SPI_Driver::global().TxHandler();
    }

    void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
        SPI_Driver::global().TxHandler();
    }
}