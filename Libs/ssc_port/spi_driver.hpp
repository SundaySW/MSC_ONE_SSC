#pragma once

#include <utility>

#include "optional"
#include "functional"

#include "stm32g4xx_hal.h"

#include "HW/IO/pin.hpp"
#include "OneWire/task_queue.hpp"

//#include "function2/function2.hpp"
//#define call_back_capacity (32U)

#define SPI_TRANSMIT_(d, s, p, c)   SPI_Driver::global().Transmit(d, s, p, c)
#define SPI_RECEIVE_(d, s, p, c)   SPI_Driver::global().Receive(d, s, p, c)
#define SPI_CLEAR_()    SPI_Driver::global().ClearQueue()

struct SPI_Driver {
    using CallBackT = std::function<void()>;
//    using CallBackT = fu2::function_base<true, false, fu2::capacity_fixed<call_back_capacity>,
//            false, false, void()>;

    SPI_Driver(SPI_HandleTypeDef* hspi)
    {
        spi_handler_ = hspi;
    }

    void SetHandler(SPI_HandleTypeDef* spiHandle){
        spi_handler_ = spiHandle;
    }

    void TxHandler(){
        FinishTask();
    }

    void RxHandler(){
        FinishTask();
    }

    void ClearQueue(){
        tasks_.empty();
        in_process_ = false;
    }

    void FinishTask(){
        if(in_process_){
//            current_task_.ChipRelease();
            current_task_.FinishTask();
            in_process_ = false;
        }
    }

    static SPI_Driver& global(){
        static auto instance = SPI_Driver(spi_handler_);
        return instance;
    }

//    template<typename T, std::size_t Size>
//    void Transmit(T(&data)[Size], PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* cs_pin, const CallBackT& callBack){
//        current_task_.Set(cs_pin, callBack);
//        current_task_.ActivateCSPin();
//        HAL_SPI_Transmit_DMA(spi_handler_, data, Size);
//    }

    void ProcessTask(){
        if(!in_process_){
            in_process_ = true;
            current_task_ = tasks_.front();
            tasks_.pop();
            current_task_.ChipSelect();
            if(current_task_.IsTransmit())
                HAL_SPI_Transmit_DMA(spi_handler_, current_task_.Data(), current_task_.Size());
            else
                HAL_SPI_Receive_DMA(spi_handler_, current_task_.Data(), current_task_.Size());
        }
    }

    void Transmit(uint8_t* data, std::size_t size, PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* cs_pin, const CallBackT& callBack){
        tasks_.push(SpiTask{cs_pin, callBack, data, size, true});
        ProcessTask();
    }

    void Receive(uint8_t* data, std::size_t size, PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* cs_pin, const CallBackT& callBack){
        tasks_.push(SpiTask{cs_pin, callBack, data, size, false});
        ProcessTask();
    }
    SPI_HandleTypeDef* operator()(){
        return spi_handler_;
    }
private:
    struct SpiTask{
        SpiTask() = default;
        SpiTask(PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* pin, CallBackT call_back, uint8_t* data, std::size_t size, bool transmit)
            :pin_(pin)
            ,call_back_(std::move(call_back))
            ,data_(data)
            ,size_(size)
            ,transmit_(transmit)
        {}
        void FinishTask(){
            call_back_();
        }
        void ChipRelease(){
            pin_->setValue(PIN_BOARD::HIGH);
        }
        void ChipSelect(){
            pin_->setValue(PIN_BOARD::LOW);
        }
        [[nodiscard]] bool IsTransmit() const{ return transmit_; }
        uint8_t* Data(){ return data_; }
        std::size_t Size() { return size_; }
    private:
        bool transmit_;
        uint8_t* data_;
        std::size_t size_;
        PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* pin_ {nullptr};
        CallBackT call_back_;
    };
    bool in_process_{false};
    SpiTask current_task_;
    TaskQueue<SpiTask, 20> tasks_;
    static inline SPI_HandleTypeDef* spi_handler_;
};

extern "C"
{
//    void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
//        if(hspi == SPI_Driver::global()())
//            SPI_Driver::global().TxHandler();
//    }
    void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
        if(hspi == SPI_Driver::global()())
            SPI_Driver::global().TxHandler();
    }

    void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
        if(hspi == SPI_Driver::global()())
            SPI_Driver::global().RxHandler();
    }
}