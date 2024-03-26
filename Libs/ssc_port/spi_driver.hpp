#pragma once

#include <utility>

#include "optional"
#include "functional"

#include "stm32g4xx_hal.h"

#include "HW/IO/pin.hpp"
#include "OneWire/task_queue.hpp"

//#include "function2/function2.hpp"
//#define call_back_capacity (32U)

#define SPI_TRANSMIT_RECEIVE_(data_ptr, s_tx, s_rx, pin_ref, cb) SPI_Driver::global().PlaceTask(data_ptr, s_tx, s_rx, pin_ref, cb)
#define SPI_TRANSMIT_(data_ptr, s_tx, pin_ref, cb) SPI_Driver::global().PlaceTask(data_ptr, s_tx, pin_ref, cb)
#define SPI_RECEIVE_(s_rx, pin_ref, cb) SPI_Driver::global().PlaceTask(s_rx, pin_ref, cb)
#define SPI_CLEAR_() SPI_Driver::global().ClearQueue()
#define SPI_POLL() SPI_Driver::global().ProcessTask()

#define buffer_size (4)
#define task_q_size (20)

struct SPI_Driver {
//    using CallBackT = fu2::function_base<true, false, fu2::capacity_fixed<call_back_capacity>,
//            false, false, void()>;
    enum TaskType{
        transmit,
        receive,
        transmit_receive
    };
    using CallBackT = std::function<void(uint8_t*)>;


    SPI_Driver(SPI_HandleTypeDef* hspi)
    {
        spi_handler_ = hspi;
    }

    void SetHandler(SPI_HandleTypeDef* spiHandle){
        spi_handler_ = spiHandle;
    }

    void TxCallbackHandler(){
        if(current_task_.Type() == transmit_receive)
            return;
        FinishTask();
    }
    void RxCallbackHandler(){
        FinishTask();
    }

    void ClearQueue(){
        tasks_.clear();
        in_process_ = false;
    }

    void FinishTask(){
        if(in_process_){
            if(current_task_.Type() != transmit)
                current_task_.CallBack();
            current_task_.ChipRelease();
            in_process_ = false;
        }
    }

    static SPI_Driver& global(){
        static auto instance = SPI_Driver(spi_handler_);
        return instance;
    }

    void ProcessTask(){
        if(!tasks_.empty() && !in_process_){
            in_process_ = true;
            current_task_ = tasks_.front();
            tasks_.pop();
            current_task_.ChipSelect();
            switch (current_task_.Type()){
                case transmit_receive:
                    HAL_SPI_Transmit_DMA(spi_handler_, current_task_.TxData(), current_task_.TxSize());
                    HAL_SPI_Receive_DMA(spi_handler_, current_task_.RxData(), current_task_.RxSize());
                    break;
                case transmit:
                    HAL_SPI_Transmit_DMA(spi_handler_, current_task_.TxData(), current_task_.TxSize());
                    break;
                case receive:
                    HAL_SPI_Receive_DMA(spi_handler_, current_task_.RxData(), current_task_.RxSize());
                    break;
            }
        }
    }

    void PlaceTask(auto&& ... args){
        tasks_.push(SpiTask(std::forward<decltype(args)>(args)...));
        ProcessTask();
    }

    auto operator()(){
        return spi_handler_;
    }
private:
    struct SpiTask{
        SpiTask() = default;
        SpiTask(const uint8_t* tx_data, std::size_t tx_size, std::size_t rx_size, PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* pin, CallBackT call_back, TaskType type = transmit_receive)
            :pin_(pin)
            ,call_back_(std::move(call_back))
            ,rx_size_(rx_size)
            ,tx_size_(tx_size)
            ,type_(type)
        {
            if(tx_size <= sizeof tx_data_)
                std::memcpy(tx_data_, tx_data, tx_size);
        }

        SpiTask(std::size_t size, PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* pin, CallBackT call_back, TaskType type = receive)
            :pin_(pin)
            ,call_back_(std::move(call_back))
            ,rx_size_(size)
            ,type_(type)
        {}

        SpiTask(const uint8_t* tx_data, std::size_t tx_size, PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* pin, CallBackT call_back, TaskType type = transmit)
            :pin_(pin)
            ,call_back_(std::move(call_back))
            ,tx_size_(tx_size)
            ,type_(type)
        {
            if(tx_size <= sizeof tx_data_)
                std::memcpy(tx_data_, tx_data, tx_size);
        }
        void CallBack(){
            call_back_(rx_data);
        }
        void ChipRelease(){
//            pin_->setValue(PIN_BOARD::HIGH);
        }
        void ChipSelect(){
            pin_->setValue(PIN_BOARD::LOW);
        }
        uint8_t* TxData(){ return tx_data_; }
        uint8_t* RxData(){ return rx_data; }
        TaskType Type(){ return type_; }
        [[nodiscard]] std::size_t TxSize() const { return tx_size_; }
        [[nodiscard]] std::size_t RxSize() const { return rx_size_; }
        PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* pin_ { nullptr };
    private:
        TaskType type_;
        uint8_t rx_data[buffer_size];
        uint8_t tx_data_[buffer_size];
        std::size_t tx_size_;
        std::size_t rx_size_;
        CallBackT call_back_;
    };

    bool in_process_{false};
    SpiTask current_task_;
    TaskQueue<SpiTask, task_q_size> tasks_;
    static inline SPI_HandleTypeDef* spi_handler_;
};

extern "C"
{
    void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
        if(hspi == SPI_Driver::global()())
            SPI_Driver::global().TxCallbackHandler();
    }

    void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
        if(hspi == SPI_Driver::global()())
            SPI_Driver::global().RxCallbackHandler();
    }
}