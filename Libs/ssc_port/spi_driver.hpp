#pragma once

#include <utility>

#include "optional"
#include "functional"

#include "stm32g4xx_hal.h"

#include "HW/IO/pin.hpp"
#include "OneWire/task_queue.hpp"

//#include "function2/function2.hpp"
//#define call_back_capacity (32U)

#define SPI_DRIVER_ SPI_Driver::global()
#define SPI_CLEAR_Q_() SPI_Driver::global().ClearQueue()
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

    explicit SPI_Driver(SPI_HandleTypeDef* hspi)
    {
        spi_handler_ = hspi;
    }

    static void SetHandler(SPI_HandleTypeDef* spiHandle){
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
        SpiTask(std::pair<const uint8_t*, std::size_t> data_size_pair, TaskType type = transmit)
                :tx_size_(data_size_pair.second)
                ,type_(type)
        {
            PlaceData(data_size_pair);
        }
        SpiTask(std::pair<const uint8_t*, std::size_t> data_size_pair, std::size_t rx_size, PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* pin, CallBackT call_back, TaskType type = transmit_receive)
            :pin_(pin)
            ,call_back_(std::move(call_back))
            ,rx_size_(rx_size)
            ,tx_size_(data_size_pair.second)
            ,type_(type)
        {
            PlaceData(data_size_pair);
        }
        SpiTask(std::pair<const uint8_t*, std::size_t> data_size_pair, std::size_t rx_size, CallBackT call_back, TaskType type = transmit_receive)
                :call_back_(std::move(call_back))
                ,rx_size_(rx_size)
                ,tx_size_(data_size_pair.second)
                ,type_(type)
        {
            PlaceData(data_size_pair);
        }
        SpiTask(std::size_t rx_size, PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* pin, CallBackT call_back, TaskType type = receive)
            :pin_(pin)
            ,call_back_(std::move(call_back))
            ,rx_size_(rx_size)
            ,type_(type)
        {}
        SpiTask(std::pair<const uint8_t*, std::size_t> data_size_pair, PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* pin, TaskType type = transmit)
                :pin_(pin)
                ,tx_size_(data_size_pair.second)
                ,type_(type)
        {
            PlaceData(data_size_pair);
        }
        SpiTask(std::pair<const uint8_t*, std::size_t> data_size_pair, CallBackT call_back, TaskType type = transmit)
                :call_back_(std::move(call_back))
                ,tx_size_(data_size_pair.second)
                ,type_(type)
        {
            PlaceData(data_size_pair);
        }
        SpiTask(std::pair<const uint8_t*, std::size_t> data_size_pair, PIN_BOARD::PIN<PIN_BOARD::PinWriteable>* pin, CallBackT call_back, TaskType type = transmit)
            :pin_(pin)
            ,call_back_(std::move(call_back))
            ,tx_size_(data_size_pair.second)
            ,type_(type)
        {
            PlaceData(data_size_pair);
        }

        void PlaceData(auto data_size_pair){
            auto&& [tx_data_p, size] = data_size_pair;
            if(size <= sizeof tx_data_)
                std::memcpy(tx_data_, tx_data_p, size);
        }
        void CallBack(){
            if(call_back_)
                call_back_.value()(rx_data);
        }
        void ChipRelease(){
//            if(pin_)
//                pin_.value()->setValue(PIN_BOARD::HIGH);
        }
        void ChipSelect(){
            if(pin_)
                pin_.value()->setValue(PIN_BOARD::LOW);
        }
        uint8_t* TxData(){ return tx_data_; }
        uint8_t* RxData(){ return rx_data; }
        TaskType Type(){ return type_; }
        [[nodiscard]] std::size_t TxSize() const { return tx_size_; }
        [[nodiscard]] std::size_t RxSize() const { return rx_size_; }
        std::optional<PIN_BOARD::PIN<PIN_BOARD::PinWriteable>*> pin_;
    private:
        TaskType type_;
        uint8_t rx_data[buffer_size];
        uint8_t tx_data_[buffer_size];
        std::size_t tx_size_;
        std::size_t rx_size_;
        std::optional<CallBackT> call_back_;
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