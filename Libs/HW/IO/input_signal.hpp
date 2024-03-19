#pragma once

#include "pin.hpp"

struct InputSignal{
    enum PinConnectionState{
        connected,
        new_connection,
        last_disconnected,
        no_device
    };
    constexpr explicit InputSignal(GPIO_TypeDef* port, uint16_t pin, uint32_t debounce_time) noexcept
        : pin_(port, pin)
        , debounce_time_(debounce_time)
    {}

//    constexpr explicit InputSignal(PIN_BOARD::PIN<PIN_BOARD::PinReadable> pin, uint32_t debounce_time) noexcept
//        : pin_(pin)
//        , debounce_time_(debounce_time)
//    {}

    PinConnectionState GetPinConnectionState(){
        auto retV = no_device;
        if(signal_state_ && last_pin_connection_state_)
            retV = connected;
        else if(signal_state_ && !last_pin_connection_state_)
            retV = new_connection;
        else if(!signal_state_ && last_pin_connection_state_)
            retV = last_disconnected;
        last_pin_connection_state_ = signal_state_;
        return retV;
    }

    constexpr void UpdatePin() {
        if(pin_.getState()){
            if(active_time_ < debounce_time_)
                active_time_++;
        }else
            active_time_ = 0;

        if(active_time_ >= debounce_time_)
            SetState(PIN_BOARD::HIGH);
        else
            SetState(PIN_BOARD::LOW);
    }

    constexpr void SetDebounceTime(uint32_t time){
        debounce_time_ = time;
    }

    constexpr void InvertSignalPin(){
        pin_.setInverted();
    }

    [[nodiscard]] constexpr PIN_BOARD::LOGIC_LEVEL getState() const {
        return signal_state_;
    }
private:
    PIN_BOARD::PIN<PIN_BOARD::PinReadable> pin_;
    PIN_BOARD::LOGIC_LEVEL signal_state_ {PIN_BOARD::LOW};
    uint32_t debounce_time_;
    uint32_t active_time_{0};
    bool last_pin_connection_state_{false};

    constexpr void SetState(PIN_BOARD::LOGIC_LEVEL level){
        signal_state_ = level;
    }
};