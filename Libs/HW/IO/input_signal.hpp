
#pragma once

#include "pin.hpp"

class InputSignal{
public:
    constexpr explicit InputSignal(GPIO_TypeDef* port, uint16_t pin, uint32_t debounce_time) noexcept
        : pin_(port, pin),
        debounce_time_(debounce_time)
    {
        SetState(pin_.getState());
    }

    constexpr void Update() {
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

protected:
    PIN_BOARD::PIN<PIN_BOARD::PinReadable> pin_;

private:
    PIN_BOARD::LOGIC_LEVEL signal_state_;
    uint32_t debounce_time_;
    uint32_t active_time_{0};

    constexpr void SetState(PIN_BOARD::LOGIC_LEVEL level){
        signal_state_ = level;
    }
};