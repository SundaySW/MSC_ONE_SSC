#pragma once

#include "pin.hpp"

struct Button{
    constexpr explicit Button(GPIO_TypeDef* port, uint16_t pin) noexcept
        : pin_(port, pin)
    {
//        pin_.setInverted();
    }

    PIN_BOARD::PIN<PIN_BOARD::PinReadable> pin_;

    constexpr uint16_t operator()() const{
        return pin_.getPin();
    }
    constexpr bool operator==(uint16_t otherPin) const{
        return pin_.getPin() == otherPin;
    }
    constexpr PIN_BOARD::LOGIC_LEVEL getState(){
        return pin_.getState();
    }
};
