
#pragma once

#include "input_signal.hpp"

class Button: public InputSignal{
public:
    constexpr explicit Button(GPIO_TypeDef* port, uint16_t pin, uint32_t debounce_time) noexcept
        : InputSignal(port, pin, debounce_time)
    {
    }

    constexpr uint16_t operator()() const{
        return pin_.getPin();
    }
    constexpr bool operator==(uint16_t otherPin) const{
        return pin_.getPin() == otherPin;
    }
};