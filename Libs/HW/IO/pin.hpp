
#ifndef SND_STM32_PIN_HPP
#define SND_STM32_PIN_HPP

#include <cstdint>
#include "type_traits"

#include "stm32g431xx.h"

namespace PIN_BOARD{

enum LOGIC_LEVEL{
    LOW = 0,
    HIGH = 1,
};

struct PinReadable
{
};

struct PinWriteable
{
};

struct PinSwitchable
{
};

template<typename InterfaceType>
class PIN{
public:
    template<typename T = InterfaceType>
    requires(std::is_base_of<PinReadable, T>::value || std::is_base_of<PinSwitchable, T>::value)
    constexpr inline LOGIC_LEVEL getState(){
        currentState_ = getValue();
        return currentState_;
    }

    template<typename T = InterfaceType>
    requires(std::is_base_of<PinWriteable, T>::value || std::is_base_of<PinSwitchable, T>::value)
    constexpr inline void setValue(LOGIC_LEVEL value){
        if(inverted_){
            if (value) port_->BRR = (uint32_t)pin_;
            else port_->BSRR = (uint32_t)pin_;
        }else{
            if (value) port_->BSRR = (uint32_t)pin_;
            else port_->BRR = (uint32_t)pin_;
        }
        currentState_ = value;
    }

    template<typename T = InterfaceType>
    requires(std::is_base_of<PinWriteable, T>::value || std::is_base_of<PinSwitchable, T>::value)
    constexpr inline void togglePinState(){
        uint32_t odr = port_->ODR;
        port_->BSRR = ((odr & pin_) << 16U) | (~odr & pin_);
    }

    constexpr void setInverted() {
        inverted_ = true;
    }

    [[nodiscard]] constexpr inline uint16_t getPin() const{
        return pin_;
    }

    [[nodiscard]] constexpr LOGIC_LEVEL* GetPinStatePtr(){
        return static_cast<LOGIC_LEVEL*>(&currentState_);
    }

    constexpr void setAsOutput(){
        port_->MODER = (port_->MODER & ~(0x03 << (2 * pin_))) | (0x01 << (2 * pin_));
    }

    constexpr void setAsInput(){
        port_->MODER = port_->MODER & (0x03 << (2 * pin_));
    }

    constexpr explicit PIN(GPIO_TypeDef* incomePortPtr, uint16_t incomePin)
            : port_(incomePortPtr),
              pin_(incomePin)
    {};

protected:
private:
    LOGIC_LEVEL currentState_ = LOW;
    GPIO_TypeDef* port_;
    uint16_t pin_;
    bool inverted_ = false;

    template<typename T = InterfaceType>
    requires(std::is_base_of<PinReadable, T>::value || std::is_base_of<PinSwitchable, T>::value)
    constexpr inline LOGIC_LEVEL getValue(){
        LOGIC_LEVEL retVal;
        if((port_->IDR & pin_) != (uint32_t)LOGIC_LEVEL::LOW)
            retVal = LOGIC_LEVEL::HIGH;
        else
            retVal = LOGIC_LEVEL::LOW;
        if(inverted_)
            return (retVal ? LOGIC_LEVEL::LOW : LOGIC_LEVEL::HIGH);
        else
            return retVal;
    }
};

} // namespace PIN_BOARD

#endif //SND_STM32_PIN_HPP