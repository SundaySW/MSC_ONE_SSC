#pragma once

#include <bit>
#include <cstring>

#include "HW/IO/pin.hpp"

#include "stm32g4xx_hal_spi.h"

#define W_CTRL_EN_256u      (uint16_t)0b0'000'1'0'0'1000'0'00'11
#define W_CTRL_EN_128u      (uint16_t)0b0'000'1'0'0'0111'0'00'11
#define W_CTRL_EN_64u       (uint16_t)0b0'000'1'0'0'0110'0'00'11
#define W_CTRL_EN_16u       (uint16_t)0b0'000'1'0'0'0100'0'00'11
#define W_TORQUE_DEF        (uint16_t)0b0'001'11111111'001'0
#define W_OFF_DEF           (uint16_t)0b0'010'00110000'0'000
#define W_BLANK_DEF         (uint16_t)0b0'011'10000000'0'000
#define W_DECAY_DEF         (uint16_t)0b0'100'00010000'000'0
#define W_STALL_DEF         (uint16_t)0b0'101'01000000'00'00
#define W_DRIVE_DEF         (uint16_t)0b0'110'01'10'01'01'10'10
#define R_STATUS            (uint16_t)0b1'111

using namespace PIN_BOARD;

namespace DRV8711{
    enum class MODE: uint16_t{
        OneTo256 = W_CTRL_EN_256u,
        OneTo128 = W_CTRL_EN_128u,
        OneTo64 = W_CTRL_EN_64u,
        OneTo16 = W_CTRL_EN_16u
    };

    class DRV8711{
    public:
        explicit DRV8711(SPI_HandleTypeDef* spi, PIN<PinWriteable>&& cs_pin)
            :spi_(spi),
             cs_pin_(cs_pin)
        {
            ConfigureDriver();
        }

        bool IsDeviceOk(){
            uint16_t result{};
            ReadFromDriver(R_STATUS, result);
            return !result;
        }

        bool ConfigureDriver(){
            SendToDriver(W_CTRL_EN_256u);
        }

        void SetMStep(MODE mStep){
            SendToDriver(static_cast<uint16_t>(mStep));
        }
    private:
        void SendToDriver(uint16_t reg){
            cs_pin_.setValue(HIGH);
            std::memcpy(&data_, &reg, sizeof(reg));
            HAL_SPI_Transmit (spi_, data_, sizeof(data_), 5000);
            cs_pin_.setValue(LOW);
        }

        void ReadFromDriver(uint16_t reg, uint16_t& storage){
            uint8_t retVal[2] {0,};
            cs_pin_.setValue(HIGH);
            std::memcpy(&data_, &reg, sizeof(reg));
            HAL_SPI_TransmitReceive(spi_, data_, retVal, sizeof(data_), 5000);
            std::memcpy(&storage, &retVal, sizeof(retVal));
            cs_pin_.setValue(LOW);
        }

        uint8_t data_ [2] {0,};
        PIN<PinWriteable> cs_pin_;
        SPI_HandleTypeDef* spi_;
    };
}

