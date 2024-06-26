#pragma once

#include "stm32g4xx_hal.h"
#include "tim.h"

#include <cstring>
#include <functional>
#include <optional>
#include <array>

#include "protos_core/protos_msg.h"
#include "protos_core/protos_device.h"
#include "protos_core/base_param.h"

#include "protos_can_device/base_device.hpp"

#include "sa_stm32_g4/i2c.hpp"
#include "sa_stm32_g4/adc.hpp"
#include "sa_stm32_g4/tim_ICmode.hpp"
#include "sa_stm32_g4/dac.hpp"
#include "sa_stm32_g4/eeprom.hpp"
#include "sa_stm32_g4/eeprom_24aa02uid.hpp"

#include "1Wire/onewire_device_pool.h"
#include "1Wire/onewire_ds2482.h"
#include "1Wire/onewire_task_provider.h"

#include "OneWire/one_wire.hpp"
#include "ssc_port/ssc_port.hpp"
#include "ssc_port/spi_adc.hpp"
#include "ssc_port/ssc_port_param.hpp"

#include "async_tim_tasks/async_tim_tasks.hpp"

#define EEPROM_I2C_ADDR 0x50
#define DS2482_I2C_ADDR 0x18

using namespace Protos;

//#define CastArg(void_ptr, type)  std::launder(static_cast<std::optional<type>*>(void_ptr))
#define CastArg(void_ptr, type)  std::launder(static_cast<decltype(type)::ret_val_t *>(void_ptr))

class MscOne : public BaseDevice
{
public:
    using ADCc3 = Adc<3>;
    using ADCc2 = Adc<2>;
    MscOne() = delete;
    MscOne(MscOne&) = delete;
    MscOne(MscOne&&) = delete;
    MscOne& operator = (MscOne &) = delete;
    MscOne& operator = (MscOne const &) = delete;

    static MscOne& global(){
        static auto self = MscOne(DeviceUID::TYPE_MICROCHIP, 0x01, 0x20, &hfdcan1);
        return self;
    }

    void MicroTimHandler(){
        ssc_port_1.GetOWPort().TimItHandler();
        ssc_port_2.GetOWPort().TimItHandler();
    }

    void PollCoroPort(){
//        if(oneWirePort1.IsNewConnected()){
//            CoroTaskRead();
//        }
        ssc_port_1.GetOWPort().Poll();
        ssc_port_2.GetOWPort().Poll();
    }

    void initPerf(ADC_HandleTypeDef *adc1,
                  ADC_HandleTypeDef *adc2,
                  I2C_HandleTypeDef *i2c2,
                  DAC_HandleTypeDef *dac1,
                  TIM_HandleTypeDef* timHall0,
                  TIM_HandleTypeDef* timHall1,
                  TIM_HandleTypeDef* ssc1_ow_tim,
                  SPI_HandleTypeDef* ssc1_spi)
    {
        I2CMaster = I2C(i2c2);
        AdcA1 = std::move(ADCc3(adc1));
        AdcA2 = std::move(ADCc2(adc2));
        TimIC0 = std::move(Tim_ICMode(timHall0, TIM_CHANNEL_1));
        TimIC1 = std::move(Tim_ICMode(timHall1, TIM_CHANNEL_1));
        Valve0Ctrl = std::move(DacParam(dac1, DAC_CHANNEL_1));
        Valve1Ctrl = std::move(DacParam(dac1, DAC_CHANNEL_2));
        ssc_port_1.Init(ssc1_spi, ssc1_ow_tim);
        sscPortParam1.SetADC(ssc_port_1.GetADC());
        ssc_port_2.Init(ssc1_spi, ssc1_ow_tim);
        sscPortParam2.SetADC(ssc_port_2.GetADC());

        Valve0Ctrl.SetId(0xC1);
        Valve0Ctrl.SetCtrlRate(500);
        Valve0Ctrl.SetSendRate(0);
        Valve0Ctrl.SetShort(0);

        Valve1Ctrl.SetId(0xC2);
        Valve1Ctrl.SetCtrlRate(500);
        Valve1Ctrl.SetSendRate(0);
        Valve1Ctrl.SetShort(0);

        Preasure1.SetId(0x31);
        Preasure1.SetUpdateRate(1000);
        Preasure1.SetSendRate(2000);

        Preasure2.SetId(0x32);
        Preasure2.SetUpdateRate(1000);
        Preasure2.SetSendRate(2000);

        Preasure3.SetId(0x33);
        Preasure3.SetUpdateRate(1000);
        Preasure3.SetSendRate(2000);

        Preasure4.SetId(0x34);
        Preasure4.SetUpdateRate(1000);
        Preasure4.SetSendRate(2000);

        Preasure5.SetId(0x35);
        Preasure5.SetUpdateRate(1000);
        Preasure5.SetSendRate(2000);

//      using ReturnType = std::invoke_result_t<decltype(makeValueFlowMeter)>;
        hallSensor0.SetId(0x41);
        hallSensor0.SetUpdateRate(1000);
        hallSensor0.SetSendRate(2000);

        hallSensor1.SetId(0x42);
        hallSensor1.SetUpdateRate(1000);
        hallSensor1.SetSendRate(2000);

        sscPortParam1.SetId(0xB1);
        sscPortParam2.SetId(0xB2);
    }

	void Start()
	{
		readUID();
        loadCalibParamsDataFromEEPROM();
		AdcA1.Start();
        AdcA2.Start();
		TimIC0.Start();
        TimIC1.Start();
		Valve0Ctrl.Start();
		Valve1Ctrl.Start();
        ssc_port_1.Start();
        ssc_port_2.Start();
        PLACE_ASYNC_TASK([](void*){ HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);}, 1);
//        PLACE_ASYNC_TASK([&]{
//            OnTimerINT(1);
//        }, 1'000);
//        OWDevices.OnSearch(0, OneWire::DEVICE_FAMILY::FAMILY_UNKNOWN);
    }

    static void saveCalibParamToEEPROM(char ID, float* data){
        for(std::size_t i = 0; i < Params.size(); i++){
            if(Params[i] == nullptr) continue;
            if(Params[i]->GetId() == ID){
                char buffer[EEPROM_CALIB_DATA_SIZE];
                int Offset = EEPROM_PARAMS_START_ADDR + EEPROM_CALIB_DATA_SIZE * i;
                memcpy(buffer, data, EEPROM_CALIB_DATA_SIZE);
                eeprom_write_block(Offset, buffer, EEPROM_CALIB_DATA_SIZE);
            }
        }
    }

    static void loadCalibParamsDataFromEEPROM(){
        for(std::size_t i = 0; i < Params.size(); i++){
            if(Params[i] == nullptr) continue;
            CalibrParam* calibPtr = nullptr;
            Params[i]->QueryInterface(IID_CALIBRATEABLE, (void*&)calibPtr);
            if(calibPtr == nullptr) continue;
            char buffer[EEPROM_CALIB_DATA_SIZE];
            int Offset = EEPROM_PARAMS_START_ADDR + EEPROM_CALIB_DATA_SIZE * i;
            eeprom_read_block(Offset, buffer, EEPROM_CALIB_DATA_SIZE);
            float calibData[PARAM_NOFCALIB_FIELDS];
            memcpy(calibData, buffer, EEPROM_CALIB_DATA_SIZE);
            calibPtr->SetOffset(calibData[0]);
            calibPtr->SetMult(calibData[1]);
        }
    }

    void readUID(){
        uint8_t r_data[6] = {0};
        eeprom.readUID(r_data);
        memcpy(&Uid.Data.I4, r_data, sizeof(uint32_t));
    }

    void ProcessMessage(const Protos::Msg& msg) override{
        Protos::Msg2 msg2;
        msg2.Pri = msg.Id.Tab[0];
        msg2.Src = msg.GetSenderID();
        msg2.Dst = msg.GetDestID();
        msg2.Type = msg.GetMSGType();
        msg2.Dlc = msg.Dlc;
        for (int i = 0; i < msg.Dlc; i++)
            msg2.Data[i] = msg.Data[i];
//        if (msg2.Dst == Address)
//            OWDevices.ProcessMessage(msg2);

        switch (msg.GetMSGType()) {
            case MSGTYPE_PARAM_SET:
            case MSGTYPE_PARAM_REQUEST:
                for (auto param : Params)
                    if(param != nullptr) param->ProcessMessage(msg);
                break;
            case MSGTYPE_PARAM_ANSWER:
            case MSGTYPE_CMDSPEC:
            case MSGTYPE_CMDSPEC_ANSWER:
            default:
                break;
        }
    };
//    char buff[144];
//    char* buff_ptr = buff;
//    void ReadFullMem(){
//        oneWirePort1.PlaceTask(OneW_Coro::read_memory_full, buff_ptr, [&](void* ret_val_ptr){
//            for(std::size_t i = 0; i < 144; i+=8){
//                SendProtosMsg(0xFF, MSGTYPE_CMDMISC, &buff[i], 8);
//            }
//        });
//    }
//
//    void CoroTaskWrite(){
//        decltype(OneWirePort::write_scratchpad_coro_)::arg_t arg = {0x8, 0, {5,5,5,5,5,5,5,5}};
//        oneWirePort1.PlaceTask(OneW_Coro::write_memory, arg, [&](void* ret_val_ptr){
//            uint8_t r =0;
//        });
//        decltype(OneWirePort::write_scratchpad_coro_)::arg_t arg2 = {0x10, 0x8, {4,4,4,4,4,4,4,4}};
//        oneWirePort1.PlaceTask(OneW_Coro::write_memory, arg2, [&](void* ret_val_ptr){
//           uint8_t r = 0;
//        });
//    }
//
//    void CoroTaskRead(){
//        decltype(OneWirePort::read_memory_coro_)::arg_t arg = {0x8, 0};
//        ssc_port_.GetOWPort().PlaceTask(OneW_Coro::read_memory, arg, [&](void* ret_val_ptr){
//            auto* ret_val = CastArg(ret_val_ptr, OneWirePort::read_memory_coro_);
//            if(ret_val->has_value()){
//                auto& v = ret_val->value();
//                SendProtosMsg(0xFF, MSGTYPE_CMDMISC, &v[0], v.size());
//            }
//        });
//        decltype(OneWirePort::read_memory_coro_)::arg_t arg2 = {0x10, 0x8};
//        ssc_port_.GetOWPort().PlaceTask(OneW_Coro::read_memory, arg2, [&](void* ret_val_ptr){
//            auto* ret_val = CastArg(ret_val_ptr, OneWirePort::read_memory_coro_);
//            if(ret_val->has_value()){
//                auto& v = ret_val->value();
//                SendProtosMsg(0xFF, MSGTYPE_CMDMISC, &v[0], v.size());
//            }
//        });
//    }

	void OnPoll() override {
        for (auto* param : Params)
            if(param != nullptr) param->Poll();
	};

	void OnTimer(short ms) override{
        for (auto param : Params)
            if(param != nullptr) param->OnTimer(ms);
	};

    I2C& getI2CMaster(){
        return I2CMaster;
    }

    static void processADCCallback(ADC_HandleTypeDef* hadc){
        if (hadc == AdcA1.getHandler())
            AdcA1.OnCallback();
        else if(hadc == AdcA2.getHandler())
            AdcA2.OnCallback();
    }

    static void processTimCallBack(TIM_HandleTypeDef* htim){
        if(htim == TimIC0.getHTim())
            TimIC0.OnCallBack();
    }

private:
    MscOne(DeviceUID::TYPE uidType, uint8_t family, uint8_t addr, FDCAN_HandleTypeDef* can)
            : BaseDevice(uidType, family, addr, can)
            , eeprom(&I2CMaster, EEPROM_I2C_ADDR)
    {}

    SSCPort ssc_port_1{pin_board::PIN<pin_board::Writeable>(SS0_GPIO_Port, SS0_Pin),
                      pin_board::PIN<pin_board::Switchable>(ID0_GPIO_Port, ID0_Pin),
                      pin_board::PIN<pin_board::Switchable>(GPIOB, GPIO_PIN_4),
                      sscPortParam1};
    SSCPort ssc_port_2{pin_board::PIN<pin_board::Writeable>(SS1_GPIO_Port, SS1_Pin),
                      pin_board::PIN<pin_board::Switchable>(ID1_GPIO_Port, ID1_Pin),
                       pin_board::PIN<pin_board::Switchable>(GPIOB, GPIO_PIN_4),
                      sscPortParam2};
    I2C I2CMaster;
    inline static ADCc3 AdcA1;
    inline static ADCc2 AdcA2;
    inline static Tim_ICMode TimIC0;
    inline static Tim_ICMode TimIC1;
    inline static AdcParam Preasure1{&AdcA1, 0, &saveCalibParamToEEPROM};
    inline static AdcParam Preasure2{&AdcA1, 1, &saveCalibParamToEEPROM};
    inline static AdcParam Preasure3{&AdcA1, 2, &saveCalibParamToEEPROM};
    inline static AdcParam Preasure4{&AdcA2, 0, &saveCalibParamToEEPROM};
    inline static AdcParam Preasure5{&AdcA2, 1, &saveCalibParamToEEPROM};
    inline static Tim_ICmParam hallSensor0{&TimIC0, &saveCalibParamToEEPROM};
    inline static Tim_ICmParam hallSensor1{&TimIC1, &saveCalibParamToEEPROM};
    inline static DacParam Valve0Ctrl;
    inline static DacParam Valve1Ctrl;
    inline static SSCPortParam sscPortParam1{&saveCalibParamToEEPROM};
    inline static SSCPortParam sscPortParam2{&saveCalibParamToEEPROM};
    inline static constexpr int PARAM_CNT = 11;
    inline static constexpr auto Params{[]() constexpr{
        std::array<BaseParam*, PARAM_CNT> result{};
        int pCount = PARAM_CNT-1;
        result[pCount--] = static_cast<BaseParam*>(&Valve0Ctrl);
        result[pCount--] = static_cast<BaseParam*>(&Valve1Ctrl);
        result[pCount--] = static_cast<BaseParam*>(&Preasure1);
        result[pCount--] = static_cast<BaseParam*>(&Preasure2);
        result[pCount--] = static_cast<BaseParam*>(&Preasure3);
        result[pCount--] = static_cast<BaseParam*>(&Preasure4);
        result[pCount--] = static_cast<BaseParam*>(&Preasure5);
        result[pCount--] = static_cast<BaseParam*>(&hallSensor0);
        result[pCount--] = static_cast<BaseParam*>(&hallSensor1);
        result[pCount--] = static_cast<BaseParam*>(&sscPortParam1);
        result[pCount]   = static_cast<BaseParam*>(&sscPortParam2);
        return result;
    }()};
    Eeprom24AAUID eeprom;
};

void SendMsg(char dest, char msgType, const char *data, char dlc)
{
    MscOne::global().SendProtosMsg(dest, msgType, data, dlc);
}
void SendMsg(char dest, char msgType, char data0, char data1)
{
    char buf[8];
    buf[0] = data0;
    buf[1] = data1;
    MscOne::global().SendProtosMsg(dest, msgType, buf, 2);
}