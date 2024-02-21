#pragma once

#include <coroutine>
#include <array>
#include "queue"
#include "variant"
#include "functional"

#include "stm32g4xx.h"
#include "stm32g4xx_hal_tim.h"

#include "HW/IO/pin.hpp"

#include "Coro/Event.hpp"
#include "Coro/Future.hpp"
#include "CoroTaskQueue.hpp"

#include "main.h"

namespace OneW::CMD {
    enum CMD
    {
        // ROM commands, basic for all devices
        MATCH_ROM					= 0x55,
        READ_ROM					= 0x33,
        SKIP_ROM					= 0xCC,
        SEARCH_ROM					= 0xF0,
        OVERDRIVE_MATCH_ROM			= 0x69,
        // custom commands
        DS18B20_CONVERT_T			= 0x44,
        WRITE_SCRATCHPAD			= 0x4E,
        COPY_SCRATCHPAD				= 0x48,
        DS18B20_READ_POWER_SUPPLY	= 0xB4,
        READ_SCRATCHPAD				= 0xBE,
        RECALL_MEMORY				= 0xB8,
        DS2413_PIO_ACCESS_WRITE		= 0x5A,
        DS2413_PIO_ACCESS_READ		= 0xF5
    };
}

namespace OneW_Coro{
    enum Coro_task_t{
        no_type,
        read_scratchpad,
        write_scratchpad,
        blink_led,
        multiple_array,
        search_coro
    };

    struct CoroTask{
        CoroTask() = default;
        explicit CoroTask(Coro_task_t t)
                :type(t)
        {};
        template<typename T>
        T* GetStoredArg(){
            return std::launder( static_cast<T*>( static_cast<void*>(arg_storage_) ) );
        }
        void* GetArgStoragePtr(){
            return static_cast<void*>(arg_storage_);
        }
        template<typename T>
        void StoreArgument(T arg){
            new(arg_storage_)(T)(arg);
        }
        template<typename T>
        T* GetRetValRef(){
            return std::launder( static_cast<T*>( static_cast<void*>(ret_storage_) ) );
        }
        template<typename T>
        void StoreRetVal(T arg){
            new(ret_storage_)(T)(arg);
        }
        void CallAwaiter(){
            finished = true;
            call_back(static_cast<void*>(ret_storage_));
        }
        void* coro_ptr;
        Coro_task_t type;
        std::function<void(void *)> call_back;
    private:
        bool finished{false};
        std::byte arg_storage_[40];
        std::byte ret_storage_[40];
    };
}

class OneWirePort{
#define tSlot                   65
#define tW1l                    10
#define tW0l                    55
#define tRL                     2
#define tMSR                    7
#define tRSTL                   480
#define tRSTH                   tRSTL
#define tPDH                    17
#define tPDL                    65
#define tMSP                    65

#define SWITCH_CONTEXT          co_await Delay(1);

#define RESET_OW_LINE           reset_coro_.Resume(); \
                                co_await reset_coro_; \
                                auto static line_reset_ok = !reset_coro_.GetRetVal().value(); \
                                SWITCH_CONTEXT

#define LINE_RESETED_OK         line_reset_ok

#define SEND_OW_CMD(cmd)        write_byte_coro_.Resume(cmd); \
                                co_await write_byte_coro_;    \
                                SWITCH_CONTEXT

#define SEND_DATA_BYTE(byte)    write_byte_coro_.Resume(byte); \
                                co_await write_byte_coro_;     \
                                SWITCH_CONTEXT

#define READ_DATA_BYTE          read_byte_coro_.Resume(); \
                                co_await read_byte_coro_; \
                                SWITCH_CONTEXT

#define OW_READ_BIT             read_bit_coro_.Resume(); \
                                co_await read_bit_coro_; \
                                SWITCH_CONTEXT

#define OW_WRITE_BIT(bit)       write_bit_coro_.Resume(bit); \
                                co_await write_bit_coro_;     \
                                SWITCH_CONTEXT

public:
    explicit OneWirePort(PIN_BOARD::PIN<PIN_BOARD::PinSwitchable> pin, TIM_HandleTypeDef* htim = nullptr)
            : pin_(pin)
            , htim_(htim)
    {}
    void SetPinLow(){
        pin_.setAsOutput();
        pin_.setValue(PIN_BOARD::LOW);
    }
    void TimItHandler(){
        if(in_process){
//            HAL_TIM_Base_Stop_IT(htim_);
            htim_->State = HAL_TIM_STATE_READY;
            timer_ev_.Notify();
        }
    }

    void SetTim(TIM_HandleTypeDef* tim){
        htim_ = tim;
    }
    
    auto Poll(){
        if(!in_process)
            RunCoro();
    }

    template<typename ArgT>
    void PlaceTask(OneW_Coro::Coro_task_t type, ArgT&& arg, std::function<void(void *)> call_back){
        auto coro = OneW_Coro::CoroTask(type);
        coro.call_back = std::move(call_back);

        if(type == OneW_Coro::write_scratchpad){
//            static_assert(std::is_same <decltype(write_scratchpad_coro_.GetArgumentValue()), ArgT>().value);
            coro.coro_ptr = static_cast<void*>(&write_scratchpad_coro_);
        }
        else if(type == OneW_Coro::read_scratchpad)
            coro.coro_ptr = static_cast<void*>(&read_scratchpad_coro_);

        else if(type == OneW_Coro::blink_led)
            coro.coro_ptr = static_cast<void*>(&blink_led_coro_);

        else if(type == OneW_Coro::search_coro){
            coro.coro_ptr = static_cast<void*>(&search_coro_);

        }

        coro.StoreArgument(std::forward<ArgT>(arg));
        coro_to_run.push(coro);
        RunCoro();
    }

    void RunCoro(){
        if(!in_process && !coro_to_run.empty()){
            in_process = true;
            running_task = coro_to_run.front();
            coro_to_run.pop();

            if(running_task.type == OneW_Coro::write_scratchpad){
                auto& arg = *running_task.GetStoredArg<decltype(write_scratchpad_coro_.GetArgumentValue())>();
                static_cast<decltype(write_scratchpad_coro_)*>(running_task.coro_ptr)->Resume(arg);
            }

            else if(running_task.type == OneW_Coro::read_scratchpad){
                auto& arg = *running_task.GetStoredArg<decltype(read_scratchpad_coro_.GetArgumentValue())>();
                static_cast<decltype(read_scratchpad_coro_)*>(running_task.coro_ptr)->Resume(arg);
            }

            else if(running_task.type == OneW_Coro::blink_led){
                auto& arg = *running_task.GetStoredArg<decltype(blink_led_coro_.GetArgumentValue())>();
                static_cast<decltype(blink_led_coro_)*>(running_task.coro_ptr)->Resume(arg);
            }

            else if(running_task.type == OneW_Coro::search_coro){
                auto& arg = *running_task.GetStoredArg<decltype(search_coro_.GetArgumentValue())>();
                static_cast<decltype(search_coro_)*>(running_task.coro_ptr)->Resume(arg);
            }
        }
    }

    void FinishCoro(auto& coro_ref){
        running_task.StoreRetVal(coro_ref.GetRetVal());
        running_task.CallAwaiter();
        in_process = false;
    }

    void TargetSetup(uint8_t family_code) {
        uint8_t i;
        // set the search state to find SearchFamily type devices
        ROM_[0] = family_code;
        for (i = 1; i < 8; i++)
            ROM_[i] = 0;
        last_discrepancy_ = 64;
        last_family_discrepancy_ = 0;
        last_device_flag_ = 0;
    }

    uint8_t GetROM(uint8_t index) {
        return ROM_[index];
    }

private:
    using Future_uint = Future<char>;

    Future_uint write_bit_coro_ = WriteBit();
    Future_uint read_bit_coro_ = ReadBit();
    Future_uint write_byte_coro_ = WriteByte();
    Future_uint read_byte_coro_ = ReadByte();
    Future_uint reset_coro_ = Reset();
    Future_uint verify_coro = Verify();

    bool in_process = false;
    using CoroTask = OneW_Coro::CoroTask;
    std::queue<CoroTask> coro_to_run;
    CoroTask running_task {};

    PIN_BOARD::PIN<PIN_BOARD::PinSwitchable> pin_;
    Event timer_ev_;
    TIM_HandleTypeDef* htim_;

    uint8_t last_discrepancy_{};
    uint8_t last_family_discrepancy_{};
    uint8_t last_device_flag_{};
    uint8_t ROM_[8]{};

//    using CoroTypes = std::variant<
//            decltype(blink_led_coro_),
//            decltype(write_scratchpad_coro_),
//            decltype(multiple_array_coro_),
//            decltype(read_scratchpad_coro_)
//    >;

    Future<uint8_t, std::array<char, 8>> write_scratchpad_coro_ = WriteScratchpad(write_scratchpad_coro_);
    decltype(write_scratchpad_coro_) WriteScratchpad(decltype(write_scratchpad_coro_)& this_coro_){
        while (true){
            auto arg = this_coro_.GetArgumentValue();
            RESET_OW_LINE
            if(LINE_RESETED_OK){
                SEND_OW_CMD(OneW::CMD::SKIP_ROM);
                SEND_OW_CMD(OneW::CMD::WRITE_SCRATCHPAD);
                for(char& ch: arg)
                    SEND_DATA_BYTE(ch);
            }
            FinishCoro(this_coro_);
            co_yield {};
        }
    }
    Future<std::array<char, 8>, uint8_t> read_scratchpad_coro_ = ReadScratchpad(read_scratchpad_coro_);
    decltype(read_scratchpad_coro_) ReadScratchpad(decltype(read_scratchpad_coro_)& this_coro_){
        decltype(read_scratchpad_coro_)::value_type buff{};
        while (true){
            RESET_OW_LINE
            if(LINE_RESETED_OK){
//                SEND_OW_CMD(OneW::CMD::SKIP_ROM);
//                SEND_OW_CMD(OneW::CMD::READ_SCRATCHPAD);
                SEND_OW_CMD(0x33);
                for(char& i : buff){
                    READ_DATA_BYTE;
                    i = read_byte_coro_.GetRetVal().value();
                }
                this_coro_.StoreValueAndNotify(buff);
            }
            FinishCoro(this_coro_);
            co_yield {};
        }
    }

    PIN_BOARD::PIN<PIN_BOARD::PinWriteable> led_pin_ = PIN_BOARD::PIN<PIN_BOARD::PinWriteable>(LED1_GPIO_Port, LED1_Pin);
    Future<bool> blink_led_coro_ = BlinkLed(blink_led_coro_);
    decltype(blink_led_coro_) BlinkLed(decltype(blink_led_coro_)& this_coro_){
        while (true){
            auto arg = this_coro_.GetArgumentValue();
//            uint16_t delay = arg ? 2500 : 500;
            uint16_t delay = 0xFFFF;
            led_pin_.setValue(PIN_BOARD::LOW);
            co_await Delay(delay);
            led_pin_.setValue(PIN_BOARD::HIGH);
            co_await Delay(delay);
            led_pin_.setValue(PIN_BOARD::LOW);
            co_await Delay(delay);
            led_pin_.setValue(PIN_BOARD::HIGH);
            this_coro_.StoreRetVal(arg);
            FinishCoro(this_coro_);
            co_yield {};
        }
    }

private:
    decltype(write_byte_coro_) WriteByte(){
        while (true){
            uint8_t i = 8;
            if(write_byte_coro_.HasStoredArgument()){
                uint8_t byte = write_byte_coro_.GetArgumentValue();
                while (i--){
                    /* LSB bit is first */
                    write_bit_coro_.Resume(byte & 0x01);
                    co_await write_bit_coro_;
                    byte >>= 1;
                    SWITCH_CONTEXT
                }
            }
            write_byte_coro_.NotifyAwaiter();
            co_yield 0;
        }
    }


    decltype(read_byte_coro_) ReadByte(){
        while (true){
            uint8_t i = 8, byte = 0;
            while (i--) {
                byte >>= 1;
                read_bit_coro_.Resume();
                co_await read_bit_coro_;
                byte |= (read_bit_coro_.GetRetVal().value() << 7);
                SWITCH_CONTEXT
            }
            read_byte_coro_.StoreValueAndNotify(byte);
            co_yield byte;
        }
    }

    Future_uint search_coro_ = Search(search_coro_);
    decltype(search_coro_) Search(decltype(search_coro_)& this_coro_) {
        while(true) {
            uint8_t id_bit_number = 1;
            uint8_t last_zero = 0, rom_byte_number = 0, search_result = 0;
            uint8_t id_bit, cmp_id_bit;
            uint8_t rom_byte_mask = 1, search_direction;

            RESET_OW_LINE
            if (LINE_RESETED_OK) {
                SEND_OW_CMD(this_coro_.GetArgumentValue())
                do {
                    // read a bit and its complement
                    OW_READ_BIT
                    id_bit = read_bit_coro_.GetRetVal().value();;
                    OW_READ_BIT
                    cmp_id_bit = read_bit_coro_.GetRetVal().value();;
                    // check for no devices on 1-wire
                    if ((id_bit == 1) && (cmp_id_bit == 1)) {
                        break;
                    } else {
                        // all devices coupled have 0 or 1
                        if (id_bit != cmp_id_bit) {
                            search_direction = id_bit;  // bit write value for search
                        } else {
                            // if this discrepancy is before the Last Discrepancy
                            // on a previous next then pick the same as last time
                            if (id_bit_number < last_discrepancy_) {
                                search_direction = ((ROM_[rom_byte_number] & rom_byte_mask) > 0);
                            } else {
                                // if equal to last pick 1, if not then pick 0
                                search_direction = (id_bit_number == last_discrepancy_);
                            }
                            // if 0 was picked then record its position in LastZero
                            if (search_direction == 0) {
                                last_zero = id_bit_number;
                                // check for Last discrepancy in family
                                if (last_zero < 9) {
                                    last_family_discrepancy_ = last_zero;
                                }
                            }
                        }
                        // set or clear the bit in the ROM_ byte rom_byte_number
                        // with mask rom_byte_mask
                        if (search_direction == 1)
                            ROM_[rom_byte_number] |= rom_byte_mask;
                        else
                            ROM_[rom_byte_number] &= ~rom_byte_mask;

                        // serial number search direction write bit
                        OW_WRITE_BIT(search_direction)

                        // increment the byte counter id_bit_number
                        // and shift the mask rom_byte_mask
                        id_bit_number++;
                        rom_byte_mask <<= 1;
                        // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                        if (rom_byte_mask == 0) {
                            //docrc8(ROM_NO[rom_byte_number]);  // accumulate the CRC
                            rom_byte_number++;
                            rom_byte_mask = 1;
                        }
                    }
                } while (rom_byte_number < 8);  // loop until through all ROM_ bytes 0-7

                // if the search was successful then
                if (id_bit_number >= 65) {
                    // search successful so set last_discrepancy_,last_device_flag_,search_result
                    last_discrepancy_ = last_zero;
                    // check for last device
                    if (last_discrepancy_ == 0)
                        last_device_flag_ = 1;
                    search_result = 1;
                }
            }
            // if no device found then reset counters so next 'search' will be like a first
            if (!search_result || !ROM_[0]) {
                last_discrepancy_ = 0;
                last_device_flag_ = 0;
                last_family_discrepancy_ = 0;
                search_result = 0;
            }
            this_coro_.StoreValueAndNotify(search_result);
            FinishCoro(this_coro_);
            co_yield search_result;
        }
    }

    decltype(verify_coro) Verify() {
        unsigned char rom_backup[8];
        int i, result, ld_backup, ldf_backup, lfd_backup;
        // keep a backup copy of the current state
        for (i = 0; i < 8; i++)
            rom_backup[i] = ROM_[i];
        ld_backup = last_discrepancy_;
        ldf_backup = last_device_flag_;
        lfd_backup = last_family_discrepancy_;
        // set search to find the same device
        last_discrepancy_ = 64;
        last_device_flag_ = 0;

        search_coro_.Resume(OneW::CMD::SEARCH_ROM);
        co_await search_coro_;
        auto search_result = search_coro_.GetRetVal();
        if(search_result){
            // check if same device found
            result = 1;
            for (i = 0; i < 8; i++) {
                if (rom_backup[i] != ROM_[i]) {
                    result = 1;
                    break;
                }
            }
        }else
            result = 0;
        // restore the search state
        for (i = 0; i < 8; i++)
            ROM_[i] = rom_backup[i];
        last_discrepancy_ = ld_backup;
        last_device_flag_ = ldf_backup;
        last_family_discrepancy_ = lfd_backup;
        // return the result of verify
        verify_coro.StoreValueAndNotify(result);
        co_yield result;
    }

//    Future<bool, std::array<uint8_t,8>> select_coro_ = Select();
//    decltype(select_coro_) Select() {
//        uint8_t i;
//        write_byte_coro_.Resume(OneW::CMD::MATCH_ROM);
//        co_await write_byte_coro_;
//        for (i = 0; i < 8; i++){
//            write_byte_coro_.Resume(*(addr + i));
//            co_await write_byte_coro_;
//        }
//    }

    void FamilySkipSetup(){
        // set the Last discrepancy to last family discrepancy
        last_discrepancy_ = last_family_discrepancy_;
        last_family_discrepancy_ = 0;
        // check for end of list
        if (last_discrepancy_ == 0)
            last_device_flag_ = 1;
    }

    Event& Delay(uint32_t micros)
    {
        if(htim_){
            __HAL_TIM_SET_AUTORELOAD(htim_, micros);
//            __HAL_TIM_SET_AUTORELOAD(htim_, __HAL_TIM_CALC_PERIOD(160000000, 0xFFFF, micros));
            HAL_TIM_Base_Start_IT(htim_);
        }
        return timer_ev_;
    }

    decltype(read_bit_coro_) ReadBit() {
        while (true){
            uint8_t bit = 0;
            pin_.setAsOutput();
            pin_.setValue(PIN_BOARD::LOW);
            co_await Delay(tRL);
            pin_.setAsInput();
            co_await Delay(tMSR);
            if(pin_.getState())
                bit = 1;
            co_await Delay(tSlot - tMSR - tRL);
            read_bit_coro_.StoreValueAndNotify(bit);
            co_yield 0;
        }
    }

    decltype(write_bit_coro_) WriteBit(){
        while (true){
            auto value_to_set = write_bit_coro_.GetArgumentValue();
            if (value_to_set) {
                pin_.setAsOutput();
                pin_.setValue(PIN_BOARD::LOW);
                co_await Delay(tW1l);
                pin_.setAsInput();
                co_await Delay(tSlot - tW1l);
                write_bit_coro_.NotifyAwaiter();
            } else {
                pin_.setAsOutput();
                pin_.setValue(PIN_BOARD::LOW);
                co_await Delay(tW0l);
                pin_.setAsInput();
                co_await Delay(tSlot - tW0l);
                write_bit_coro_.NotifyAwaiter();
            }
            co_yield value_to_set;
        }
    }

    decltype(reset_coro_) Reset(){
        while (true){
            uint8_t i;
            pin_.setAsOutput();
            pin_.setValue(PIN_BOARD::LOW);
            co_await Delay(tRSTL);
            pin_.setAsInput();
            co_await Delay(tMSP);
            i = pin_.getState();
            co_await Delay(tRSTH - tMSP);
            reset_coro_.StoreValueAndNotify(i);
            co_yield i;
        }
    }

    decltype(search_coro_)SelectROM(const uint8_t *ROM){
        uint8_t i;
        write_byte_coro_.Resume(OneW::CMD::MATCH_ROM);
        co_await write_byte_coro_;
        for (i = 0; i < 8; i++){
            write_byte_coro_.Resume(*(ROM + i));
            co_await write_byte_coro_;
        }
    }

    void GetFullROM(uint8_t *firstIndex){
        uint8_t i;
        for (i = 0; i < 8; i++)
            *(firstIndex + i) = ROM_[i];
    }

    static uint8_t CRC8(uint8_t *addr, uint8_t len){
        uint8_t crc = 0, inbyte, i, mix;
        while (len--) {
            inbyte = *addr++;
            for (i = 8; i; i--) {
                mix = (crc ^ inbyte) & 0x01;
                crc >>= 1;
                if (mix) {
                    crc ^= 0x8C;
                }
                inbyte >>= 1;
            }
        }
        return crc;
    }

//    Future_uint FirstDevice(){
//        while (true){
//            /* Reset search values */
//            ResetSearch();
//            search_coro_.Resume(OneW::CMD::SEARCH_ROM);
//            co_await search_coro_;
//            auto result = search_coro_.GetArgumentValue();
//            co_yield result;
//        }
//    }
//
//    Future_uint NextDevice(){
//        while (true){
//            search_coro_.Resume(OneW::CMD::SEARCH_ROM);
//            co_await search_coro_;
//            auto result = search_coro_.GetArgumentValue();
//            co_yield result;
//        }
//    }

    void ResetSearch(){
        last_discrepancy_ = 0;
        last_device_flag_ = 0;
        last_family_discrepancy_ = 0;
    }
};