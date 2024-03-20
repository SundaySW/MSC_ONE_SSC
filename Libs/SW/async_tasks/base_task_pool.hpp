#pragma once

//#include "function2/function2.hpp"
//
//#define call_back_capacity (32U)

namespace async_tim_task{

using CallBackT = std::function<void()>;
//using CallBackT = fu2::function_base<true, false, fu2::capacity_fixed<call_back_capacity>,
//            false, false, void()>;
using DelayT = uint32_t;

struct BaseTaskPool{
    BaseTaskPool(const BaseTaskPool&) = delete;
    BaseTaskPool(BaseTaskPool&&) = delete;
    BaseTaskPool& operator=(const BaseTaskPool&) = delete;
    BaseTaskPool& operator=(BaseTaskPool&&) = delete;

    explicit BaseTaskPool() = default;
    virtual ~BaseTaskPool() = default;

    virtual void OnTimTick() = 0;
    virtual int PlaceToPool(CallBackT&&, DelayT) = 0;
    virtual void Poll() = 0;
    virtual bool RemoveFromPool(unsigned short idx) = 0;
    virtual bool StopTim(unsigned short idx) = 0;
};

}// namespace async_tim_task