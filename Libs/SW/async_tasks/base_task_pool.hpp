#pragma once

namespace async_tim_task{

using CallBackT = std::function<void()>;
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