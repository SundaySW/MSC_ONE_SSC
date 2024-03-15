#pragma once

#include <optional>

#include "async_task.hpp"
#include "base_task_pool.hpp"
#include "fwd.hpp"

namespace async_tim_task{

namespace {
    const std::size_t pool_size = 20;
}

struct TaskPool final: BaseTaskPool{
    ~TaskPool() override = default;
    TaskPool(TaskPool&) = delete;
    TaskPool(TaskPool&&) = delete;
    TaskPool& operator = (TaskPool &) = delete;
    TaskPool& operator = (TaskPool const &) = delete;

    static TaskPoolRef GetPool(){
        static TaskPool task_pool{};
        return (TaskPoolRef)task_pool;
    }

    int PlaceToPool(CallBackT&& f, DelayT d) override{
        int idx = -1;
        for(std::size_t i = 0; i < pool_size; i++){
            if(!pool_[i].has_value()){
                pool_[i] = AsyncTask{std::forward<CallBackT>(f), d};
                idx = i;
                break;
            }
        }
        return idx;
    }

    bool RemoveFromPool(unsigned short idx) override{
        if(idx >= pool_size)
            return false;
        pool_[idx].reset();
        return true;
    }

    bool StopTim(unsigned short idx) override{
        pool_[idx].value().Disable();
        return true;
    }

    void OnTimTick() override{
        for(auto& tim : pool_)
            if(tim.has_value())
                tim.value().TickHandle();
    }

    void Poll() override{
        for(auto& tim : pool_)
            if(tim.has_value())
                tim.value().Poll();
    }
private:
    TaskPool() = default;
    std::array<std::optional<AsyncTask>, pool_size> pool_;
};

}// namespace async_tim_task