#pragma once

#include "app_timer.hpp"
#include "optional"
#include "tim.h"

namespace {
    const std::size_t pool_size = 20;
}

struct TimersPool{
    TimersPool(TimersPool&) = delete;
    TimersPool(TimersPool&&) = delete;
    TimersPool& operator = (TimersPool &) = delete;
    TimersPool& operator = (TimersPool const &) = delete;

    static TimersPool& global(){
        static auto self = TimersPool();
        return self;
    }

    void SetTim(TIM_HandleTypeDef* htim){
        htim_ = htim;
        HAL_TIM_Base_Start_IT(htim_);
    }

    int PlaceToPool(AppTimer&& timer){
        int idx = -1;
        for(std::size_t i = 0; i < pool_size; i++){
            if(!pool_[i].has_value()){
                pool_[i] = timer;
                idx = i;
            }
        }
        return idx;
    }

    bool RemoveFromPool(uint16_t idx){
        if(idx >= pool_size)
            return false;
        pool_[idx].reset();
        return true;
    }

    bool StopTim(uint16_t idx){
        pool_[idx].value().StopTimer();
        return true;
    }

    void OnTimTick(){
        for(auto& tim : pool_)
            tim.value().TickHandle();
    }

    void Poll(){
        for(auto& tim : pool_)
            tim.value().Poll();
    }
private:
    TimersPool() = default;
    TIM_HandleTypeDef* htim_;
    std::array<std::optional<AppTimer>, pool_size> pool_;
};


