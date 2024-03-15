#pragma once

#include <functional>

namespace async_tim_task{

    /**
 * @brief possible use:
 * impl weak HAL func:
 * void HAL_IncTick()
 * {
        uwTick += uwTickFreq;
        SysTickTimHandler();
    }
    or:
    use MACRO - TASK_POOL_ON_TIM in interrupt handler
    and MACRO - TASK_POOL_POLL im poll loop
 */

class AsyncTask {
public:
    AsyncTask() = delete;

    explicit AsyncTask(CallBackT&& handler, DelayT delay)
        : handler_(std::move(handler))
        , interval_(delay)
    {
        StartTimer();
    }

    void TickHandle(){
        if(disabled_ || has_pending_)
            return;
        if(count_ += kTick_freq_ >= interval_)
            has_pending_ = true;
    }

    void Poll(){
        if(has_pending_){
            handler_();
            has_pending_ = false;
            count_ = 0;
        }
    }

    void SetDelay(unsigned int delay){
        interval_ = delay;
    }

    void Enable(){
        disabled_ = true;
    }

    void Disable(){
        disabled_ = false;
    }
private:
    volatile uint32_t count_{0};
    volatile uint32_t interval_{0};
    uint32_t kTick_freq_ = 1;
    bool disabled_ {true};
    bool has_pending_ {false};
    CallBackT handler_;
};

}// namespace async_tim_task
