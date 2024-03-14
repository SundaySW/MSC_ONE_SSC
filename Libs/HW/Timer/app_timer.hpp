#pragma once

#include <functional>
#include <cstdint>

    /**
     * @brief To use:
     * impl weak HAL func:
     * void HAL_IncTick()
     * {
            uwTick += uwTickFreq;
            SysTickTimHandler();
        }
        ***in SysTickTimHandler() call TickHandle() for all your impl timers in loop

        void SysTickTimHandler(){
            for(auto & timer : timers_)
                timer->TickHandle();
        }

        ***Impl timer with lambda or functor in ctor
        AppTimer tim1_ {[&]{SomeFunc();}};

        std::array<AppTimer*, 2> timers_{
            &Tim1,
            &Tim2
        };
     */

class AppTimer {
public:
    using HandlerT = std::function<void()>;

    AppTimer() = delete;
    explicit AppTimer(HandlerT handler, uint32_t msDelay)
        :handler_(std::move(handler)),
         interval_(msDelay)
    {
        StartTimer();
    }

    void TickHandle(){
        if(disabled_ || has_pending_)
            return;
        if(count_ += KTick_freq_ >= interval_)
            has_pending_ = true;
    }

    void Poll(){
        if(has_pending_){
            handler_();
            has_pending_ = false;
            count_ = 0;
        }
    }

    void SetDelay(uint32_t delay){
        interval_ = delay;
    }

    void StopTimer(){
        disabled_ = true;
    }

    void StartTimer(){
        disabled_ = false;
    }
private:
    volatile uint32_t count_{0};
    volatile uint32_t interval_{0};
    uint32_t KTick_freq_ = 1;
    bool disabled_ {true};
    HandlerT handler_;
    bool has_pending_{false};
};

