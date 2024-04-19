#pragma once

#include <functional>

#include "fwd.hpp"

namespace async_tim_task{

#define kTick_freq_ (1)

class AsyncTask {
public:
    AsyncTask() = default;

    explicit AsyncTask(CallBackT&& handler, DelayT delay, bool suspended = false)
            : handler_(std::move(handler))
            , interval_(delay)
            , inited_(true)
    {
        if(!suspended)
            Enable();
    }

    void TickHandle(){
        if(disabled_ || has_pending_)
            return;
        else
            count_ += kTick_freq_;
    }

    void Poll(){
        if(!disabled_ && count_ >= interval_){
            has_pending_ = true;
            handler_();
            count_ = 0;
            has_pending_ = false;
        }
    }

    void SetDelay(unsigned int delay){
        interval_ = delay;
    }

    void Enable(){
        disabled_ = false;
    }

    void Disable(){
        disabled_ = true;
    }

    [[nodiscard]] bool IsInited() const{
        return inited_;
    }

    void Reset(){
        disabled_ = true;
        inited_ = true;
    }
private:
    std::size_t count_{0};
    std::size_t interval_{0};
    bool inited_ {false};
    bool disabled_ {true};
    bool has_pending_ {false};
    CallBackT handler_;
};

}// namespace async_tim_task
