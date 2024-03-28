#pragma once

#include <optional>
#include "coroutine"

class Event{
public:
    using coro_handle = std::coroutine_handle<>;

    struct Awaiter {
        Event& event_;
        coro_handle handle_ = nullptr;

        Awaiter(Event& event) noexcept
            : event_(event)
        {}

        bool await_ready() const noexcept {
            return event_.IsSet();
        }

        void await_resume() noexcept {
            event_.Reset();
        }

        void await_suspend(coro_handle handle) {
            handle_ = handle;
            event_.SetAwaiter(this);
        }
    };

public:
    void Notify() {
        set_ = true;
        if(consumer_ && consumer_->handle_)
            consumer_->handle_.resume();
    }

    [[nodiscard]] bool IsSet() const {
        return set_;
    }

    [[nodiscard]] bool IsWaiting() const{
        return waiting_;
    }

    void Reset() {
        set_ = false;
        waiting_ = false;
    }

    void SetAwaiter(Awaiter* consumer) {
        consumer_ = consumer;
        waiting_ = true;
    }

    Awaiter operator co_await() noexcept {
        return {*this};
    }

private:
    Awaiter* consumer_ = nullptr;
    bool set_ = false;
    bool waiting_ = false;
};

