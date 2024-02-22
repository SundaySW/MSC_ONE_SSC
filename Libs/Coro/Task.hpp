#pragma once

#include <coroutine>
#include <optional>
#include "MemoryAllocator.hpp"
#include "Event.hpp"

template <typename RET_T, typename ARG_T = RET_T>
class Task
{
    class Promise
    {
    public:
        using return_val_type = std::optional<RET_T>;
        using arg_val_type = std::optional<ARG_T>;

        Promise() = default;
        std::suspend_always initial_suspend() {
            suspended_ = true;
            return {};
        }
        std::suspend_always final_suspend() noexcept {
            suspended_ = true;
            return {};
        }
        void unhandled_exception() {}

        std::suspend_always yield_value(RET_T v) {
            suspended_ = true;
            this->value = std::move(v);
            return {};
        }

        return_val_type get_value() {
            return value;
        }

        void* operator new(std::size_t size) {
            return MemoryAllocator::Allocate(size);
        }

        void operator delete(void* ptr) {
            MemoryAllocator::Deallocate(ptr);
        }

        inline Task<RET_T, ARG_T> get_return_object()
        {
            return Task{ std::coroutine_handle<Promise>::from_promise(*this) };
        }

        void return_void() {
            suspended_ = false;
        }

        void Resume(){
            suspended_ = false;
        }

        bool IsSuspended(){
            return suspended_;
        }

    private:
        return_val_type value{};
        bool suspended_ = false;
    };

public:
    using value_type = RET_T;
    using promise_type = Promise;
    using ret_val_t = typename Promise::return_val_type;
    using arg_val_t = typename Promise::arg_val_type;

    explicit Task(std::coroutine_handle<Promise> handle)
            : handle_(handle)
    {}

    ~Task() {
        if (handle_)
            handle_.destroy();
    }

    ret_val_t Next() {
        if (handle_) {
            Resume();
            return handle_.promise().get_value();
        }
        else
            return {};
    }

    ret_val_t GetValue() {
        if (handle_) {
            return std::move(handle_.promise().get_value());
        }
        else
            return {};
    }

    void HandleResume(){
        handle_.promise().Resume();
        if(handle_)
            handle_.resume();
    }

    void Resume(){
        if(CanResume())
            HandleResume();
    }

    void Resume(arg_val_t val){
        stored_value_ = std::move(val);
        if(CanResume())
            HandleResume();
    }

    bool CanResume(){
        return !event_.IsWaiting() && handle_.promise().IsSuspended();
    }

    void StoreArgument(arg_val_t val){
        stored_value_ = std::move(val);
    }

    void StoreRetVal(ret_val_t val){
        ret_value_ = std::move(val);
    }

    void StoreValueAndNotify(RET_T val){
        ret_value_ = std::move(val);
        event_.Notify();
    }

    void NotifyAwaiter(){
        event_.Notify();
    }

    ret_val_t GetRetVal(){
        return ret_value_;
    }

    ARG_T GetArgumentValue(){
        if(stored_value_.has_value()){
            auto ret_val = stored_value_.value();
            stored_value_.reset();
            return ret_val;
        }else
            return {};
    }

    constexpr bool HasStoredArgument(){
        return stored_value_.has_value();
    }

    Event& GetEvent(){
        return event_;
    }

    Event::Awaiter operator co_await() noexcept {
        return {event_};
    }

private:
    arg_val_t stored_value_ = std::nullopt;
    ret_val_t ret_value_ = std::nullopt;
    Event event_;
    std::coroutine_handle<Promise> handle_;
};