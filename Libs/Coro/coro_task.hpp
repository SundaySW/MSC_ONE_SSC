#pragma once

#include <coroutine>
#include <optional>
#include "mem_allocator.hpp"
#include "coro_event.hpp"

template <typename RET_T = bool, typename ARG_T = RET_T>
class CoroTask
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

//        void* operator new(std::size_t size) {
//            return MemoryAllocator::Allocate(size);
//        }
//
//        void operator delete(void* ptr) {
//            MemoryAllocator::Deallocate(ptr);
//        }

        inline CoroTask<RET_T, ARG_T> get_return_object()
        {
            return CoroTask{std::coroutine_handle<Promise>::from_promise(*this) };
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
    using ret_t = RET_T;
    using arg_t = ARG_T;
    using promise_type = Promise;
    using ret_val_t = typename Promise::return_val_type;
    using arg_val_t = typename Promise::arg_val_type;

    explicit CoroTask(std::coroutine_handle<Promise> handle)
        :handle_(handle)
    {}

    ~CoroTask() {
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
        if(handle_){
            handle_.promise().Resume();
            handle_.resume();
        }
    }

    void Resume(){
        if(CanResume())
            HandleResume();
    }

    void Resume(arg_val_t val){
        arg_value_ = std::move(val);
        Resume();
    }

    bool CanResume(){
        return !event_.IsWaiting() && handle_.promise().IsSuspended();
    }

    void StoreArg(arg_val_t val){
        arg_value_ = std::move(val);
    }

    void StoreRetV(ret_val_t val){
        ret_value_ = std::move(val);
    }

    void StoreValueAndNotify(RET_T val){
        ret_value_ = std::move(val);
        event_.Notify();
    }

    void NotifyAwaiter(){
        event_.Notify();
    }

    ret_val_t GetRetOpt(){
        return ret_value_;
    }

    ARG_T GetArgValue(){
        if(arg_value_.has_value()){
            auto ret_val = arg_value_.value();
            arg_value_.reset();
            return ret_val;
        }else
            return {};
    }

    constexpr bool HasStoredArg(){
        return arg_value_.has_value();
    }

    Event& GetEvent(){
        return event_;
    }

    Event::Awaiter operator co_await() noexcept {
        return {event_};
    }

private:
    arg_val_t arg_value_ = std::nullopt;
    ret_val_t ret_value_ = std::nullopt;
    Event event_;
    std::coroutine_handle<Promise> handle_;
};