#pragma once

#include <functional>
#include <cstdint>
#include "base_task_pool.hpp"

namespace async_tim_task{

struct TaskProvider{
    TaskProvider(BaseTaskPool& pool)
        :pool_(pool)
    {}
    TaskProvider(TaskProvider&&) = delete;
    TaskProvider(const TaskProvider&) = delete;
    TaskProvider& operator=(TaskProvider&&) = delete;
    TaskProvider& operator=(const TaskProvider&) = delete;
    TaskProvider& operator() (CallBackT&& h, DelayT d);
    TaskProvider& AsLvalue() { return *this; }
private:
    BaseTaskPool& pool_;
};
}// namespace async_tim_task