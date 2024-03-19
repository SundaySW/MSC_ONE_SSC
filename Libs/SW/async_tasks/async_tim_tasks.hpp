#pragma once

#include "async_task_provider.hpp"
#include "fwd.hpp"

namespace async_tim_task{

TaskPoolRef GetTaskPool();
#define TASK_POOL_ON_TIM()  async_tim_task::GetTaskPool().OnTimTick()
#define TASK_POOL_POLL()    async_tim_task::GetTaskPool().Poll()
#define PLACE_TO(pool, f, d) async_tim_task::TaskProvider(pool).AsLvalue().Place(f,d)
#define PLACE_ASYNC_TASK(f, d) PLACE_TO(async_tim_task::GetTaskPool(), (f), (d))
#define REMOVE_TASK(n) async_tim_task::TaskProvider(async_tim_task::GetTaskPool()).AsLvalue().Remove(n)

}// namespace async_tim_task

