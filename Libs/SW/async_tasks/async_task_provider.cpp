#include "async_task_provider.hpp"


async_tim_task::TaskProvider &
async_tim_task::TaskProvider::operator()(async_tim_task::CallBackT&& h, async_tim_task::DelayT d) {
    pool_.PlaceToPool(std::forward<CallBackT>(h), d);
    return *this;
}
