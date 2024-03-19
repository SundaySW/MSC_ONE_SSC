#include "async_task_provider.hpp"


async_tim_task::TaskProvider &
async_tim_task::TaskProvider::operator()(async_tim_task::CallBackT&& h, async_tim_task::DelayT d) {
    pool_.PlaceToPool(std::forward<CallBackT>(h), d);
    return *this;
}

int async_tim_task::TaskProvider::Place(async_tim_task::CallBackT &&h, async_tim_task::DelayT d) {
    return pool_.PlaceToPool(std::forward<CallBackT>(h), d);
}

bool async_tim_task::TaskProvider::Remove(unsigned short n) {
    return pool_.RemoveFromPool(n);
}
