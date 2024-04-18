#pragma once

#include "atomic"

struct CoroMutex{
    bool TryLock(){
        return !lock_.exchange(true);
    }
    void UnLock(){
         lock_.store(false, std::memory_order_relaxed);
    }
private:
    std::atomic<bool> lock_{false};
};
