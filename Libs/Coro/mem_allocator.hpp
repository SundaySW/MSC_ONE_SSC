#pragma once

#include "cstdint"

class MemoryAllocator{

static const uint32_t kCapacity = 1280;

public:
    static void* Allocate(std::size_t size) {
        if(free_space_ < size)
            return nullptr;
        free_space_ -= size;
        cursor_ += size;
        return cursor_;
    }
    static void Deallocate(void* ptr) {
        int i = 0;
        for(auto* c = data_; c < data_+kCapacity; c++, i++){
            if(ptr == c){
                cursor_ = c;
                free_space_ += i;
            }
        }
    }
private:
    static inline std::size_t data_[kCapacity];
    static inline std::size_t free_space_ = kCapacity;
    static inline std::size_t* cursor_ = data_;
};
