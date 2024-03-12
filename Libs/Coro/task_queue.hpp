#pragma once

#include <cstring>

template<typename T, std::size_t Size>
class TaskQueue{
public:
    TaskQueue()=default;

    bool push(T& new_elem){
        if(full())
            return false;
        elems_++;
        storage_[tail_] = std::move(new_elem);
        SetTailIdx();
        return true;
    }

    void pop(){
        elems_--;
        SetHeadIdx();
    }

    T& front(){
        return storage_[head_];
    }

    bool full(){
        return elems_ == Size;
    }

    bool empty(){
        return elems_ == 0;
    }
private:
    std::size_t head_{0};
    std::size_t tail_{0};
    std::size_t elems_{0};
    T storage_[Size];

    std::size_t SetTailIdx(){
        if(++tail_ == Size)
            tail_ = 0;
        return tail_;
    }
    std::size_t SetHeadIdx(){
        if(++head_ == Size)
            head_ = 0;
        return head_;
    }
};
