#pragma once

#include <cstring>

template<typename T, std::size_t Size>
class CoroTaskQueue{
public:
    CoroTaskQueue()=default;
    void push(T new_elem){
        if(cursor < Size)
            cursor++;
        storage[cursor] = std::forward<T>(new_elem);
    }

    bool emplace(T&& elem){
        return true;
    }

    T front(){
        return T();
    }

    bool pop(){
        return true;
    }

    bool empty() const {
        return false;
    }
private:
    std::size_t cursor = 0;
    T storage[Size];
};
