#pragma once

#include <bit>

#define ow_eeprom_size          (144)
#define ow_table_lut_offset     (26)

struct OWTable{
    using TableItemT = char;

    auto GetFloatFromIdx(std::size_t idx){
        uint32_t ret_val{0};
        idx *= 4;
        idx += ow_table_lut_offset;
        for(std::size_t i = idx, offset = 0; i < sizeof(float); i++, offset++)
            ret_val |= ( table_[idx] << (sizeof(TableItemT) * offset) );
        return std::bit_cast<float>(ret_val);
    }

    float GetRref(){
        return GetFloatFromIdx(0);
    }
    float GetR0(){
        return GetFloatFromIdx(1);
    }
    auto GetPlatinumData() ->std::array<float,3> {
        return {GetFloatFromIdx(2), GetFloatFromIdx(3), GetFloatFromIdx(4)};
    }
    auto* data(){
        return table_.data();
    }
private:
    std::array<TableItemT, ow_eeprom_size> table_;
};
