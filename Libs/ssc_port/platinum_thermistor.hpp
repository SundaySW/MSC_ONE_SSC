#pragma once

#include <cmath>
#include <cstdint>

struct PRTD{
    void StoreRref(float v){
        R_ref = v;
    }
    void StoreR0(float v){
        R0 = v;
    }
    void StorePlatinumData(std::array<float,3> data){
        A = data[0],
        B = data[1],
        C = data[2];
        a = B,
        b = A;
        b_d2 = b/2,
        r = - (b_d2 / a),
        b_sq = b * b,
        b_d2sq = b_d2 * b_d2;
    }
    [[nodiscard]] float CalcT2(float adc_v) const{
        float Rt = R_ref * (adc_v / UINT16_MAX),
                c = 1 - Rt / R0,
                D = b_sq - (4 * a * c);
        return (-b + sqrtf(D)) / (2 * a);
    }

    [[nodiscard]] float CalcT(float adc_v) const{
        auto Rt = R_ref * (adc_v / UINT16_MAX),
                c = 1 - Rt / R0,
                D = b_d2sq - a * c;
        return r + (sqrtf(D) / a);
    }
private:
    float R_ref = 270,
          R0 = 100;
    float A = 3.9083e-3,
          B = -5.775e-7,
          C = -4.183e-12;
    float& a = B,
           b = A;
    float  b_d2 = b/2,
           r = - (b_d2 / a),
           b_sq = b * b,
           b_d2sq = b_d2 * b_d2;
};
