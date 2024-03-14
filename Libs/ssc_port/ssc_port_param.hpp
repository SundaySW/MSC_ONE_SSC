#pragma once

struct SSCPortParam : UpdateParam, FloatParam, CalibrParam
{
    using Func = void(*)(char, float*);
    SSCPortParam() = default;
    SSCPortParam(Func f)
            : saveToEEPROM(f)
    {}

    void SetADC(SpiADC* adc){
        adc_ = (adc);
    }

    bool QueryInterface(INTERFACE_ID iid, void*& p) override
    {
        switch (iid)
        {
            case IID_UPDATABLE:
                p = (UpdateParam*)this;
                return true;
            case IID_CALIBRATEABLE:
                p = (CalibrParam*)this;
                return true;
            default:
                return false;
        }
    }

    void SaveToEEPROM() override {
        float data[2] = {Offset, Mult};
        saveToEEPROM(Id, data);
    }

    bool UpdateValue() override
    {
        float value;
        if (adc_->GetValue(value)){
            Value = Calibrate(value);
            return true;
        }
        return false;
    }

private:
    Func saveToEEPROM = nullptr;
    SpiADC* adc_ = nullptr;
};

