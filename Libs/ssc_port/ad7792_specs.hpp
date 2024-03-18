#pragma once

namespace AD7792_adc {
    enum {
        RS0 = 3,
        RW = 6,
        WEN = 7,
    };

    enum registers {
        AD7792_COMMUNICATION_REGISTER = 0,
        AD7792_STATUS_REGISTER = 0,
        AD7792_MODE_REGISTER = 1,
        AD7792_CONFIGURATION_REGISTER = 2,
        AD7792_DATA_REGISTER = 3,
        AD7792_ID_REGISTER = 4,
        AD7792_IO_REGISTER = 5,
        AD7792_OFFSET_REGISTER = 6,
        AD7792_FULLSCALE_REGISTER = 7,
    };

    enum sensorTypes {
        RTD_2currentSources = 1,
        RTD_1currentSource = 2,
        TC_plus_RTDtype2 = 3,
        NTC_onAIN2 = 4
    };

    enum ioCurrents {
        currentDisabled = 0b00000000,
        current_10uA = 0b00000001,
        current_210uA = 0b00000010,
        current_1mA = 0b00000011,
        current_20uA_ch1 = 0b00001001,
        current_20uA_ch2 = 0b00001101,
        current_420uA_ch1 = 0b00001010,
        current_420uA_ch2 = 0b00001110
    };

    enum adcGains {
        gain_1 = 0,
        gain_2 = 1,
        gain_4 = 2,
        gain_8 = 3,
        gain_16 = 4,
        gain_32 = 5,
        gain_64 = 6,
        gain_128 = 7
    };

    enum updateRates {
        fADC_X = 0,
        fADC_470Hz,
        fADC_242Hz,
        fADC_123Hz,
        fADC_62Hz,
        fADC_50Hz,
        fADC_39Hz,
        fADC_33_2Hz,
        fADC_19_6Hz,
        fADC_16_7Hz_50HzRejection,
        fADC_16_7Hz,
        fADC_12_5Hz,
        fADC_10Hz,
        fADC_8_33Hz,
        fADC_6_25Hz,
        fADC_4_17Hz
    };

    enum adcReferenceSources {
        external = 0,
        Internal = 1
    };

    enum adcChannels {
        AIN1 = 0,
        AIN2 = 1,
        AIN3 = 2,
        AIN1_ = 3,
        TempSensor = 6,
        AVdd = 7
    };
}