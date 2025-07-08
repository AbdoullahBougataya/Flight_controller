#ifndef PPMDECODER_H
#define PPMDECODER_H

#include <Arduino.h>

class PPMDecoder {
public:
    PPMDecoder(uint8_t ppmPin, uint8_t channelAmount);
    void begin();
    bool available();
    bool isSignalLost();
    unsigned int getChannelValue(uint8_t channel);

private:
    static void isrHandler();
    static void readReceiver();

    static uint8_t _ppmPin;
    static uint8_t _channelAmount;
    static unsigned int _rawValues[10]; // Up to 10 channels supported
    static volatile bool _signalReady;
    static unsigned int _channelIndex;
    static unsigned long _lastTime;
    static unsigned long _duration;
    static unsigned long _lastFrameTime;

    static constexpr unsigned int BLANK_TIME = 2100;
    static constexpr unsigned int MIN_CHANNEL_VALUE = 1000;
    static constexpr unsigned int MAX_CHANNEL_VALUE = 2000;
    static constexpr unsigned int CHANNEL_VALUE_MAX_ERROR = 10;
    static constexpr unsigned long TIMEOUT_DURATION = 500000;
};

#endif // PPMDECODER_H
