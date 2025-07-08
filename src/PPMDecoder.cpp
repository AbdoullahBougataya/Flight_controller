#include "../include/PPMDecoder.h"

uint8_t PPMDecoder::_ppmPin;
uint8_t PPMDecoder::_channelAmount;
unsigned int PPMDecoder::_rawValues[10];
volatile bool PPMDecoder::_signalReady = false;
unsigned int PPMDecoder::_channelIndex = 0;
unsigned long PPMDecoder::_lastTime = 0;
unsigned long PPMDecoder::_duration = 0;
unsigned long PPMDecoder::_lastFrameTime = 0;

PPMDecoder::PPMDecoder(uint8_t ppmPin, uint8_t channelAmount)
{
    _ppmPin = ppmPin;
    _channelAmount = channelAmount;
}

void PPMDecoder::begin()
{
    // Initialize the _rawValues array to zero
    for (uint8_t i = 0; i < _channelAmount; ++i) {
        _rawValues[i] = 0;
    }

    // Attach an interrupt to the pin
    pinMode(_ppmPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_ppmPin), isrHandler, RISING);

    _lastFrameTime = micros();
}

bool PPMDecoder::available()
{
    if (_signalReady) {
        _signalReady = false;
        return true;
    }
    return false;
}

bool PPMDecoder::isSignalLost()
{
    unsigned long now = micros();
    return (now - _lastFrameTime) > TIMEOUT_DURATION;
}

unsigned int PPMDecoder::getChannelValue(uint8_t channel)
{
    if (channel >= 0 && channel < _channelAmount) {
        return _rawValues[channel];
    }
    return 0;
}

void PPMDecoder::isrHandler()
{
    readReceiver();
    _signalReady = true;
}

void PPMDecoder::readReceiver()
{
    unsigned long currentTime = micros();
    _duration = currentTime - _lastTime;
    _lastTime = currentTime;

    int min = MIN_CHANNEL_VALUE - CHANNEL_VALUE_MAX_ERROR;
    int max = MAX_CHANNEL_VALUE + CHANNEL_VALUE_MAX_ERROR;

    if (_duration > BLANK_TIME) {
        _channelIndex = 0;
        _lastFrameTime = currentTime;
    }
    else if (_channelIndex < _channelAmount && _duration >= min && _duration <= max) {
        _rawValues[_channelIndex] = _duration;
        _channelIndex++;

        if (_channelIndex == _channelAmount) {
            _lastFrameTime = currentTime;
        }
    }
}
