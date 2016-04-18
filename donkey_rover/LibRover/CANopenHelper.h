/***************************************************************************
 *   Copyright (C) 2015 BlueBotics SA                                      *
 ***************************************************************************/

#ifndef CAN_OPEN_HELPER_H

#define CAN_OPEN_HELPER_H

#include <atomic>
#include <cstdint>
#include <iostream>
#include <string.h>
#include <time.h>

using namespace std;

void printError(const char* _errMsg, int _errno);

template <typename T>
void valueToLSBBuffer(T _value, uint8_t* _buffer)
{
    for (uint8_t i = 0; i < sizeof(_value); ++i)
    {
        _buffer[i] = _value & 0xFF;
        _value >>= 8;
    }
}

template <typename T>
T LSBBufferToValue(const uint8_t* _buffer)
{
    T value = 0;
    for (int8_t i = sizeof(value)-1; i >= 0; --i)
    {
        value <<= 8;
        value |= _buffer[i];
    }
    return value;
}

class Time : protected timespec
{
public:
    static const long NanoSecond  = 1;
    static const long MicroSecond = 1000*NanoSecond;
    static const long MilliSecond = 1000*MicroSecond;

    static Time getCurrent();

    static void sleep(time_t _sec, long _nsec);

    Time(bool _atomic = false);

    Time(time_t _sec, long _nsec, bool _atomic = false);

    ~Time();

    Time(const Time& _t);

    Time& operator=(const Time& _right);

    bool operator==(const Time& _right) const;
    bool operator!=(const Time& _right) const;
    bool operator<(const Time& _right) const;
    bool operator<=(const Time& _right) const;
    bool operator>(const Time& _right) const;
    bool operator>=(const Time& _right) const;

    time_t getSec() const;

    long getNSec() const;

    int addNSec(long _nsec);

    int compare(const Time& _right) const;

    void sleep() const;

    string str() const;

protected:
    pthread_mutex_t* pMutex;
};

#endif //CAN_OPEN_HELPER_H

