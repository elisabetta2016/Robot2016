/***************************************************************************
 *   Copyright (C) 2015 BlueBotics SA                                      *
 ***************************************************************************/

#include "CANopenHelper.h"

#include <iomanip>
#include <sstream>

void printError(const char* _errMsg, int _errno)
{
    cout << _errMsg << ": errno=" << _errno << " (" << ::strerror(_errno) << ')' << endl;
}

Time Time::getCurrent()
{
    Time t;
    ::clock_gettime(CLOCK_REALTIME, &t);
    return t;
}

void Time::sleep(time_t _sec, long _nsec)
{
    Time(_sec, _nsec).sleep();
}

Time::Time(bool _atomic)
: timespec{0, 0}
, pMutex(_atomic ? new pthread_mutex_t(PTHREAD_MUTEX_INITIALIZER) : NULL)
{
    if (pMutex != NULL)
    {
        ::pthread_mutex_init(pMutex, NULL);
    }
}

Time::Time(time_t _sec, long _nsec, bool _atomic)
: timespec{_sec, _nsec}
, pMutex(_atomic ? new pthread_mutex_t(PTHREAD_MUTEX_INITIALIZER) : NULL)
{
    if (pMutex != NULL)
    {
        ::pthread_mutex_init(pMutex, NULL);
    }
}

Time::~Time()
{
    if (pMutex != NULL)
    {
        ::pthread_mutex_destroy(pMutex);
    }
}

Time::Time(const Time& _t)
: timespec{_t.tv_sec, _t.tv_nsec}
, pMutex(_t.pMutex != NULL ? new pthread_mutex_t(PTHREAD_MUTEX_INITIALIZER) : NULL)
{
    if (pMutex != NULL)
    {
        ::pthread_mutex_init(pMutex, NULL);
    }
}

Time& Time::operator=(const Time& _right)
{
    if (pMutex != NULL)
    {
        ::pthread_mutex_lock(pMutex);
    }
    tv_sec = _right.tv_sec;
    tv_nsec = _right.tv_nsec;
    if (pMutex != NULL)
    {
        ::pthread_mutex_unlock(pMutex);
    }
    return *this;
}

bool Time::operator==(const Time& _right) const
{
    return compare(_right) == 0;
}

bool Time::operator!=(const Time& _right) const
{
    return compare(_right) != 0;
}

bool Time::operator<(const Time& _right) const
{
    return compare(_right) < 0;
}

bool Time::operator<=(const Time& _right) const
{
    return compare(_right) <= 0;
}

bool Time::operator>(const Time& _right) const
{
    return compare(_right) > 0;
}

bool Time::operator>=(const Time& _right) const
{
    return compare(_right) >= 0;
}

time_t Time::getSec() const
{
    if (pMutex != NULL)
    {
        ::pthread_mutex_lock(pMutex);
    }
    time_t ret = tv_sec;
    if (pMutex != NULL)
    {
        ::pthread_mutex_unlock(pMutex);
    }
    return ret;
}

long Time::getNSec() const
{
    if (pMutex != NULL)
    {
        ::pthread_mutex_lock(pMutex);
    }
    time_t ret = tv_nsec;
    if (pMutex != NULL)
    {
        ::pthread_mutex_unlock(pMutex);
    }
    return ret;
}

int Time::addNSec(long _nsec)
{
    if (_nsec <= 0 || _nsec >= 1000000000)
    {
        return EINVAL;
    }
    if (pMutex != NULL)
    {
        ::pthread_mutex_lock(pMutex);
    }
    tv_nsec += _nsec;
    if (tv_nsec > 1000000000)
    {
        ++tv_sec;
        tv_nsec -= 1000000000;
    }
    if (pMutex != NULL)
    {
        ::pthread_mutex_unlock(pMutex);
    }
    return 0;
}

int Time::compare(const Time& _right) const
{
    if (pMutex != NULL)
    {
        ::pthread_mutex_lock(pMutex);
    }
    int ret;
    if (tv_sec < _right.tv_sec)
    {
        ret = -1;
    }
    else if (tv_sec > _right.tv_sec)
    {
        ret = 1;
    }
    else
    {
        ret = tv_nsec - _right.tv_nsec;
    }
    if (pMutex != NULL)
    {
        ::pthread_mutex_unlock(pMutex);
    }
    return ret;
}

void Time::sleep() const
{
    if (pMutex != NULL)
    {
        ::pthread_mutex_lock(pMutex);
    }
    timespec t = *this;
    if (pMutex != NULL)
    {
        ::pthread_mutex_unlock(pMutex);
    }
    ::nanosleep(&t, NULL);
}

string Time::str() const
{
    if (pMutex != NULL)
    {
        ::pthread_mutex_lock(pMutex);
    }
    ostringstream frameStream;
    frameStream << tv_sec << '.' << setw(9) << setfill('0') << tv_nsec;
    if (pMutex != NULL)
    {
        ::pthread_mutex_unlock(pMutex);
    }
    return frameStream.str();
}

