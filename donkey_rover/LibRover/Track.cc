/***************************************************************************
 *   Copyright (C) 2015 BlueBotics SA                                      *
 ***************************************************************************/

#include "Track.h"

Track::Track()
: targetVelocity(0)
, controlWord(0)
, velocityActualValue(0)
, statusWord(0)
{
}

Track::~Track()
{
}

Track::Track(const Track& _t)
: Drive(_t)
, targetVelocity(_t.targetVelocity.load())
, controlWord(_t.controlWord.load())
, velocityActualValue(_t.velocityActualValue.load())
, statusWord(_t.statusWord.load())
{
}

Track& Track::operator=(const Track& _right)
{
    Drive::operator=(_right);
    targetVelocity = _right.targetVelocity.load();
    controlWord = _right.controlWord.load();
    velocityActualValue = _right.velocityActualValue.load();
    statusWord = _right.statusWord.load();
    return *this;
}

