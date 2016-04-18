/***************************************************************************
 *   Copyright (C) 2015 BlueBotics SA                                      *
 ***************************************************************************/

#include "Drive.h"

Drive::Drive()
: motorRatedCurrent(0)
, angleReference(0.0)
, nmtState(0)
, currentActualValue(0)
, angle(0)
, lastGuardTime(true)
{
}

Drive::~Drive()
{
}

Drive::Drive(const Drive& _d)
: motorRatedCurrent(_d.motorRatedCurrent.load())
, angleReference(_d.angleReference.load())
, nmtState(_d.nmtState.load())
, currentActualValue(_d.currentActualValue.load())
, angle(_d.angle.load())
, lastGuardTime(_d.lastGuardTime)
{
}

Drive& Drive::operator=(const Drive& _right)
{
    motorRatedCurrent = _right.motorRatedCurrent.load();
    angleReference = _right.angleReference.load();
    nmtState = _right.nmtState.load();
    currentActualValue = _right.currentActualValue.load();
    angle = _right.angle.load();
    lastGuardTime = _right.lastGuardTime;
    return *this;
}

