/***************************************************************************
 *   Copyright (C) 2015 BlueBotics SA                                      *
 ***************************************************************************/

#ifndef DRIVE_H

#define DRIVE_H

#include "CANopenHelper.h"

struct Drive
{
    Drive();
    virtual ~Drive();

    Drive(const Drive& _d);

    Drive& operator=(const Drive& _right);

    // Various data
    atomic<uint32_t> motorRatedCurrent; // Read only once [in mA].
    atomic<float> angleReference; // [in rad]

    // NMT
    atomic<uint8_t> nmtState;

    // TxPDO 3 (received)
    atomic<int16_t> currentActualValue; // [in mA].
    atomic<int16_t> angle;

    // Guarding
    Time lastGuardTime; // [in s].
};

#endif //DRIVE_H

