/***************************************************************************
 *   Copyright (C) 2015 BlueBotics SA                                      *
 ***************************************************************************/

#ifndef TRACK_H

#define TRACK_H

#include "Drive.h"

struct Track : public Drive
{
    Track();
    virtual ~Track();

    Track(const Track& _t);

    Track& operator=(const Track& _right);

    // RxPDO 1 (sent)
    atomic<int32_t> targetVelocity;
    atomic<uint16_t> controlWord;

    // TxPDO 1 (received)
    atomic<int32_t> velocityActualValue;
    atomic<uint16_t> statusWord;
};

#endif //TRACK_H

