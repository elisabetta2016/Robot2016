/***************************************************************************
 *   Copyright (C) 2015 BlueBotics SA                                      *
 ***************************************************************************/

#ifndef BOGIE_SCANNER_H

#define BOGIE_SCANNER_H

#include "Drive.h"

struct BogieScanner : public Drive
{
    BogieScanner();
    virtual ~BogieScanner();

    BogieScanner(const BogieScanner& _bs);

    BogieScanner& operator=(const BogieScanner& _right);
};

#endif //BOGIE_SCANNER_H

