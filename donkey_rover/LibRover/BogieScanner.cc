/***************************************************************************
 *   Copyright (C) 2015 BlueBotics SA                                      *
 ***************************************************************************/

#include "BogieScanner.h"

BogieScanner::BogieScanner()
{
}

BogieScanner::~BogieScanner()
{
}

BogieScanner::BogieScanner(const BogieScanner& _bs)
: Drive(_bs)
{
}

BogieScanner& BogieScanner::operator=(const BogieScanner& _right)
{
    Drive::operator=(_right);
    return *this;
}

