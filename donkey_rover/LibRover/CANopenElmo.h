/***************************************************************************
 *   Copyright (C) 2015 BlueBotics SA                                      *
 ***************************************************************************/

#ifndef CAN_OPEN_ELMO_H

#define CAN_OPEN_ELMO_H

#include "CANopenMaster.h"

const CANopenObject AnalogInput1{0x2205, 1, 2};

#define DRIVE_VOLTAGE                  "AN", 6

#define DRIVE_KP                       "KP", 2
#define DRIVE_KI                       "KI", 2

#define SCANNER_HOME_POSITION          "UF", 1
#define SCANNER_ADJUSTMENT             "UF", 2
#define SCANNER_ROLL_ANGLE             "UF", 3
#define SCANNER_ROLL_PERIOD            "UF", 4

#define SCANNER_STATE                  "UI", 1
#define SCANNER_COMMAND                "UI", 2

#define DRIVE_KILL_MOTION_AND_PROGRAM  "KL", 0

#define DRIVE_SAVE_PARAMETERS_TO_FLASH "SV", 0

#endif //CAN_OPEN_ELMO_H

