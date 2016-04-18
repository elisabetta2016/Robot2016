/***************************************************************************
 *   Copyright (C) 2015 BlueBotics SA                                      *
 ***************************************************************************/

#include "libRover.h"

#include <sstream>

#include "CANopenElmo.h"

Rover::Rover(bool _logFrames)
: CANopenMaster(_logFrames)
, pdo2ResponseMapMutex(PTHREAD_MUTEX_INITIALIZER)
, timestampNotUpdated(true)
, timestamp(true)
, driveIdToDrive({{EDIFrontRightTrack, &frontRightTrack},
                  {EDIFrontLeftTrack,  &frontLeftTrack },
                  {EDIRearRightTrack,  &rearRightTrack },
                  {EDIRearLeftTrack,   &rearLeftTrack  },
                  {EDIBogieScanner,    &bogieScanner   }})
, driveIdToAngleSign{{EDIFrontRightTrack,  1.0},
                     {EDIFrontLeftTrack,  -1.0},
                     {EDIRearRightTrack,   1.0},
                     {EDIRearLeftTrack,   -1.0},
                     {EDIBogieScanner,     1.0}}
, scannerAdjustmentSign(-1.0)
, trackIdToSpeedSign{{EDIFrontRightTrack, -1.0},
                     {EDIFrontLeftTrack,   1.0},
                     {EDIRearRightTrack,  -1.0},
                     {EDIRearLeftTrack,    1.0}}
{
}

Rover::~Rover()
{
    ::pthread_mutex_destroy(&pdo2ResponseMapMutex);
}

int Rover::init(const char* _interfaceName)
{
    int retCode = ::pthread_mutex_init(&pdo2ResponseMapMutex, NULL); // Returns 0 on success
    if (retCode != 0)
    {
        printError("Error during initialization: call to pthread_mutex_init(&pdo2ResponseMapMutex) failed", retCode);
        return retCode;
    }

    return CANopenMaster::init(_interfaceName);
}

RoverParams& Rover::getParams()
{
    return roverParams;
}

int Rover::setLogFrames(bool _logFrames)
{
    return CANopenMaster::setLogFrames(_logFrames);
}

int Rover::getSyncFreq(float& _freq)
{
    return CANopenMaster::getSyncFreq(_freq);
}

int Rover::setSyncFreq(float _freq)
{
    return CANopenMaster::setSyncFreq(_freq);
}

int Rover::getTimestamp(Time& _timestamp)
{
    _timestamp = timestamp;
    return 0;
}

int Rover::enableTracks()
{
    return enableDrives();
}

int Rover::disableTracks()
{
    return disableDrives();
}

int Rover::getCurrent(EDriveId _driveId, Time& _timestamp, float& _current)
{
    _timestamp = timestamp;
    Drive* drive = driveIdToDrive[_driveId];
    _current = (float)drive->motorRatedCurrent * drive->currentActualValue / 1E6;
    return 0;
}

int Rover::readVoltage(EDriveId _driveId, Time& _timestamp, float& _voltage)
{
    _timestamp = timestamp;
    return sendRxPdo2(_driveId, Rover::ECTGet, DRIVE_VOLTAGE, _voltage);
}

int Rover::getNetworkState(EDriveId _driveId, EDriveNetworkState& _state)
{
    return CANopenMaster::getDriveNetworkState(driveIdToDrive[_driveId]->nmtState);
}

int Rover::getTrackStatus(EDriveId _driveId, uint16_t& _statusWord, EDriveStatus& _status)
{
    if (_driveId < EDIFirstTrack || EDILastTrack < _driveId)
    {
        return EINVAL;
    }
    _statusWord = driveIdToTrack(_driveId)->statusWord;
    _status = getDriveStatus(_statusWord);
    return 0;
}

int Rover::getAngleReference(EDriveId _driveId, float& _angle)
{
    _angle = driveIdToAngleSign[_driveId] * driveIdToDrive[_driveId]->angleReference;
    return 0;
}

int Rover::setAngleReference(EDriveId _driveId, float _angle)
{
    driveIdToDrive[_driveId]->angleReference = driveIdToAngleSign[_driveId] * _angle;
    return 0;
}

int Rover::getAngle(EDriveId _driveId, float& _angle)
{
    Drive* drive = driveIdToDrive[_driveId];
    _angle = driveIdToAngleSign[_driveId] * ((float)drive->angle*getAnalogInput1ToRadRatio() - drive->angleReference);
    return 0;
}

int Rover::getAngles(Time& _timestamp, float& _frontLeft, float& _frontRight, float& _rearLeft, float& _rearRight, float& _bogie)
{
    _timestamp = timestamp;
    float ratio = getAnalogInput1ToRadRatio();
    _frontLeft = driveIdToAngleSign[EDIFrontLeftTrack] * ((float)frontLeftTrack.angle * ratio - frontLeftTrack.angleReference);
    _frontRight = driveIdToAngleSign[EDIFrontRightTrack] * ((float)frontRightTrack.angle * ratio - frontRightTrack.angleReference);
    _rearLeft = driveIdToAngleSign[EDIFrontLeftTrack] * ((float)rearLeftTrack.angle * ratio - rearLeftTrack.angleReference);
    _rearRight = driveIdToAngleSign[EDIFrontRightTrack] * ((float)rearRightTrack.angle * ratio - rearRightTrack.angleReference);
    _bogie = driveIdToAngleSign[EDIBogieScanner] * ((float)bogieScanner.angle * ratio - bogieScanner.angleReference);
    return 0;
}

int Rover::getPI(EDriveId _driveId, float& _kp, float& _ki)
{
    int retCode = sendRxPdo2(_driveId, Rover::ECTGet, DRIVE_KP, _kp);
    if (retCode != 0)
    {
        return retCode;
    }
    return sendRxPdo2(_driveId, Rover::ECTGet, DRIVE_KI, _ki);
}

int Rover::setPI(EDriveId _driveId, float _kp, float _ki)
{
    int retCode = sendRxPdo2(_driveId, Rover::ECTSet, DRIVE_KP, _kp);
    if (retCode != 0)
    {
        return retCode;
    }
    return sendRxPdo2(_driveId, Rover::ECTSet, DRIVE_KI, _ki);
}

int Rover::getScannerHomePosition(float& _pos)
{
    return sendRxPdo2(EDIBogieScanner, Rover::ECTGet, SCANNER_HOME_POSITION, _pos);
}

int Rover::setScannerHomePosition(float _pos)
{
    return sendRxPdo2(EDIBogieScanner, Rover::ECTSet, SCANNER_HOME_POSITION, _pos);
}

int Rover::getScannerAdjustment(float& _adj)
{
    int retCode = sendRxPdo2(EDIBogieScanner, Rover::ECTGet, SCANNER_ADJUSTMENT, _adj);
    if (retCode != 0)
    {
        return retCode;
    }
    _adj *= scannerAdjustmentSign;
    return 0;
}

int Rover::setScannerAdjustment(float _adj)
{
    float adj = scannerAdjustmentSign * _adj;
    return sendRxPdo2(EDIBogieScanner, Rover::ECTSet, SCANNER_ADJUSTMENT, adj);
}

int Rover::getScannerAngle(float& _angle)
{
    return sendRxPdo2(EDIBogieScanner, Rover::ECTGet, SCANNER_ROLL_ANGLE, _angle);
}

int Rover::setScannerAngle(float _angle)
{
    // Note: the angle is checked inside the drive
    return sendRxPdo2(EDIBogieScanner, Rover::ECTSet, SCANNER_ROLL_ANGLE, _angle);
}

int Rover::getScannerPeriod(float& _period)
{
    return sendRxPdo2(EDIBogieScanner, Rover::ECTGet, SCANNER_ROLL_PERIOD, _period);
}

int Rover::setScannerPeriod(float _period)
{
    // Note: the period is checked inside the drive
    return sendRxPdo2(EDIBogieScanner, Rover::ECTSet, SCANNER_ROLL_PERIOD, _period);
}

int Rover::getScannerState(EScannerState& _state)
{
    int state;
    int retCode = sendRxPdo2(EDIBogieScanner, Rover::ECTGet, SCANNER_STATE, state);
    if (retCode != 0)
    {
        return retCode;
    }
    switch (state)
    {
        case ESSIdle:
        case ESSCommandReceived:
        case ESSHoming:
        case ESSGoingHome:
        case ESSRolling:
            _state = (EScannerState)state;
            break;
        default:
            _state = ESSUnknown;
            break;
    }
    return 0;
}

int Rover::sendScannerCommand(EScannerCommand _command)
{
    EScannerState currentState;
    int retCode = getScannerState(currentState);
    if (retCode != 0)
    {
        return retCode;
    }
    if ((currentState == ESSIdle && _command != ESCDoHoming
                                 && _command != ESCGoHome
                                 && _command != ESCStart) ||
        (currentState == ESSRolling && _command != ESCGoHome
                                    && _command != ESCStop))
    {
        return EINVAL;
    }
    int command;
    int desiredState = ESSCommandReceived;
    if (currentState == ESSRolling && _command == ESCGoHome)
    {
        // Stop rolling first
        command = ESCStop;
        retCode = sendRxPdo2(EDIBogieScanner, Rover::ECTSet, SCANNER_COMMAND, command);
        if (retCode != 0)
        {
            return retCode;
        }
        retCode = sendRxPdo2(EDIBogieScanner, Rover::ECTSet, SCANNER_STATE, desiredState);
        if (retCode != 0)
        {
            return retCode;
        }
        do
        {
            Time::sleep(0, SdoAnswerTryWaitDuration);
            retCode = getScannerState(currentState);
        }
        while (currentState != ESSIdle);
    }
    command = _command;
    retCode = sendRxPdo2(EDIBogieScanner, Rover::ECTSet, SCANNER_COMMAND, command);
    if (retCode != 0)
    {
        return retCode;
    }
    desiredState = ESSCommandReceived;
    retCode = sendRxPdo2(EDIBogieScanner, Rover::ECTSet, SCANNER_STATE, desiredState);
    if (retCode != 0)
    {
        return retCode;
    }
    return 0;
}

int Rover::getSpeed(Time& _timestamp, float& _frontLeft, float& _frontRight, float& _rearLeft, float& _rearRight)
{
    _timestamp = timestamp;
    // Read the 4 speeds as fast as we can (in sequence)
    _frontLeft = frontLeftTrack.velocityActualValue;
    _frontRight = frontRightTrack.velocityActualValue;
    _rearLeft = rearLeftTrack.velocityActualValue;
    _rearRight = rearRightTrack.velocityActualValue;
    // Apply the conversion ratio
    float ratio = getTrackCountToRadRatio();
    _frontLeft *= trackIdToSpeedSign[EDIFrontLeftTrack] * ratio;
    _frontRight *= trackIdToSpeedSign[EDIFrontRightTrack] * ratio;
    _rearLeft *= trackIdToSpeedSign[EDIRearLeftTrack] * ratio;
    _rearRight *= trackIdToSpeedSign[EDIRearRightTrack] * ratio;
    return 0;
}

int Rover::setSpeed(float _frontLeft, float _frontRight, float _rearLeft, float _rearRight)
{
    // Apply the conversion ratio
    float ratio = getRadToTrackCountRatio();
    _frontLeft *= trackIdToSpeedSign[EDIFrontLeftTrack] * ratio;
    _frontRight *= trackIdToSpeedSign[EDIFrontRightTrack] * ratio;
    _rearLeft *= trackIdToSpeedSign[EDIRearLeftTrack] * ratio;
    _rearRight *= trackIdToSpeedSign[EDIRearRightTrack] * ratio;
    // Write the 4 speeds as fast as we can (in sequence)
    frontLeftTrack.targetVelocity = _frontLeft;
    frontRightTrack.targetVelocity = _frontRight;
    rearLeftTrack.targetVelocity = _rearLeft;
    rearRightTrack.targetVelocity = _rearRight;
    return 0;
}

int Rover::getSpeed(Time& _timestamp, float& _left, float& _right)
{
    _timestamp = timestamp;
    // Read the 4 speeds as fast as we can (in sequence)
    float frontLeft = frontLeftTrack.velocityActualValue;
    float frontRight = frontRightTrack.velocityActualValue;
    float rearLeft = rearLeftTrack.velocityActualValue;
    float rearRight = rearRightTrack.velocityActualValue;
    // Apply the conversion ratio
    float ratio = getTrackCountToRadRatio();
    _left = trackIdToSpeedSign[EDIFrontLeftTrack] * (frontLeft+rearLeft) / 2.0 * ratio;
    _right = trackIdToSpeedSign[EDIFrontRightTrack] * (frontRight+rearRight) / 2.0 * ratio;
    return 0;
}

int Rover::setSpeed(float _left, float _right)
{
    // Apply the conversion ratio
    float ratio = getRadToTrackCountRatio();
    _left *= trackIdToSpeedSign[EDIFrontLeftTrack] * ratio;
    _right *= trackIdToSpeedSign[EDIFrontRightTrack] * ratio;
    // Write the 4 speeds as fast as we can (in sequence)
    frontLeftTrack.targetVelocity = _left;
    frontRightTrack.targetVelocity = _right;
    rearLeftTrack.targetVelocity = _left;
    rearRightTrack.targetVelocity = _right;
    return 0;
}

int Rover::getSpeedInMPerS(Time& _timestamp, float& _frontLeft, float& _frontRight, float& _rearLeft, float& _rearRight)
{
    _timestamp = timestamp;
    // Read the 4 speeds as fast as we can (in sequence)
    _frontLeft = frontLeftTrack.velocityActualValue;
    _frontRight = frontRightTrack.velocityActualValue;
    _rearLeft = rearLeftTrack.velocityActualValue;
    _rearRight = rearRightTrack.velocityActualValue;
    // Apply the conversion ratio
    float ratio = getTrackCountToMRatio();
    _frontLeft *= trackIdToSpeedSign[EDIFrontLeftTrack] * ratio;
    _frontRight *= trackIdToSpeedSign[EDIFrontRightTrack] * ratio;
    _rearLeft *= trackIdToSpeedSign[EDIRearLeftTrack] * ratio;
    _rearRight *= trackIdToSpeedSign[EDIRearRightTrack] * ratio;
    return 0;
}

int Rover::setSpeedInMPerS(float _frontLeft, float _frontRight, float _rearLeft, float _rearRight)
{
    // Apply the conversion ratio
    float ratio = getMToTrackCountRatio();
    _frontLeft *= trackIdToSpeedSign[EDIFrontLeftTrack] * ratio;
    _frontRight *= trackIdToSpeedSign[EDIFrontRightTrack] * ratio;
    _rearLeft *= trackIdToSpeedSign[EDIRearLeftTrack] * ratio;
    _rearRight *= trackIdToSpeedSign[EDIRearRightTrack] * ratio;
    // Write the 4 speeds as fast as we can (in sequence)
    frontLeftTrack.targetVelocity = _frontLeft;
    frontRightTrack.targetVelocity = _frontRight;
    rearLeftTrack.targetVelocity = _rearLeft;
    rearRightTrack.targetVelocity = _rearRight;
    return 0;
}

int Rover::getSpeedInMPerS(Time& _timestamp, float& _left, float& _right)
{
    _timestamp = timestamp;
    // Read the 4 speeds as fast as we can (in sequence)
    float frontLeft = frontLeftTrack.velocityActualValue;
    float frontRight = frontRightTrack.velocityActualValue;
    float rearLeft = rearLeftTrack.velocityActualValue;
    float rearRight = rearRightTrack.velocityActualValue;
    // Apply the conversion ratio
    float ratio = getTrackCountToMRatio();
    _left = trackIdToSpeedSign[EDIFrontLeftTrack] * (frontLeft+rearLeft) / 2.0 * ratio;
    _right = trackIdToSpeedSign[EDIFrontRightTrack] * (frontRight+rearRight) / 2.0 * ratio;
    return 0;
}

int Rover::setSpeedInMPerS(float _left, float _right)
{
    // Apply the conversion ratio
    float ratio = getMToTrackCountRatio();
    _left *= trackIdToSpeedSign[EDIFrontLeftTrack] * ratio;
    _right *= trackIdToSpeedSign[EDIFrontRightTrack] * ratio;
    // Write the 4 speeds as fast as we can (in sequence)
    frontLeftTrack.targetVelocity = _left;
    frontRightTrack.targetVelocity = _right;
    rearLeftTrack.targetVelocity = _left;
    rearRightTrack.targetVelocity = _right;
    return 0;
}

int Rover::getSpeedVO(Time& _timestamp, float& _v, float& _omega)
{
    float left, right;
    int retCode = getSpeedInMPerS(_timestamp, left, right);
    if (retCode != 0)
    {
        return retCode;
    }
    _v = (left+right) / 2.0;
    _omega = (right-left) / roverParams.TrackLeftToRightDistanceCalibrated;
    return 0;
}

int Rover::setSpeedVO(float _v, float _omega)
{
	float left = (2.0*_v - roverParams.TrackLeftToRightDistanceCalibrated*_omega) / 2.0;
    float right = (2.0*_v - left);
    return setSpeedInMPerS(left, right);
}

void Rover::logDrivesData()
{
    cout << "Timestamp=" << timestamp.str() << "s" << endl;
    Time t;
    float angle, current, voltage;
    for (EDriveId driveId = EDIFirstDrive; driveId <= EDILastDrive; driveId = (EDriveId)(driveId+1))
    {
        Drive* drive = driveIdToDrive[driveId];
        getAngle(driveId, angle);
        getCurrent(driveId, t, current);
        readVoltage(driveId, t, voltage);
        cout << "Drive id=" << driveId << ": networkState=" << (uint16_t)drive->nmtState
        << ", angle=" << angle << "rad"
        << ", current=" << current << "A"
        << ", voltage=" << voltage << "V"
        << ", lastGuardTime=" << drive->lastGuardTime.str() << "s" << endl;
    }
    for (EDriveId driveId = EDIFirstTrack; driveId <= EDILastTrack; driveId = (EDriveId)(driveId+1))
    {
        Track* track = driveIdToTrack(driveId);
        cout << "Track id=" << driveId
             << ": targetVelocity=" << track->targetVelocity
             << ", controlWord=" << track->controlWord
             << ", velocityActualValue=" << track->velocityActualValue
             << ", statusWord=" << track->statusWord
             << endl;
    }
    cout << endl;
}

int Rover::configureDrives()
{
    struct WriteSdoValue
    {
        CANopenObject object;
        uint32_t value;
    };

    int retCode;
    uint32_t pdoCobId, ratedCurrent;

    // Configure velocity mode

    cout << "Configuring velocity mode ..." << endl;

    WriteSdoValue velocityCmd[] =
    {
        { ModesOfOperation,    ProfileVelocityMode      },
        { MotionProfileType,   LinearRamp               }, // Trapezoidal velocity profile
        { ProfileAcceleration, ProfileAccelerationValue },
        { ProfileDeceleration, ProfileDecelerationValue }
    };
    const int VelocityCmdNb = ARRAY_LENGTH(velocityCmd);
    for (EDriveId driveId = EDIFirstTrack; driveId <= EDILastTrack; driveId = (EDriveId)(driveId+1))
    {
        for (int cmd = 0; cmd < VelocityCmdNb; ++cmd)
        {
            retCode = sendWriteSdo(driveId, velocityCmd[cmd].object, velocityCmd[cmd].value);
            if (retCode != 0)
            {
                return retCode;
            }
        }
    }

    // Configure RxPDO 1

    cout << "Configuring RxPDO 1 ..." << endl;

    WriteSdoValue rxPdo1Cmd[] =
    {
        { PDO_COM_TRANSMISSION_TYPE(RxPdo1Com), PdoComAsynchronous             },
        { PDO_MAP_NB_OF_ENTRIES(RxPdo1Map),     0                              },
        { PDO_MAP_ENTRY(RxPdo1Map, 1),          PDO_MAP_OBJECT(TargetVelocity) },
        { PDO_MAP_ENTRY(RxPdo1Map, 2),          PDO_MAP_OBJECT(ControlWord)    },
        { PDO_MAP_NB_OF_ENTRIES(RxPdo1Map),     2                              }
    };
    const int RxPdo1CmdNb = ARRAY_LENGTH(rxPdo1Cmd);
    for (EDriveId driveId = EDIFirstTrack; driveId <= EDILastTrack; driveId = (EDriveId)(driveId+1))
    {
        retCode = sendReadSdo(driveId, PDO_COM_COB_ID(RxPdo1Com), pdoCobId);
        if (retCode != 0)
        {
            return retCode;
        }

        retCode = sendWriteSdo(driveId, PDO_COM_COB_ID(RxPdo1Com), PdoComDisableFlag | pdoCobId);
        if (retCode != 0)
        {
            return retCode;
        }
        
        for (int cmd = 0; cmd < RxPdo1CmdNb; ++cmd)
        {
            retCode = sendWriteSdo(driveId, rxPdo1Cmd[cmd].object, rxPdo1Cmd[cmd].value);
            if (retCode != 0)
            {
                return retCode;
            }
        }

        retCode = sendWriteSdo(driveId, PDO_COM_COB_ID(RxPdo1Com), ~PdoComDisableFlag & pdoCobId);
        if (retCode != 0)
        {
            return retCode;
        }
    }

    // RxPDO 2 is used by the binary interpreter

    // Configure TxPDO 1

    cout << "Configuring TxPDO 1 ..." << endl;

    WriteSdoValue txPdo1Cmd[] =
    {
        { PDO_COM_TRANSMISSION_TYPE(TxPdo1Com), PdoComCyclicSynchronous             },
        { PDO_MAP_NB_OF_ENTRIES(TxPdo1Map),     0                                   },
        { PDO_MAP_ENTRY(TxPdo1Map, 1),          PDO_MAP_OBJECT(VelocityActualValue) },
        { PDO_MAP_ENTRY(TxPdo1Map, 2),          PDO_MAP_OBJECT(StatusWord)          },
        { PDO_MAP_NB_OF_ENTRIES(TxPdo1Map),     2                                   }
    };
    const int TxPdo1CmdNb = ARRAY_LENGTH(txPdo1Cmd);
    for (EDriveId driveId = EDIFirstTrack; driveId <= EDILastTrack; driveId = (EDriveId)(driveId+1))
    {
        retCode = sendReadSdo(driveId, PDO_COM_COB_ID(TxPdo1Com), pdoCobId);
        if (retCode != 0)
        {
            return retCode;
        }

        retCode = sendWriteSdo(driveId, PDO_COM_COB_ID(TxPdo1Com), PdoComDisableFlag | pdoCobId);
        if (retCode != 0)
        {
            return retCode;
        }
        
        for (int cmd = 0; cmd < TxPdo1CmdNb; ++cmd)
        {
            retCode = sendWriteSdo(driveId, txPdo1Cmd[cmd].object, txPdo1Cmd[cmd].value);
            if (retCode != 0)
            {
                return retCode;
            }
        }

        retCode = sendWriteSdo(driveId, PDO_COM_COB_ID(TxPdo1Com), ~PdoComDisableFlag & pdoCobId);
        if (retCode != 0)
        {
            return retCode;
        }
    }

    // TxPDO 2 is used by the binary interpreter

    // Configure TxPDO 3

    cout << "Configuring TxPDO 3 ..." << endl;

    WriteSdoValue txPdo3Cmd[] =
    {
        { PDO_COM_TRANSMISSION_TYPE(TxPdo3Com), PdoComCyclicSynchronous            },
        { PDO_MAP_NB_OF_ENTRIES(TxPdo3Map),     0                                  },
        { PDO_MAP_ENTRY(TxPdo3Map, 1),          PDO_MAP_OBJECT(CurrentActualValue) },
        { PDO_MAP_ENTRY(TxPdo3Map, 2),          PDO_MAP_OBJECT(AnalogInput1)       },
        { PDO_MAP_NB_OF_ENTRIES(TxPdo3Map),     2                                  }
    };
    const int TxPdo3CmdNb = ARRAY_LENGTH(txPdo3Cmd);
    for (EDriveId driveId = EDIFirstDrive; driveId <= EDILastDrive; driveId = (EDriveId)(driveId+1))
    {
        // Get the motor rated current
        retCode = sendReadSdo(driveId, MotorRatedCurrent, ratedCurrent);
        if (retCode != 0)
        {
            return retCode;
        }
        driveIdToDrive[driveId]->motorRatedCurrent = ratedCurrent;

        retCode = sendReadSdo(driveId, PDO_COM_COB_ID(TxPdo3Com), pdoCobId);
        if (retCode != 0)
        {
            return retCode;
        }

        retCode = sendWriteSdo(driveId, PDO_COM_COB_ID(TxPdo3Com), PdoComDisableFlag | pdoCobId);
        if (retCode != 0)
        {
            return retCode;
        }

        for (int cmd = 0; cmd < TxPdo3CmdNb; ++cmd)
        {
            retCode = sendWriteSdo(driveId, txPdo3Cmd[cmd].object, txPdo3Cmd[cmd].value);
            if (retCode != 0)
            {
                return retCode;
            }
        }

        retCode = sendWriteSdo(driveId, PDO_COM_COB_ID(TxPdo3Com), ~PdoComDisableFlag & pdoCobId);
        if (retCode != 0)
        {
            return retCode;
        }
    }

    // Configure hearbeat

    cout << "Configuring hearbeat ..." << endl;

    for (EDriveId driveId = EDIFirstDrive; driveId <= EDILastDrive; driveId = (EDriveId)(driveId+1))
    {
        retCode = sendWriteSdo(driveId, ProducerHeartbeatTime, ProducerHeartbeatTimeValue);
        if (retCode != 0)
        {
            return retCode;
        }
    }

    cout << "Configuration done." << endl << endl;

    return 0;
}

int Rover::enableDrives()
{
    // We only enable the tracks.

    struct EnableDrive
    {
        int command;
        EDriveStatus status;
    };

    EnableDrive enableDrivesCmd[] =
    {
        { 0x0006, EDSReadyToSwitchOn  }, // Go to state "Ready to switch on"
        { 0x000F, EDSOperationEnabled }  // Go to state "Operation enabled"
    };
    const int EnableDrivesCmdNb = ARRAY_LENGTH(enableDrivesCmd);
    for (int i = 0; i < EnableDrivesCmdNb; ++i)
    {
        for (EDriveId driveId = EDIFirstTrack; driveId <= EDILastTrack; driveId = (EDriveId)(driveId+1))
        {
            driveIdToTrack(driveId)->controlWord = enableDrivesCmd[i].command;
        }
        bool allOk = false;
        while (!allOk)
        {
            allOk = true;
            for (EDriveId driveId = EDIFirstTrack; driveId <= EDILastTrack; driveId = (EDriveId)(driveId+1))
            {
                allOk = allOk && (getDriveStatus(driveIdToTrack(driveId)->statusWord) == enableDrivesCmd[i].status);
            }
            Time::sleep(0, EnableDrivesWaitDuration);
        }
    }

    return 0;
}

int Rover::disableDrives()
{
    // We only disable the tracks.

    for (EDriveId driveId = EDIFirstTrack; driveId <= EDILastTrack; driveId = (EDriveId)(driveId+1))
    {
        driveIdToTrack(driveId)->controlWord = 0; // Go to state "Switch on disabled"
    }
    bool allOk = false;
    while (!allOk)
    {
        allOk = true;
        for (EDriveId driveId = EDIFirstTrack; driveId <= EDILastTrack; driveId = (EDriveId)(driveId+1))
        {
            allOk = allOk && ((driveIdToTrack(driveId)->statusWord & 0x6F) == 0x40);
        }
        Time::sleep(0, EnableDrivesWaitDuration);
    }

    return 0;
}

int Rover::onSync()
{
    timestampNotUpdated = true;
    return 0;
}

int Rover::sendRxPdos()
{
    int retCode;
    for (EDriveId driveId = EDIFirstTrack; driveId <= EDILastTrack; driveId = (EDriveId)(driveId+1))
    {
        Track* t = driveIdToTrack(driveId);
        retCode = sendRxPdo1(driveId, t->targetVelocity, t->controlWord);
        if (retCode != 0)
        {
            return retCode;
        }
    }
    return 0;
}

int Rover::onFrameReceived(const can_frame& _frame)
{
    uint16_t msgType = _frame.can_id & MsgTypeMask;
    EDriveId driveId = (EDriveId)(_frame.can_id & NodeIdMask);
    if (driveId < EDIFirstDrive || EDILastDrive < driveId)
    {
        return EINVAL;
    }
    switch (msgType)
    {
        case NmtBase:
            driveIdToDrive[driveId]->nmtState = _frame.data[0];
            break;
        case TxPdo1Base: // TxPDO 1
            if (EDIFirstTrack <= driveId && driveId <= EDILastTrack)
            {
                if (timestampNotUpdated)
                {
                    timestamp = Time::getCurrent();
                    timestampNotUpdated = false;
                }
                Track* t = driveIdToTrack(driveId);
                t->velocityActualValue = LSBBufferToValue<int32_t>(&_frame.data[0]);
                t->statusWord = LSBBufferToValue<uint16_t>(&_frame.data[4]);
            }
            break;
        case TxPdo2Base: // TxPDO 2
            ::pthread_mutex_lock(&pdo2ResponseMapMutex);
            pdo2ResponseMap[driveId] = _frame;
            ::pthread_mutex_unlock(&pdo2ResponseMapMutex);
            break;
        case TxPdo3Base: // TxPDO 3
            if (timestampNotUpdated)
            {
                timestamp = Time::getCurrent();
                timestampNotUpdated = false;
            }
            driveIdToDrive[driveId]->currentActualValue = LSBBufferToValue<int16_t>(&_frame.data[0]);
            driveIdToDrive[driveId]->angle = LSBBufferToValue<int16_t>(&_frame.data[2]);
            break;
        case GuardingBase:
            driveIdToDrive[driveId]->lastGuardTime = Time::getCurrent();
            break;
        default:
            break;
    };
    return 0;
}

string Rover::getFrameString(const can_frame& _frame)
{
    ostringstream frameStream;
    frameStream << CANopenMaster::getFrameString(_frame);
    if (_frame.can_dlc > 2 && ::isupper(_frame.data[0]) && isupper(_frame.data[1]))
    {
        uint16_t index = (_frame.data[3]&0x3F)<<8 | _frame.data[2];
        frameStream << " (" << _frame.data[0] << _frame.data[1] << '[' << index << "])";
    }
    return frameStream.str();
}

Track* Rover::driveIdToTrack(EDriveId _driveId)
{
    if (_driveId < EDIFirstTrack || EDILastTrack < _driveId)
    {
        return NULL;
    }
    return (Track*)driveIdToDrive[_driveId];
}

float Rover::getAnalogInput1ToRadRatio()
{
    return M_2PI/AnalogInput1Range;
}

float Rover::getTrackCountToRadRatio()
{
    return M_2PI/(roverParams.TrackEncoderResolution*roverParams.TrackGearRatio);
}

float Rover::getRadToTrackCountRatio()
{
    return (roverParams.TrackEncoderResolution*roverParams.TrackGearRatio)/M_2PI;
}

float Rover::getTrackCountToMRatio()
{
    return (M_2PI*roverParams.TrackWheelRadius)/(roverParams.TrackEncoderResolution*roverParams.TrackGearRatio);
}

float Rover::getMToTrackCountRatio()
{
    return (roverParams.TrackEncoderResolution*roverParams.TrackGearRatio)/(M_2PI*roverParams.TrackWheelRadius);
}

int Rover::sendRxPdo1(uint8_t _nodeId, int32_t _targetVelocity, uint16_t _controlWord)
{
    uint8_t data[sizeof(_targetVelocity)+sizeof(_controlWord)];
    const uint8_t dataLen = ARRAY_LENGTH(data);
    valueToLSBBuffer<int32_t>(_targetVelocity, &data[0]);
    valueToLSBBuffer<uint16_t>(_controlWord, &data[sizeof(_targetVelocity)]);
    return sendRxPdo(1, _nodeId, dataLen, data);
}

int Rover::saveDriveParametersToFlash(uint8_t _driveId)
{
    int cmdRetCode = 0;
    int retCode = sendRxPdo2(EDIBogieScanner, Rover::ECTCmd, DRIVE_KILL_MOTION_AND_PROGRAM, cmdRetCode);
    if (retCode != 0)
    {
        return retCode;
    }
    if (cmdRetCode != 0)
    {
        return cmdRetCode;
    }
    retCode = sendRxPdo2(EDIBogieScanner, Rover::ECTCmd, DRIVE_SAVE_PARAMETERS_TO_FLASH, cmdRetCode);
    if (retCode != 0)
    {
        return retCode;
    }
    return cmdRetCode;
}

