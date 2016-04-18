/***************************************************************************
 *   Copyright (C) 2015 BlueBotics SA                                      *
 ***************************************************************************/

/** \file  CANopenMaster.h
 *  \brief Base class for a CANopen master.
 */

#ifndef CAN_OPEN_MASTER_H

#define CAN_OPEN_MASTER_H

#include <pthread.h>
#include <unordered_map>

#include <linux/can/raw.h>

#include "CANopenHelper.h"

#define ARRAY_LENGTH(array)                    (sizeof(array)/sizeof(array[0]))

struct CANopenObject
{
    uint16_t objectId;
    uint8_t subId;
    uint8_t byteSize;
};

#define PDO_COM_COB_ID(pdoComId)               CANopenObject{(pdoComId), 1, 4}

#define PDO_COM_TRANSMISSION_TYPE(pdoComId)    CANopenObject{(pdoComId), 2, 1}

#define PDO_MAP_NB_OF_ENTRIES(pdoMapId)        CANopenObject{(pdoMapId), 0, 1}
#define PDO_MAP_ENTRY(pdoMapId, entryId)       CANopenObject{(pdoMapId), (entryId), 4}
#define PDO_MAP_OBJECT(obj)                    (uint32_t)(((obj).objectId)<<16) | (((obj).subId)<<8) | (((obj).byteSize)*8)

const uint16_t      NmtBase                    = 0x000;
const uint16_t      SyncBase                   = 0x080;
const uint16_t      TxPdoBase                  = 0x180;
const uint16_t      RxPdoBase                  = 0x200;
const uint16_t      PdoOffset                  = 0x100;
const uint16_t      TxPdo1Base                 = TxPdoBase;
const uint16_t      RxPdo1Base                 = RxPdoBase;
const uint16_t      TxPdo2Base                 = TxPdoBase + PdoOffset;
const uint16_t      RxPdo2Base                 = RxPdoBase + PdoOffset;
const uint16_t      TxPdo3Base                 = TxPdoBase + 2*PdoOffset;
const uint16_t      RxPdo3Base                 = RxPdoBase + 2*PdoOffset;
const uint16_t      TxPdo4Base                 = TxPdoBase + 3*PdoOffset;
const uint16_t      RxPdo4Base                 = RxPdoBase + 3*PdoOffset;
const uint16_t      TxSdoBase                  = 0x580;
const uint16_t      RxSdoBase                  = 0x600;
const uint16_t      GuardingBase               = 0x700;

const uint16_t      MsgTypeMask                = 0x780;
const uint16_t      NodeIdMask                 = 0x7F;

const uint16_t      NodeIdMax                  = 0x7F;

const uint8_t       NmtStartRemoteNode         = 0x01;
const uint8_t       NmtResetNode               = 0x81;

const uint16_t      RxPdo1Com                  = 0x1400;
const uint16_t      RxPdo2Com                  = 0x1401;
const uint16_t      RxPdo3Com                  = 0x1402;
const uint16_t      RxPdo4Com                  = 0x1403;

const uint16_t      TxPdo1Com                  = 0x1800;
const uint16_t      TxPdo2Com                  = 0x1801;
const uint16_t      TxPdo3Com                  = 0x1802;
const uint16_t      TxPdo4Com                  = 0x1803;

const uint32_t      PdoComDisableFlag          = 0x80000000;

const uint8_t       PdoComCyclicSynchronous    = 1;
const uint8_t       PdoComAsynchronous         = 255;

const uint16_t      RxPdo1Map                  = 0x1600;
const uint16_t      RxPdo2Map                  = 0x1601;
const uint16_t      RxPdo3Map                  = 0x1602;
const uint16_t      RxPdo4Map                  = 0x1603;

const uint16_t      TxPdo1Map                  = 0x1A00;
const uint16_t      TxPdo2Map                  = 0x1A01;
const uint16_t      TxPdo3Map                  = 0x1A02;
const uint16_t      TxPdo4Map                  = 0x1A03;

const CANopenObject ProducerHeartbeatTime      {0x1017, 0, 2};

const CANopenObject ControlWord                {0x6040, 0, 2};

const CANopenObject StatusWord                 {0x6041, 0, 2};

const CANopenObject ModesOfOperation           {0x6060, 0, 1};
const uint8_t       ProfileVelocityMode        = 3;

const CANopenObject VelocityActualValue        {0x606C, 0, 4};

const CANopenObject MotorRatedCurrent          {0x6075, 0, 4};
const CANopenObject CurrentActualValue         {0x6078, 0, 2};

const CANopenObject ProfileAcceleration        {0x6083, 0, 4};
const CANopenObject ProfileDeceleration        {0x6084, 0, 4};

const CANopenObject MotionProfileType          {0x6086, 0, 2};
const uint16_t      LinearRamp                 = 0; /* Trapezoidal profile */

const CANopenObject TargetVelocity             {0x60FF, 0, 4};

const long          SendNmtWaitDuration        = 500*Time::MilliSecond;

const float         SyncFreqMin                = 10.0;  // Should always be > 1.0 [in Hz]
const float         SyncFreqMax                = 200.0; // Should always be > 1.0 [in Hz]
const float         SyncFreqDefault            = 100.0; // Should always be > 1.0 [in Hz]
                                                        // Note: the rover even works at 200Hz!

// We try to write the socket during SocketWriteTotalWaitDuration by steps of SocketWriteTryWaitDuration
const long          SocketWriteTotalWaitDuration = 10*Time::MilliSecond;
const long          SocketWriteTryWaitDuration   = 20*Time::MicroSecond;

// We wait each SDO answer during SdoAnswerTotalWaitDuration by steps of SdoAnswerTryWaitDuration
const long          SdoAnswerTotalWaitDuration   = 100*Time::MilliSecond;
const long          SdoAnswerTryWaitDuration     = 50*Time::MicroSecond;

/** \enum  EDriveNetworkState
 *  \brief Enumeration to describe the network state of a drive.
 *  \note  This is the CANopen NMT state.
 */
enum EDriveNetworkState
{
    EDNSUnknown        =  -1, ///< The drive is in an unknown state
    EDNSBootup         =   0, ///< The drive is booting
    EDNSStopped        =   4, ///< The drive is stopped
    EDNSOperational    =   5, ///< The drive is operational
    EDNSPreOperational = 127  ///< The drive is pre-operational
};

/** \enum  EDriveStatus
 *  \brief Enumeration to describe the internal status of a drive.
 *  \note  This represents the CANopen status word.
 */
enum EDriveStatus
{
    EDSUnknown             =     -1, ///< The drive is in an unknown state
    EDSNotReadyToSwitchOn  = 0x0000, ///< The drive is in the "Not Ready To Switch On" state
    EDSSwitchOnDisabled    = 0x0040, ///< The drive is in the "Switch On Disabled" state
    EDSReadyToSwitchOn     = 0x0021, ///< The drive is ready to switch on
    EDSSwitchedOn          = 0x0023, ///< The drive is switched on
    EDSOperationEnabled    = 0x0027, ///< The drive is operational
    EDSQuickStopActive     = 0x0007, ///< The drive is in the "Quick Stop Active" state
    EDSFaultReactionActive = 0x000F, ///< The drive is in the "Fault Reaction Active" state
    EDSFault               = 0x0008  ///< The drive is faulty
};

class CANopenMaster
{
public:
    CANopenMaster(bool _logFrames = false);
    virtual ~CANopenMaster();

    virtual int init(const char* _interfaceName = "can0");

    virtual int configureDrives() = 0;

    virtual int enableDrives() = 0;

    virtual int disableDrives() = 0;

    virtual int onSync() = 0;     // Called just before sending the sync message

    virtual int sendRxPdos() = 0; // Called just aftert sending the sync message

    virtual int onFrameReceived(const can_frame& _frame) = 0;

    virtual string getFrameString(const can_frame& _frame);

    int setLogFrames(bool _logFrames = true);

    int getSyncFreq(float& _freq);

    int setSyncFreq(float _freq);

    int sendRxPdo(uint8_t _pdoId, uint8_t _nodeId, uint8_t _dataLen, const uint8_t* _data);

    int sendReadSdo(uint8_t _nodeId, const CANopenObject& _obj, uint32_t& _data);

    int sendWriteSdo(uint8_t _nodeId, const CANopenObject& _obj, uint32_t _data);

protected:
    static EDriveNetworkState getDriveNetworkState(uint8_t _state);

    static EDriveStatus getDriveStatus(uint16_t _statusWord);

    CANopenMaster(const CANopenMaster& _m) = delete;

    CANopenMaster& operator=(const CANopenMaster& _right) = delete;

    bool logFrames;
    int socket;
    pthread_t threadId;
    atomic<float> syncFreq;  // Should always be > 1.0 [in Hz]
                             // Restricted to [SyncFreqMin, SyncFreqMax], default = SyncFreqMax
    atomic<long> syncPeriod; // Should always be < 1000000000 [in ns]
    pthread_mutex_t sdoResponseMapMutex;
    unordered_map<uint8_t, can_frame> sdoResponseMap;

    static void* thread(void* _master);

    int receiveFrame(can_frame& _frame);

    int sendFrame(const can_frame& _frame);

    int sendNmt(uint8_t _cmdId, uint8_t _nodeId);

    int sendSync();

    void logFrame(const can_frame& _frame);
};

#endif //CAN_OPEN_MASTER_H

