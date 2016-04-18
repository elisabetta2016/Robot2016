/***************************************************************************
 *   Copyright (C) 2015 BlueBotics SA                                      *
 ***************************************************************************/

#include "CANopenMaster.h"

#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>

#define DRIVE_STATUS_WORD_MASK_1 0x004F
#define DRIVE_STATUS_WORD_MASK_2 0x006F

#define PSELECT_TIMEOUT_DEFAULT  10000000 // [in ns]
#define PSELECT_TIMEOUT_DIVIDER  100      // Sync period / 100 [in ns]

CANopenMaster::CANopenMaster(bool _logFrames)
: logFrames(_logFrames)
, socket(-1)
, syncFreq(0.0)
, syncPeriod(0)
, sdoResponseMapMutex(PTHREAD_MUTEX_INITIALIZER)
{
}

CANopenMaster::~CANopenMaster()
{
    ::pthread_cancel(threadId);
    ::pthread_mutex_destroy(&sdoResponseMapMutex);
}

int CANopenMaster::init(const char* _interfaceName)
{
    int retCode = EINVAL;
    for (;;)
    {
        if (::strlen(_interfaceName)+1 > IFNAMSIZ)
        {
            retCode = EINVAL;
            printError("Error during initialization: bad interface name length", retCode);
            break;
        }

        if ((socket = ::socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            retCode = errno;
            printError("Error during initialization: call to socket() failed", retCode);
            break;
        }

        if (::fcntl(socket, F_SETFL, O_NONBLOCK) < 0)
        {
            retCode = errno;
            printError("Error during initialization: call to fcntl() failed", retCode);
            break;
        }

        if (logFrames)
        {
            int optionValue;
            socklen_t optionSize = sizeof(optionValue);
            ::getsockopt(socket, SOL_SOCKET, SO_RCVBUF, &optionValue, &optionSize);
            cout << "Socket buffer sizes:" << endl
                 << "* Receive: " << optionValue << " bytes" << endl;
            ::getsockopt(socket, SOL_SOCKET, SO_SNDBUF, &optionValue, &optionSize);
            cout << "* Send   : " << optionValue << " bytes" << endl << endl;

            int recvOwnMsgs = 1; // 0 = disabled (default), 1 = enabled
            if (::setsockopt(socket, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recvOwnMsgs, sizeof(recvOwnMsgs)) < 0)
            {
                retCode = errno;
                printError("Error during initialization: call to setsockopt() failed", retCode);
                break;
            }
        }

        struct ifreq ifReq;
        ::strcpy(ifReq.ifr_name, _interfaceName);
        if (::ioctl(socket, SIOCGIFINDEX, &ifReq) < 0)
        {
            retCode = errno;
            printError("Error during initialization: call to ioctl() failed", retCode);
            break;
        }

        struct sockaddr_can sockAddr;
        sockAddr.can_ifindex = ifReq.ifr_ifindex;
        if (::bind(socket, (sockaddr*)&sockAddr, sizeof(sockAddr)) < 0)
        {
            retCode = errno;
            printError("Error during initialization: call to bind() failed", retCode);
            break;
        }

        retCode = ::pthread_mutex_init(&sdoResponseMapMutex, NULL); // Returns 0 on success
        if (retCode != 0)
        {
            printError("Error during initialization: call to pthread_mutex_init(&sdoResponseMapMutex) failed", retCode);
            break;
        }

        retCode = ::pthread_create(&threadId, NULL, thread, this); // Returns 0 on success
        if (retCode != 0)
        {
            printError("Error during initialization: call to pthread_create() failed", retCode);
            break;
        }

        retCode = sendNmt(NmtResetNode, 0); // Go to pre-operational state (all nodes)
        if (retCode != 0)
        {
            printError("Error during initialization: call to sendNmt(NmtResetNode) failed", retCode);
            break;
        }

        retCode = configureDrives(); // Should return 0 on success
        if (retCode != 0)
        {
            printError("Error during initialization: call to configureDrives() failed", retCode);
            break;
        }

        retCode = sendNmt(NmtStartRemoteNode, 0); // Go to operational state (all nodes)
        if (retCode != 0)
        {
            printError("Error during initialization: call to sendNmt(NmtStartRemoteNode) failed", retCode);
            break;
        }

        retCode = setSyncFreq(SyncFreqDefault); // Should return 0 on success
        if (retCode != 0)
        {
            printError("Error during initialization: call to setSyncFreq() failed", retCode);
            break;
        }

        retCode = enableDrives(); // Should return 0 on success
        if (retCode != 0)
        {
            printError("Error during initialization: call to enableDrives() failed", retCode);
            break;
        }

        break;
    }

    if (retCode != 0)
    {
        ::close(socket);
        socket = -1;
    }

    return retCode;
}

string CANopenMaster::getFrameString(const can_frame& _frame)
{
    ostringstream frameStream;
    frameStream << setw(3) << setfill('0') << hex << _frame.can_id << '#';
    if (_frame.can_dlc > 0)
    {
        frameStream << setw(2) << setfill('0') << hex << (uint16_t)_frame.data[0];
    }
    for (int i = 1; i < _frame.can_dlc; ++i)
    {
        frameStream << '.' << setw(2) << setfill('0') << hex << (uint16_t)_frame.data[i];
    }
    return frameStream.str();
}

int CANopenMaster::setLogFrames(bool _logFrames)
{
    int recvOwnMsgs = _logFrames ? 1 : 0; // 0 = disabled (default), 1 = enabled
    if (::setsockopt(socket, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recvOwnMsgs, sizeof(recvOwnMsgs)) < 0)
    {
        return errno;
    }
    logFrames = _logFrames;
    return 0;
}

int CANopenMaster::getSyncFreq(float& _freq)
{
    _freq = syncFreq;
    return 0;
}

int CANopenMaster::setSyncFreq(float _freq)
{
    if (_freq < SyncFreqMin || SyncFreqMax < _freq)
    {
        return EINVAL;
    }
    syncFreq = _freq;
    syncPeriod = 1E9/_freq;
    return 0;
}

int CANopenMaster::sendRxPdo(uint8_t _pdoId, uint8_t _nodeId, uint8_t _dataLen, const uint8_t* _data)
{
    if (--_pdoId > 3 || // Valid PDO ids are {1, 2, 3, 4}
        _nodeId > NodeIdMax ||
        _dataLen > 8)
    {
        return EINVAL;
    }
    can_frame frame;
    frame.can_id = (RxPdoBase + PdoOffset*_pdoId) | _nodeId;
    frame.can_dlc = _dataLen;
    ::memcpy(&frame.data[0], &_data[0], _dataLen);
    return sendFrame(frame);
}

int CANopenMaster::sendReadSdo(uint8_t _nodeId, const CANopenObject& _obj, uint32_t& _data)
{
    if (_nodeId > NodeIdMax ||
        (_obj.byteSize != 1 && _obj.byteSize != 2 && _obj.byteSize != 4))
    {
        return EINVAL;
    }
    can_frame frame;
    frame.can_id = RxSdoBase | _nodeId;
    frame.can_dlc = 4;
    frame.data[0] = 0x40;
    valueToLSBBuffer<uint16_t>(_obj.objectId, &frame.data[1]);
    frame.data[3] = _obj.subId;
    int retCode = sendFrame(frame);
    if (retCode != 0)
    {
        return retCode;
    }
    long remainingTime = SdoAnswerTotalWaitDuration;
    uint8_t sdoAnswerCmd = 0x00;
    uint8_t sdoAnswerData[4];
    while (remainingTime > 0 && sdoAnswerCmd == 0x00)
    {
        if (::pthread_mutex_trylock(&sdoResponseMapMutex) == 0)
        {
            unordered_map<uint8_t, can_frame>::iterator it = sdoResponseMap.find(_nodeId);
            if (it != sdoResponseMap.end())
            {
                sdoAnswerCmd = it->second.data[0];
                ::memcpy(&sdoAnswerData[0], &it->second.data[4], 4);
                sdoResponseMap.erase(it);
            }
            ::pthread_mutex_unlock(&sdoResponseMapMutex);
        }
        Time::sleep(0, SdoAnswerTryWaitDuration);
        remainingTime -= SdoAnswerTryWaitDuration;
    }
    switch (sdoAnswerCmd)
    {
        case 0x4F:
            _data = sdoAnswerData[0];
            break;
        case 0x4B:
            _data = LSBBufferToValue<uint16_t>(&sdoAnswerData[0]);
            break;
        case 0x42:
        case 0x43:
            _data = LSBBufferToValue<uint32_t>(&sdoAnswerData[0]);
            break;
        default:
            return EINVAL;
    }
    return 0;
}

int CANopenMaster::sendWriteSdo(uint8_t _nodeId, const CANopenObject& _obj, uint32_t _data)
{
    if (_nodeId > NodeIdMax ||
        (_obj.byteSize != 1 && _obj.byteSize != 2 && _obj.byteSize != 4))
    {
        return EINVAL;
    }
    can_frame frame;
    frame.can_id = RxSdoBase | _nodeId;
    frame.can_dlc = 4 + _obj.byteSize;
    frame.data[0] = 0x23 + (4-_obj.byteSize)*0x04;
    valueToLSBBuffer<uint16_t>(_obj.objectId, &frame.data[1]);
    frame.data[3] = _obj.subId;
    valueToLSBBuffer<uint32_t>(_data, &frame.data[4]);
    int retCode = sendFrame(frame);
    if (retCode != 0)
    {
        return retCode;
    }
    long remainingTime = SdoAnswerTotalWaitDuration;
    uint8_t sdoAnswerCmd = 0x00;
    while (remainingTime > 0 && sdoAnswerCmd == 0x00)
    {
        if (::pthread_mutex_trylock(&sdoResponseMapMutex) == 0)
        {
            unordered_map<uint8_t, can_frame>::iterator it = sdoResponseMap.find(_nodeId);
            if (it != sdoResponseMap.end())
            {
                sdoAnswerCmd = it->second.data[0];
                sdoResponseMap.erase(it);
            }
            ::pthread_mutex_unlock(&sdoResponseMapMutex);
        }
        Time::sleep(0, SdoAnswerTryWaitDuration);
        remainingTime -= SdoAnswerTryWaitDuration;
    }
    if (sdoAnswerCmd != 0x60)
    {
        return EINVAL;
    }
    return 0;
}

EDriveNetworkState CANopenMaster::getDriveNetworkState(uint8_t _state)
{
    switch (_state)
    {
        case EDNSBootup:
        case EDNSStopped:
        case EDNSOperational:
        case EDNSPreOperational:
            return (EDriveNetworkState)_state;
        default:
            return EDNSUnknown;
    }
}

EDriveStatus CANopenMaster::getDriveStatus(uint16_t _statusWord)
{
    struct DriveStatusMatch
    {
        int mask;
        EDriveStatus value;
    };
    static DriveStatusMatch driveStatusMatch[] =
    {
        { DRIVE_STATUS_WORD_MASK_1, EDSNotReadyToSwitchOn  },
        { DRIVE_STATUS_WORD_MASK_1, EDSSwitchOnDisabled    },
        { DRIVE_STATUS_WORD_MASK_2, EDSReadyToSwitchOn     },
        { DRIVE_STATUS_WORD_MASK_2, EDSSwitchedOn          },
        { DRIVE_STATUS_WORD_MASK_2, EDSOperationEnabled    },
        { DRIVE_STATUS_WORD_MASK_2, EDSQuickStopActive     },
        { DRIVE_STATUS_WORD_MASK_1, EDSFaultReactionActive },
        { DRIVE_STATUS_WORD_MASK_1, EDSFault               }
    };
    static const uint8_t driveStatusMatchLen = ARRAY_LENGTH(driveStatusMatch);
    for (int i = 0; i < driveStatusMatchLen; ++i)
    {
        const DriveStatusMatch& dsm = driveStatusMatch[i];
        if ((_statusWord & dsm.mask) == dsm.value)
        {
            return dsm.value;
        }
    }
    return EDSUnknown;
}

void* CANopenMaster::thread(void* _master)
{
    CANopenMaster& master = *(CANopenMaster*)_master;

    fd_set readSet;
    int retCode;
    can_frame frame;
    long prevSyncPeriod = master.syncPeriod;
    Time nextSync;
    for (;;)
    {
        FD_ZERO(&readSet);
        FD_SET(master.socket, &readSet);
        struct timespec timeout = { 0, ((master.syncPeriod > 0) ? master.syncPeriod/PSELECT_TIMEOUT_DIVIDER : PSELECT_TIMEOUT_DEFAULT) };
        retCode = ::pselect(master.socket+1, &readSet, NULL, NULL, &timeout, NULL);
        switch (retCode)
        {
            case 0:
                // The (small) timeout expired, this is by design and not a problem
                break;
            case 1:
                // We received a frame
                if (master.receiveFrame(frame) == 0)
                {
                    uint16_t msgType = frame.can_id & MsgTypeMask;
                    if (msgType == TxSdoBase)
                    {
                        uint8_t nodeId = frame.can_id & NodeIdMask;
                        ::pthread_mutex_lock(&master.sdoResponseMapMutex);
                        master.sdoResponseMap[nodeId] = frame;
                        ::pthread_mutex_unlock(&master.sdoResponseMapMutex);
                    }
                    else
                    {
                        master.onFrameReceived(frame);
                    }
                }
                else
                {
                    // Something went really wrong here because pselect() returned 1 and receiveFrame() returned != 0
                    printError("Error receiving frame", errno);
                }
                break;
            default:
                if (retCode < 0)
                {
                    // pselect() returned an error code
                    printError("Error waiting for frame", errno);
                }
                else
                {
                    // pselect() returned a code > 1 (this one should never happen)
                    printError("Error waiting for frame (return code > 1)", retCode);
                }
                break;
        }

        if (master.syncPeriod > 0)
        {
            Time currentTime = Time::getCurrent();
            if (master.syncPeriod != prevSyncPeriod)
            {
                nextSync = currentTime;
                nextSync.addNSec(master.syncPeriod);
                prevSyncPeriod = master.syncPeriod;
            }
            else if (currentTime >= nextSync)
            {
                nextSync.addNSec(master.syncPeriod);
                master.onSync();
                master.sendSync();
                master.sendRxPdos();
            }
        }
    }

    return NULL;
}

int CANopenMaster::receiveFrame(can_frame& _frame)
{
    ssize_t bytesReceivedNb = ::read(socket, &_frame, sizeof(_frame));
    if (bytesReceivedNb == sizeof(_frame))
    {
        if (logFrames)
        {
            logFrame(_frame);
        }
        return 0;
    }
    else if (bytesReceivedNb < 0)
    {
        printError("Error receiving frame", errno);
        return errno;
    }
    else // bytesReceivedNb >= 0 && bytesReceivedNb != sizeof(_frame)
    {
        // This case is deliberately handled as an error because:
        // * we do not expect partial reads of frames
        // * bytesReceivedNb > sizeof(_frame) should never happen
        printError("Error receiving frame", EINVAL);
        return EINVAL;
    }
}

int CANopenMaster::sendFrame(const can_frame& _frame)
{
    long remainingTime = SocketWriteTotalWaitDuration;
    ssize_t bytesWrittenNb;
    while (remainingTime > 0)
    {
        bytesWrittenNb = ::write(socket, &_frame, sizeof(_frame));
        if (bytesWrittenNb == sizeof(_frame))
        {
            return 0;
        }
        else if (bytesWrittenNb < 0)
        {
            if (errno == EAGAIN || errno == ENOBUFS)
            {
                Time::sleep(0, SocketWriteTryWaitDuration);
                remainingTime -= SocketWriteTryWaitDuration;            
            }
            else
            {
                string errMsg = string("Error sending frame '") + getFrameString(_frame) + "'";
                printError(errMsg.c_str(), errno);
                return errno;
            }
        }
        else // bytesWrittenNb >= 0 && bytesWrittenNb != sizeof(_frame)
        {
            // This case is deliberately handled as an error because:
            // * we do not expect partial writes of frames
            // * bytesWrittenNb > sizeof(_frame) should never happen
            string errMsg = string("Error sending frame '") + getFrameString(_frame) + "'";
            printError(errMsg.c_str(), EINVAL);
            return EINVAL;
        }
    }
    return ETIMEDOUT;
}

int CANopenMaster::sendNmt(uint8_t _cmdId, uint8_t _nodeId)
{
    if ((_cmdId != NmtStartRemoteNode && _cmdId != NmtResetNode) ||
        _nodeId > NodeIdMax)
    {
        return EINVAL;
    }
    can_frame frame;
    frame.can_id = NmtBase;
    frame.can_dlc = 2;
    frame.data[0] = _cmdId;
    frame.data[1] = _nodeId;
    int retCode = sendFrame(frame);
    if (retCode != 0)
    {
        return retCode;
    }
    Time::sleep(0, SendNmtWaitDuration);
    return 0;
}

int CANopenMaster::sendSync()
{
    can_frame frame;
    frame.can_id = SyncBase;
    frame.can_dlc = 0;
    return sendFrame(frame);
}

void CANopenMaster::logFrame(const can_frame& _frame)
{
    Time currentTime = Time::getCurrent();
    cout << setw(0) << setfill(' ') << dec << currentTime.getSec() << '.'
         << setw(6) << setfill('0') << dec << currentTime.getNSec()/1000 << ' '
         << getFrameString(_frame)
         << endl;
}

