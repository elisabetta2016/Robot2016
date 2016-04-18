/***************************************************************************
 *   Copyright (C) 2015 BlueBotics SA                                      *
 ***************************************************************************/

/** \file  libRover.h
 *  \brief API controlling the Sherpa rover.
 *
 * \attention The int code returned by the functions is an error code.
 *            It may originates from lower level functions related to CANopen,
 *            sockets or others. A code different from zero is an error.
 *            All units used throughout the API are S.I. ones (e.g. rad, meter,
 *            second, Hz etc).
 *
 * \mainpage BlueBotics Sherpa LibRover Source Code Documentation.
 * \section  Platform Overview.
 * \author   Copyright (C) 2015 BlueBotics SA.
 */

#ifndef LIB_ROVER_H

#define LIB_ROVER_H

#include "CANopenMaster.h"

#include <math.h>

#include "BogieScanner.h"
#include "Track.h"

#define M_2PI (2.0*M_PI)

const uint32_t ProfileAccelerationValue   = 3000; // [in count/s^2].
const uint32_t ProfileDecelerationValue   = 3000; // [in count/s^2].

const uint16_t ProducerHeartbeatTimeValue = 300; // [in ms].

const long     EnableDrivesWaitDuration   = 50*Time::MicroSecond;

const int      AnalogInput1Range          = 1<<14; // The analog input (for the angle) is converted to 14bits.

// Helper template for binary interpreter commands sent to RxPdo2.
template <typename T> struct ECommandValue {};
template <> struct ECommandValue<int>   { static const int typeFlag = 0; };
template <> struct ECommandValue<float> { static const int typeFlag = 1; };

/** \enum  EDriveId
 *  \brief Enumeration to identify the 5 drives consisting of 4 tracks and 1 bogie/scanner.
 *         The values are the CAN ids of the drives.
 */
enum EDriveId   
{
    EDIFrontRightTrack = 2, ///< The front right track
    EDIFrontLeftTrack  = 3, ///< The front left track
    EDIRearRightTrack  = 4, ///< The rear right track
    EDIRearLeftTrack   = 5, ///< The rear left track
    EDIBogieScanner    = 6, ///< The bogie or the scanner, depending on the case
    EDIFirstDrive      = EDIFrontRightTrack,
    EDILastDrive       = EDIBogieScanner,
    EDIFirstTrack      = EDIFrontRightTrack,
    EDILastTrack       = EDIRearLeftTrack
};

/** \enum  EScannerCommand
 *  \brief Enumeration to send commands to the scanner.
 */
enum EScannerCommand
{
    ESCDoHoming = 1, ///< Start the homing sequence
    ESCGoHome   = 2, ///< Go to the home position
    ESCStart    = 3, ///< Start the rolling process
    ESCStop     = 4  ///< Stop the rolling process
};

/** \enum  EScannerState
 *  \brief Enumeration to describe the internal state of the scanner.
 */
enum EScannerState
{
    ESSUnknown         = -1, ///< The scanner is in an unknown state
    ESSIdle            =  0, ///< The scanner is idle
    ESSCommandReceived =  1, ///< The scanner has received a new command, and will process it soon
    ESSHoming          =  2, ///< The scanner is executing the homing process
    ESSGoingHome       =  3, ///< The scanner is going to the home position
    ESSRolling         =  4  ///< The scanner is executing the rolloing process
};

/** \class RoverParams
 *  \brief Class to store the parameters of the rover.
 *         Some parameters (the non-const ones) may be changed at runtime by the
 *         user as a result of some calibration process.
 *         The positions are expressed relatively to the rover referential.
 */
class RoverParams
{
public:
    const float Length                             = 1.0670; ///< Length of the rover (including the scanner) [in m].
    const float Width                              = 0.8946; ///< Width of the rover [in m].

    const float BodyLength                         = 0.6970; ///< Length of the body of the rover [in m].
    const float BodyWidth                          = 0.6500; ///< Width of the body of the rover [in m].

    const float TrackPositionX                     = 0.2650; ///< X absolute position of a track [in m].
    const float TrackPositionY                     = 0.3970; ///< Y absolute position of a track (distance of its center relatively to the longitudinal axis of the rover) [in m].
    const float TrackPositionZ                     = 0.0500; ///< Z position of a track [in m].
    const float TrackFrontToRearDistance           = 0.5300; ///< Distance between front and rear tracks [in m].
    const float TrackLeftToRightDistance           = 0.7940; ///< Distance between left and right tracks [in m].
    float       TrackLeftToRightDistanceCalibrated = 1.0000; ///< Calibrated distance between left and right tracks [in m].
                                                             ///< This value is a "virtual" one resulting from calibration
                                                             ///< It is not a physical measure
    const float TrackLength                        = 0.5070; ///< Length of a track [in m].
    const float TrackWidth                         = 0.1016; ///< Width of a track [in m].
    float       TrackWheelRadius                   = 0.1220; ///< Track wheel radius [in m].
    const int   TrackEncoderResolution             = 9*4;    ///< Encoder resolution [in increments/motor revolution].
    const int   TrackGearRatio                     = 50;     ///< Motor to wheel gear ratio (the wheel goes slower).

    const float ScannerPositionX                   = 0.4932; ///< X position of the scanner [in m].
    const float ScannerPositionY                   = 0.0000; ///< Y position of the scanner [in m].
    const float ScannerPositionZ                   = 0.2520; ///< Z position of the scanner [in m].
    const int   ScannerEncoderResolution           = 128*4;  ///< Encoder resolution [in increments/motor revolution].
    const int   ScannerGearRatio                   = 8*86;   ///< Motor to scanner gear ratio (the scanner goes slower).
};

/** \class Rover
 *  \brief Class to control the rover.
 */
class Rover : protected CANopenMaster
{
public:
    /********************************************************************************
     * Rover initialization
     ********************************************************************************/

    /** \brief      Constructor for the instance of the rover.
     *  \param[in]  _logFrames: true to log the CAN frames in the console, false otherwise (default).
     */
    Rover(bool _logFrames = false);

    /** \brief      Destructor for the instance of the rover.
     */
    virtual ~Rover();

    /** \brief      Initialize the rover.
     *  \param[in]  _interfaceName: the name of the Linux network interface of the CAN bus.
     *  \return     0 if success, an error code otherwise.
     */
    virtual int init(const char* _interfaceName = "can0");

    /** \brief      Return the physical parameters of the rover. 
     *  \return     The instance of the rover parameters class.
     */
    RoverParams& getParams();

    /** \brief      Enable or disable frame logging.
     *  \param[in]  _logFrames: true to enable logging, false otherwise.
     *  \return     0 if success, an error code otherwise.
     */
    int setLogFrames(bool _logFrames = true);

    /********************************************************************************
     * CANopen synchronization frequency
     ********************************************************************************/

    /** \brief      Return the CANopen synchronization frequency.
     *  \param[out] _freq: the CANopen synchronization frequency [in Hz].
     *              Allowed range: [10.0 .. 200.0].
     *  \return     0 if success, an error code otherwise.
     */
    int getSyncFreq(float& _freq);

    /** \brief      Set the CANopen synchronization frequency.
     *  \param[in]  _freq: the desired CANopen synchronization frequency [in Hz].
     *              Allowed range: [10.0 .. 200.0].
     *  \return     0 if success, an error code otherwise.
     */
    int setSyncFreq(float _freq);

    /********************************************************************************
     * Timestamp
     ********************************************************************************/

    /** \brief      Return timestamp.
     *  \param[out] _timestamp: the current timestamp [in s].
     *  \return     0 if success, an error code otherwise.
     */
    int getTimestamp(Time& _timestamp);

    /********************************************************************************
     * Tracks enabling and disabling
     ********************************************************************************/

    /** \brief      Enable the tracks.
     *  \return     0 if success, an error code otherwise.
     *  \warning    This starts the control of the tracks.
     *              If the last speed that was set is not 0, the tracks will
     *              immediately try to reach this speed. Use with caution.
     */
    int enableTracks();

    /** \brief      Disable the tracks.
     *  \return     0 if success, an error code otherwise.
     *  \warning    This stops the control of the tracks: they become free.
     *              It is advised to set the speed to 0 before, and not to
     *              use this when on a non-horizontal ground. Use with caution.
     */
    int disableTracks();

    /********************************************************************************
     * Tracks and scanner current and voltage
     ********************************************************************************/

    /** \brief      Return the measure of the instantaneous current inside a drive.
     *  \param[in]  _driveId: the drive identifier.
     *  \param[out] _timestamp: the current timestamp [in s].
     *  \param[out] _current: the current [in A].
     *  \return     0 if success, an error code otherwise.
     */
    int getCurrent(EDriveId _driveId, Time& _timestamp, float& _current);

    /** \brief      Return the measure of the voltage of a drive.
     *              Note: this method sends a command and waits for its answer,
     *                    while 'get' methods just return values already received.
     *  \param[in]  _driveId: the drive identifier.
     *  \param[out] _timestamp: the current timestamp [in s].
     *  \param[out] _voltage: the voltage [in V].
     *  \return     0 if success, an error code otherwise.
     */
    int readVoltage(EDriveId _driveId, Time& _timestamp, float& _voltage);

    /********************************************************************************
     * Tracks and scanner status
     ********************************************************************************/

    /** \brief      Return the network state of the drive.
     *  \param[in]  _driveId: the drive identifier.
     *  \param[out] _state: the network state.
     *  \return     0 if success, an error code otherwise.
     */
    int getNetworkState(EDriveId _driveId, EDriveNetworkState& _state);

    /** \brief      Return the status of the track.
     *  \param[in]  _driveId: the drive identifier.
     *  \param[out] _statusWord: the track status word.
     *  \param[out] _status: the track status.
     *  \return     0 if success, an error code otherwise.
     */
    int getTrackStatus(EDriveId _driveId, uint16_t& _statusWord, EDriveStatus& _status);

    /********************************************************************************
     * Tracks and bogie angle
     ********************************************************************************/

    /** \brief      Return the angle corresponding to the horizontal position of the track/bogie.
     *  \param[in]  _driveId: the drive identifier.
     *  \param[out] _angle: the angle [in rad].
     *  \return     0 if success, an error code otherwise.
     */
    int getAngleReference(EDriveId _driveId, float& _angle);

    /** \brief      Set the angle corresponding to the horizontal position of the track/bogie.
     *  \param[in]  _driveId: the drive identifier.
     *  \param[out] _angle: the desired angle [in rad].
     *  \return     0 if success, an error code otherwise.
     */
    int setAngleReference(EDriveId _driveId, float _angle);

    /** \brief      Return the angle of the track/bogie.
     *  \param[in]  _driveId: the drive identifier.
     *  \param[out] _angle: the drive angle [in rad].
     *              It is expressed relatively to the corresponding angle reference.
     *  \return     0 if success, an error code otherwise.
     */
    int getAngle(EDriveId _driveId, float& _angle);

    /** \brief      Return the angles of the tracks and the bogie.
     *  \param[out] _timestamp: the current timestamp [in s].
     *  \param[out] _frontLeft: the angle of the front left track [in rad].
     *  \param[out] _frontRight: the angle of the front right track [in rad].
     *  \param[out] _rearLeft: the angle of the rear left track [in rad].
     *  \param[out] _rearRight: the angle of the rear right track [in rad].
     *  \param[out] _bogie: the angle of the bogie [in rad].
     *              They are expressed relatively to the corresponding angle reference.
     *  \return     0 if success, an error code otherwise.
     */
    int getAngles(Time& _timestamp, float& _frontLeft, float& _frontRight, float& _rearLeft, float& _rearRight, float& _bogie);

    /********************************************************************************
     * Tracks and scanner PI
     ********************************************************************************/

    /** \brief      Return the Kp and Ki of the PI of a drive.
     *  \param[in]  _driveId: the drive identifier.
     *  \param[out] _kp: the Kp of the PI.
     *  \param[out] _ki: the Ki of the PI.
     *  \return     0 if success, an error code otherwise.
     */
    int getPI(EDriveId _driveId, float& _kp, float& _ki);

    /** \brief      Set the Kp and Ki of the PI of a drive.
     *  \param[in]  _driveId: the drive identifier.
     *  \param[in]  _kp: the Kp of the PI.
     *  \param[in]  _ki: the Ki of the PI.
     *  \return     0 if success, an error code otherwise.
     */
    int setPI(EDriveId _driveId, float _kp, float _ki);

    /********************************************************************************
     * Scanner home position
     ********************************************************************************/

    /** \brief      Return the scanner factory set home position.
     *  \param[out] _pos: the home position [in rad].
     *              It is expressed relatively to the scanner rightmost position
     *              (i.e. _pos == 0 means at the rightmost position).
     *  \return     0 if success, an error code otherwise.
     */
    int getScannerHomePosition(float& _pos);

    /** \brief      Set the scanner factory set home position.
     *  \param[in]  _pos: the home position [in rad].
     *              It is expressed relatively to the scanner rightmost position
     *              (i.e. _pos == 0 means at the rightmost position).
     *  \return     0 if success, an error code otherwise.
     *  \warning    The final user of this library is advised to use
     *              setScannerAdjustment() to tune the home position.
     */
    int setScannerHomePosition(float _pos);

    /** \brief      Return the scanner adjustment.
     *  \param[out] _adj: the adjustment [in rad].
     *              It is expressed relatively to the scanner home position
     *              (_adj > 0 means at the right of the home position).
     *  \return     0 if success, an error code otherwise.
     */
    int getScannerAdjustment(float& _adj);

    /** \brief      Set the scanner adjustment.
     *  \param[in]  _adj: the adjustment [in rad].
     *              It is expressed relatively to the scanner home position
     *              (_adj > 0 means at the right of the home position).
     *  \return     0 if success, an error code otherwise.
     */
    int setScannerAdjustment(float _adj);

    /********************************************************************************
     * Scanner rolling
     ********************************************************************************/

    /** \brief      Return the scanner roll angle.
     *  \param[out] _angle: the roll angle [in rad].
     *  \return     0 if success, an error code otherwise.
     */
    int getScannerAngle(float& _angle);

    /** \brief      Set the scanner roll angle.
     *  \param[in]  _angle: the desired roll angle [in rad].
     *              Allowed range: ]0.0 .. +maxAngle]
     *              where maxAngle is the angle between the homing and rightmost positions.
     *              Values outside this range will replaced by Pi/2.
     *  \return     0 if success, an error code otherwise.
     */
    int setScannerAngle(float _angle);

    /** \brief      Return the scanner roll period.
     *  \param[out] _period: the roll period [in s].
     *  \return     0 if success, an error code otherwise.
     */
    int getScannerPeriod(float& _period);

    /** \brief      Set the scanner roll period.
     *  \param[in]  _period: the desired roll period [in s].
     *              Allowed range: ]1.0 .. +oo].
     *              Values below this range will replaced by 1.0.
     *              In pratice it seems that the lowest achievable period is something like 2.5s.
     *  \return     0 if success, an error code otherwise.
     */
    int setScannerPeriod(float _period);

    /** \brief      Return the current scanner state.
     *  \param[out] _state: the scanner state.
     *  \return     0 if success, an error code otherwise.
     */
    int getScannerState(EScannerState& _state);

    /** \brief      Send a command to the scanner.
     *              Notes: depending on the command, the state of the scanner should be appropriate.
     *                     GoHome can interrupt the rolling.
     *  \param[in]  _command: the desired scanner action.
     *              Allowed values: {ESADoHoming, ESAGoHome, ESAStart, ESEStop}.
     *  \return     0 if success, an error code otherwise.
     */
    int sendScannerCommand(EScannerCommand _command);

    /********************************************************************************
     * Tracks speed
     ********************************************************************************/

    /** \brief      Get the speed of the tracks [in rad/s].
     *  \param[out] _timestamp: the current timestamp [in s].
     *  \param[out] _frontLeft: the speed of the front left track [in rad/s].
     *  \param[out] _frontRight: the speed of the front right track [in rad/s].
     *  \param[out] _rearLeft: the speed of the rear left track [in rad/s].
     *  \param[out] _rearRight: the speed of the rear right track [in rad/s].
     *  \return     0 if success, an error code otherwise.
     */
    int getSpeed(Time& _timestamp, float& _frontLeft, float& _frontRight, float& _rearLeft, float& _rearRight);

    /** \brief      Set the speed of the tracks [in rad/s].
     *  \param[in]  _frontLeft: the desired speed of the front left track [in rad/s].
     *  \param[in]  _frontRight: the desired speed of the front right track [in rad/s].
     *  \param[in]  _rearLeft: the desired speed of the rear left track [in rad/s].
     *  \param[in]  _rearRight: the desired speed of the rear right track [in rad/s].
     *  \return     0 if success, an error code otherwise.
     */
    int setSpeed(float _frontLeft, float _frontRight, float _rearLeft, float _rearRight);

    /** \brief      Get the speed of the tracks [in rad/s].
     *  \param[out] _timestamp: the current timestamp [in s].
     *  \param[out] _left: the average speed of the front and rear left tracks [in rad/s].
     *  \param[out] _right: the average speed of the front and rear right tracks [in rad/s].
     *  \return     0 if success, an error code otherwise.
     */
    int getSpeed(Time& _timestamp, float& _left, float& _right);

    /** \brief      Set the speed of the tracks [in rad/s].
     *  \param[in]  _left: the desired speed of the front and rear left tracks [in rad/s].
     *  \param[in]  _right: the desired speed of the front and rear right tracks [in rad/s].
     *  \return     0 if success, an error code otherwise.
     */
    int setSpeed(float _left, float _right);

    /** \brief      Get the speed of the tracks [in m/s].
     *  \param[out] _timestamp: the current timestamp [in s].
     *  \param[out] _frontLeft: the speed of the front left track [in m/s].
     *  \param[out] _frontRight: the speed of the front right track [in m/s].
     *  \param[out] _rearLeft: the speed of the rear left track [in m/s].
     *  \param[out] _rearRight: the speed of the rear right track [in m/s].
     *  \return     0 if success, an error code otherwise.
     */
    int getSpeedInMPerS(Time& _timestamp, float& _frontLeft, float& _frontRight, float& _rearLeft, float& _rearRight);

    /** \brief      Set the speed of the tracks [in m/s].
     *  \param[in]  _frontLeft: the desired speed of the front left track [in m/s].
     *  \param[in]  _frontRight: the desired speed of the front right track [in m/s].
     *  \param[in]  _rearLeft: the desired speed of the rear left track [in m/s].
     *  \param[in]  _rearRight: the desired speed of the rear right track [in m/s].
     *  \return     0 if success, an error code otherwise.
     */
    int setSpeedInMPerS(float _frontLeft, float _frontRight, float _rearLeft, float _rearRight);

    /** \brief      Get the speed of the tracks [in m/s].
     *  \param[out] _timestamp: the current timestamp [in s].
     *  \param[out] _left: the average speed of the front and rear left tracks [in m/s].
     *  \param[out] _right: the average speed of the front and rear right tracks [in m/s].
     *  \return     0 if success, an error code otherwise.
     */
    int getSpeedInMPerS(Time& _timestamp, float& _left, float& _right);

    /** \brief      Set the speed of the tracks [in m/s].
     *  \param[in]  _left: the desired speed of the front and rear left tracks [in m/s].
     *  \param[in]  _right: the desired speed of the front and rear right tracks [in m/s].
     *  \return     0 if success, an error code otherwise.
     */
    int setSpeedInMPerS(float _left, float _right);

    /** \brief      Get the speed of the rover.
     *  \param[out] _timestamp: the current timestamp [in s].
     *  \param[out] _v: the tangential velocity [in m/s].
     *  \param[out] _omega: the radial velocity [in rad/s].
     *  \return     0 if success, an error code otherwise.
     */
    int getSpeedVO(Time& _timestamp, float& _v, float& _omega);

    /** \brief      Set the speed of the rover.
     *  \param[in]  _v: the desired tangential velocity [in m/s].
     *  \param[in]  _omega: the desired radial velocity [in rad/s].
     *  \return     0 if success, an error code otherwise.
     */
    int setSpeedVO(float _v, float _omega);

    /********************************************************************************
     * Tracks, bogie and scanner logging
     ********************************************************************************/

    void logDrivesData();

protected:
    Rover(const Rover& _rover) = delete;

    Rover& operator=(const Rover& _rover) = delete;

    RoverParams roverParams;

    pthread_mutex_t pdo2ResponseMapMutex;
    unordered_map<int, can_frame> pdo2ResponseMap;

    // Note: the data could be improved somehow with double-buffering
    atomic<bool> timestampNotUpdated;
    Time timestamp;
    Track frontRightTrack, frontLeftTrack, rearRightTrack, rearLeftTrack;
    BogieScanner bogieScanner;
    unordered_map<int, Drive*> driveIdToDrive;
    unordered_map<int, float> driveIdToAngleSign;
    float scannerAdjustmentSign;
    unordered_map<int, float> trackIdToSpeedSign;

    virtual int configureDrives();

    virtual int enableDrives();

    virtual int disableDrives();

    virtual int onSync();

    virtual int sendRxPdos();

    virtual int onFrameReceived(const can_frame& _frame);

    virtual string getFrameString(const can_frame& _frame);

    Track* driveIdToTrack(EDriveId _driveId);

    float getAnalogInput1ToRadRatio();

    float getTrackCountToRadRatio();

    float getRadToTrackCountRatio();

    float getTrackCountToMRatio();

    float getMToTrackCountRatio();

    int sendRxPdo1(uint8_t _driveId, int32_t _targetVelocity, uint16_t _controlWord);

    // Enumeration to specify binary interpreter commands sent to RxPdo2.
    enum ECommandType
    {
        ECTCmd = 0x00, // For example: BG or SV
        ECTGet = 0x01, // For example: get MO or KP[2]
        ECTSet = 0x02  // For example: set MO=1 or KP[2]=1000
    };

    /* Send get/set commands to the binary interpreter (type T should be int or float).
     * _inOutValue will systematically be updated.
     * Do not use this function inside 'configureDrives()' because we do not yet receive answers there
     */
    template <typename T>
    int sendRxPdo2(uint8_t _nodeId, ECommandType _cmdType, const char* _cmdName, uint16_t _index, T& _inOutValue)
    {
        T inValue = _inOutValue;
        _inOutValue = T();
        if (_nodeId > NodeIdMax ||
            ::strlen(_cmdName) != 2 || // All commands are 2 letters long
            (_index & 0xC000) != 0)    // Indexes are on 14 bits
        {
            return EINVAL;
        }
        uint8_t dataLen = 4;
        uint8_t data[8];
        data[0] = ::toupper(_cmdName[0]);
        data[1] = ::toupper(_cmdName[1]);
        uint16_t indexExt = _index | ((_cmdType&0x01)<<14) | (ECommandValue<T>::typeFlag<<15);
        valueToLSBBuffer<uint16_t>(indexExt, &data[2]);
        if (_cmdType == ECTSet)
        {
            dataLen += 4;
            valueToLSBBuffer<uint32_t>(*(uint32_t*)&inValue, &data[4]);
        }
        int retCode = sendRxPdo(2, _nodeId, dataLen, data);
        if (retCode != 0)
        {
            return retCode;
        }
        // Let us do like when waiting for SDO answers
        // Note: 'cmd' and 'set' commands return an answer too
        long remainingTime = SdoAnswerTotalWaitDuration;
        bool pdo2AnswerNotReceived = true;
        uint8_t pdo2AnswerData[4];
        while (remainingTime > 0 && pdo2AnswerNotReceived)
        {
            if (::pthread_mutex_trylock(&pdo2ResponseMapMutex) == 0)
            {
                unordered_map<int, can_frame>::iterator it = pdo2ResponseMap.find(_nodeId);
                if (it != pdo2ResponseMap.end())
                {
                    pdo2AnswerNotReceived = false;
                    ::memcpy(&pdo2AnswerData[0], &it->second.data[4], 4);
                    pdo2ResponseMap.erase(it);
                }
                ::pthread_mutex_unlock(&pdo2ResponseMapMutex);
            }
            Time::sleep(0, SdoAnswerTryWaitDuration);
            remainingTime -= SdoAnswerTryWaitDuration;
        }
        if (pdo2AnswerNotReceived)
        {
            return EINVAL;
        }
        uint32_t outValue = LSBBufferToValue<uint32_t>(&pdo2AnswerData[0]);
        _inOutValue = *(T*)&outValue;
        if (_cmdType == ECTSet && inValue != _inOutValue)
        {
            return EINVAL;
        }
        return 0;
    }

    /* Save the drive parameters to the flash.
     *  Do not use this method, it is for development only.
     */
    int saveDriveParametersToFlash(uint8_t _driveId);
};

#endif //LIB_ROVER_H

