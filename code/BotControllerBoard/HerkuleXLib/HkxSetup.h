#ifndef _HKXSETUP_H
#define _HKXSETUP_H

/**
 * \file HkxSetup.h
 * \brief Servo setup of the Arduino library for HerkuleX servo (DRS-0101 and DRS-0201 models)
 * \author Benoit Puel (contact@benoitpuel.com)
 * \version 1.0
 * \date 11 December 2015
 * \copyright This project is released under the GNU GPL V3.
 */
 
#include "Arduino.h"
#include "HkxCommunication.h"

/**
 * \class HkxSetup
 * \brief Class to setup the servos.
 * \details This class allows to setup the servos.
 */
class HkxSetup{
private:
  /** Communication with servos  */
  HkxCommunication& _herkXCom;
  /** Communication to print messages  */
  HkxPrint& _print;
  /** \cond developer */
  /** Class name to print the messages */
  const String _className;

  /**
   * \brief Print error
   * \details Print an error message if setup.
   * \param[in] message : content to print as an error.  
   */
  void errorPrint(const String& message){if(_print._errorMessages){_print.errorPrint(_className + message);}};

  /**
   * \brief Print warning
   * \details Print a warning message if setup.
   * \param[in] message : content to print as a warning.  
   */
  void warningPrint(const String& message){if(_print._errorMessages){_print.warningPrint(_className + message);}};

  /**
   * \brief Print info
   * \details Print an info message if setup.
   * \param[in] message : content to print as an info.  
   */
  void infoPrint(const String& message){if(_print._errorMessages){_print.infoPrint(_className + message);}};

  /**
   * \brief Set policies
   * \details Modify the current Alarm LED policy and Torque policy of the servo in both EEP and RAM. For easier definition the following constants can be used.
   * - \c HKX_STAT_VOLTAGE : Exceed Input Voltage limit
   * - \c HKX_STAT_POSITION : Exceed allowed POT limit
   * - \c HKX_STAT_TEMPERATURE : Exceed Temperature limit
   * - \c HKX_STAT_PACKET : Invalid Packet
   * - \c HKX_STAT_OVERLOAD : Overload detected
   * - \c HKX_STAT_DRIVER : Driver fault detected
   * - \c HKX_STAT_ROM_DISTORTED : EEP REG distorted
   * - \c HKX_STAT_ALL : all the list
   * \param[in] ID : current id of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \param[in] newAlarmLEDPolicy : Alarm LED policy. See the user manual p.33 for more details. \n It can be easily setup thanks the below constants. For instance, the setting  \c HKX_STAT_VOLTAGE & \c HKX_STAT_POSITION to start the LED alarm for any of this two errors.
   * \param[in] newTorquePolicy : Torque policy. See the user manual p.33 for more details. \n It can be easily setup thanks the below constants. For instance, the setting  \c HKX_STAT_TEMPERATURE & \c HKX_STAT_OVERLOAD & \c HKX_STAT_DRIVER to start the torque alarm for any of this three errors.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t setPolicies(uint8_t ID, uint8_t newAlarmLEDPolicy, uint8_t newTorquePolicy);

  /**
   * \brief Set temperature and voltage
   * \details Modify the max temperature, and min and max voltages of the servo in both EEP and RAM. 
   * \warning Consider the following obvious constraint: \c newMinVoltage < \c newMaxVoltage
   * \param[in] ID : current id of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \param[in] newMaxTemp : Maximum allowed temperature. Exceeding this temperature switch the servo to error. The value shall be set in 10^-2 degree within the range [-79.47° ; 300.91°].
   * \param[in] newMinVoltage : Minimum allowed voltage value. Under this input voltage, the servo switches to error. The value shall be set in millivolts within the range [0 ; 18.889V].
   * \param[in] newMaxVoltage : Maximum allowed voltage value. Over this input voltage, the servo switches to error. The value shall be set in millivolts within the range [0 ; 18.889V].
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setTemperatureVoltage(uint8_t ID, int16_t newMaxTemp, uint16_t newMinVoltage, uint16_t newMaxVoltage);

   /**
   * \brief Set Acceleration
   * \details Modify the acceleration parameters of the servo in both EEP and RAM. The servo generates a trapezoidal velocity trajectory profile (see figure below) according to play time given by the user (see for instance HkxPosControl.movePosition()) where: \c acceleration \c time = \c deceleration \c time = \c min (\c playtime  x \c accelerationRatio / \c 100, \c maxAccelerationTime). \verbatim
if (playtime x accelerationRatio / 100) < maxAccelerationTime   if (playtime x accelerationRatio / 100) > maxAccelerationTime
            V |                                                        V |          ______________                   
            e |         /*\                                            e |         /*            *\                      
            l |        / * \                                           l |        / *            * \                 
            o |       /  *  \                                          o |       /  *            *  \                
            c |      /   *   \                                         c |      /   *            *   \                   
            i |     /    *    \                                        i |     /    *            *    \                  
            t |    /     *     \                                       t |    /     *            *     \                 
            y |   /      *      \                                      y |   /      *            *      \                
              |  /       *       \                                       |  /       *            *       \               
              | /        *        \                                      | /        *            *        \              
              |/_________*_________\________ Time                        |/_________*____________*_________\________ Time
              < Acc time> <Dec time >                                    < Acc time >            < Dec time >            
              <----- Play time ----->                                    <----------- Play time ------------>            \endverbatim
   * \param[in] ID : current id of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \param[in] newAccelerationRatio : Acceleration ratio considered to generate velocity trajectory. Its value shall be set in percent (%) within the range [0 ; 50%].
   * \param[in] newMaxAccelerationTime : Maximum acceleration time to generate velocity trajectory. Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t setAcceleration(uint8_t ID, uint8_t newAccelerationRatio, uint16_t newMaxAccelerationTime);
  
  /**
   * \brief Set load control parameters
   * \details Modify the load control parameters of the servo in both EEP and RAM. The load control parameters define the saturation curve according to the figure below.\verbatim
          PWM+
           |  ___________........................................ PWM max
           |             \
           |              \ Saturator slope
           |               \..................................... Saturator offset
           |                |<dead zone> <dead zone>
           |                |___________ ........................ PWM min
           |                            |
           |  --------------------------|------------------------ PWM offset
           |                            |__________
 Position- |____________________________*__________|_____________ Position+
           |                          goal         |
           |                        position        \
           |                                         \__________
           |
          PWM- \endverbatim
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \param[in] newDeadZone : The dead zone is the angle error before the servo applies a load compensation to maintain its position (see the above figure). Its value shall be set in 10^-1 degrees within the range [0 ; 82.8°].
   * \param[in] newSaturatorOffset : The saturator offset is the step PWM value of the saturator once deadzone the position goes out of the dead zone (see the above figure). Its value shall be set in PWM within the range [0 ; 254].
   * \param[in] newSaturatorSlope : The saturator slop is a gradient of the saturator, going from the saturator offset and increasing gradually the PWM with the error position to the goal (see the above figure). It provides a flexible and elastic response of the servo to external forces. Its value shall be set in 10^-2 PWM / degree  within the range [0 ; 393 PWM/degree].
   * \param[in] newPWMOffset : The PWM offset allows compensating permanent load (e.g. gravity) by shifting the saturator by the same value (see the above figure). Its value shall be set in PWM within the range [-128 ; 127].
   * \param[in] newMinPWM : The min PWM is the minimum PWM that is applied by the servo (see the above figure). As specified in the user manual p.37, this may lead to unstable system. Use 0 as default value. The value shall be set in PWM within the range [0 ; 254].
   * \param[in] newMaxPWM : The max PWM is the maximum PWM that is applied by the servo (see the above figure). It could be used to limit the load of the servo, to avoid damages or injuries or instance. The value shall be set in PWM within the range [0 ; 1023].
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */  
  uint8_t setLoadControl(uint8_t ID, uint16_t newDeadZone, uint8_t newSaturatorOffset, const uint32_t& newSaturatorSlope, int8_t newPWMOffset, uint8_t newMinPWM, uint16_t newMaxPWM);

  /**
   * \brief Set PWM and position limits
   * \details Modify the PWM and position limits of the servo in both EEP and RAM. Over these defined limits, the servo is switching to error mode. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \param[in] newOverlaodPWMThreshold : The overload PWM threshold is the limits off PWM load from which the servo switches to error mode. Its value shall be set in PWM within the range [0 ; 1023 PWM]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] newMinPosition : The min position is the lower limit position (given compare to the neutral position = 512 raw) the servo is allowed to move before switching to error mode. It can be used to avoid collisions for instance. Its value shall be set in 10^-1 degrees within the range [-166.7° ; 166.7°]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] newMaxPosition : The max position is the upper limit position (given compare to the neutral position = 512 raw) the servo is allowed to move before switching to error mode. It can be used to avoid collisions for instance. Its value shall be set in 10^-1 degrees within the range [-166.7° ; 166.7°]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t setPWMPositionLimits(uint8_t ID, uint16_t newOverlaodPWMThreshold, int16_t newMinPosition, int16_t newMaxPosition);

  /**
   * \brief Set Control System
   * \details Modify the control system of the servo in both EEP and RAM. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \param[in] kProportionnal : Proportional gain, when its value increase, the response time decreases but may lead to over response (vibration, overshoot). Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] kDerivative : Derivative gain, when its value increase, the over response is attenuated but may lead to instability. Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] kInteger : Integral gain, when its value increase, reduce the position offset error in steady state  but may lead response lag. Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] feedforwardGain1 : Position feed forward 1st gain, when its value increase, the response time decreases. Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] feedforwardGain2 : Position feed forward 2nd gain, when its value increase, the response time decreases. Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setControlSystem(uint8_t ID, uint16_t kProportionnal, uint16_t kDerivative, uint16_t kInteger, uint16_t feedforwardGain1, uint16_t feedforwardGain2);

  /**
   * \brief Set check periods
   * \details Modify the check periods for de different errors of the servo in both EEP and RAM. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \param[in] LEDBlink : Check period for the alarm LED policy to activate LED blinking. Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \param[in] ADCFault : Check period for temperature and input voltage error (when temperature > max temperature or voltage < min voltage or voltage > max voltage). Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \param[in] packetGarbage : Check period for incomplete packet error. Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \param[in] stopDetection : Check period for the servo stoppage. Its value shall be set in milliseconds within the range [0 ; 2845ms]. 
   * \param[in] overloadDetection : Check period for the overload PWM threshold. Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setErrorsCheckPeriod(uint8_t ID, uint16_t LEDBlink, uint16_t ADCFault, uint16_t packetGarbage, uint16_t stopDetection, uint16_t overloadDetection);

  /**
   * \brief Set in position criteria
   * \details Modify the in position criteria of the servo in both EEP and RAM. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \param[in] stopThreshold : Under this angle displacement, the servo is seen as not moving. Its value shall be set in 10^-1 degrees within the range [0 ; 82.8°]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] inPositionMargin : Minimum angle distance to determine the goal position reached. Its value shall be set in 10^-1 degrees within the range [0 ; 82.8°]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setInPositionCriteria(uint8_t ID, uint16_t stopThreshold, uint16_t inPositionMargin);

  /**
   * \brief Set torque actuation and LED colour
   * \details Modify the torque actuation and LED colour of the servo in both EEP and RAM. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \param[in] newTorqueControl : Torque actuator to apply. Possible values are:
   * - \c HKX_TORQUE_FREE: no resistance to the movements (or let say mechanical resistance only)
   * - \c HKX_TORQUE_BREAK: the motor resists but allows the movements
   * - \c HKX_TORQUE_ON: the motor maintain its position (don't allow movements)
   * \param[in] newLEDControl : LED colour to set. Possible values are:
   * - \c HKX_LED_OFF,
   * - \c HKX_LED_GREEN,
   * - \c HKX_LED_BLUE,
   * - \c HKX_LED_RED,
   * - \c HKX_LED_CYAN,
   * - \c HKX_LED_YELLOW,
   * - \c HKX_LED_PINK,
   * - \c HKX_LED_WHITE,
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setTorqueLEDControl(uint8_t ID, hkxTorqueControl newTorqueControl,hkxLEDControl newLEDControl);

  /**
   * \brief Clear status
   * \details Clear the status of the servo to leave the "error state" and stop the LED blink and torque deactivation.
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t clearStatus(uint8_t ID);

  /**
   * \brief Factory reset
   * \details Reset all the parameters to factory default values.
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \param[in] IDskip : If \c true, the ID is NOT reset. If \c false, the ID is reset.
   * \param[in] baudrateSkip : If \c true, the baud rate is NOT reset. If \c false, the baud rate is reset.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t factoryReset(uint8_t ID, boolean IDskip, boolean baudrateSkip);

  /**
   * \brief Reboot
   * \details Reboot the servo.
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t reboot(uint8_t ID){
    if(ID > HKX_ID_ALL){
      errorPrint(F("reboot > Input not correct"));
    return 1;
    }
    return _herkXCom.rebootRequest(ID);
  };
  /** \endcond */  
public:  
  /**
   * \brief Constructor
   * \details The constructor of HkxSetup.
   * \param[in] herkXCom : Communication with the servos. 
   * \param[in] print : Communication to print messages.
   */
  HkxSetup(HkxCommunication& herkXCom, HkxPrint& print) : _herkXCom(herkXCom), _print(print), _className(F("HkxSetup::")){};
  
  /**
   * \brief Get model and firmware 
   * \details Get the model and the firmware version of the servo from EEP memory. 
   * \param[in] ID : id of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] model : Model of the servo (DRS-0101 returns 0x0101, DRS-0201 returns 0x0201). This parameter is optional, either set the address of \c uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] version : Version of the firmware. This parameter is optional, either set the address of \c uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   */
  uint8_t getServoInfo(uint8_t ID, HkxMaybe<uint16_t> model, HkxMaybe<uint16_t> version);

  /**
   * \brief Get the baud rate
   * \details Get the baud rate from the servo settings from EEP memory. 
   * \param[in] ID : id of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] baudRate : Baud rate setup in the servo. This parameter is a point, then set the address of \ref hkxBaudrate variable.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   */
  uint8_t getBaudRate(uint8_t ID, hkxBaudrate* baudRate); 

  /**
   * \brief Set the baud rate for all
   * \details Modify the baud rate for all the servos in EEP memory. A reboot of all the servos is performed at the same time the communication serial port is restarted with the new baud rate.
   * \warning From experience, the baud rate 57600 does NOT work properly. Please avoid using it.
   * \param[in] baudrate : Baud rate to apply to the servos and the communication port. The possible values are:
   * - \c HKX_57600  : 57600  bps
   * - \c HKX_115200 : 115200 bps
   * - \c HKX_200000 : 200000 bps
   * - \c HKX_250000 : 250000 bps
   * - \c HKX_400000 : 400000 bps
   * - \c HKX_500000 : 500000 bps
   * - \c HKX_666666 : 666666 bps
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t setAllBaudRate(const hkxBaudrate& baudrate); 

  /**
   * \brief Set ID
   * \details Modify the ID of the servo in both EEP and RAM. A check that no other servo already has the \c newID is performed before applying the modification.
   * \param[in] ID : current id of the servo to address the request. The value shall be [0 ; 253].
   * \param[in] newID : new id of the servo. The value shall be [0 ; 253].
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 4 = newID already used
   */
  uint8_t setID(uint8_t ID, uint8_t newID); 

  /**
   * \brief Get Policies
   * \details Get the current Alarm LED policy and Torque policy of the servo in the RAM. 
   * \param[in] ID : current id of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] newAlarmLEDPolicy : Alarm LED policy. See the user manual p.33 for more details. This parameter is optional, either set the address of \c uint8_t (or \c byte) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] newTorquePolicy : Torque policy. See the user manual p.33 for more details. This parameter is optional, either set the address of \c uint8_t (or \c byte) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   */
  uint8_t getPolicies(uint8_t ID, HkxMaybe<uint8_t> newAlarmLEDPolicy, HkxMaybe<uint8_t> newTorquePolicy);

  /**
   * \brief Set policies
   * \details Modify the current Alarm LED policy and Torque policy of the servo in both EEP and RAM. For easier definition the following constants can be used.
   * - \c HKX_STAT_VOLTAGE : Exceed Input Voltage limit
   * - \c HKX_STAT_POSITION : Exceed allowed POT limit
   * - \c HKX_STAT_TEMPERATURE : Exceed Temperature limit
   * - \c HKX_STAT_PACKET : Invalid Packet
   * - \c HKX_STAT_OVERLOAD : Overload detected
   * - \c HKX_STAT_DRIVER : Driver fault detected
   * - \c HKX_STAT_ROM_DISTORTED : EEP REG distorted
   * - \c HKX_STAT_ALL : all the list
   * \param[in] ID : current id of the servo to address the request. The value shall be [0 ; 253].
   * \param[in] newAlarmLEDPolicy : Alarm LED policy. See the user manual p.33 for more details. \n It can be easily setup thanks the below constants. For instance, the setting  \c HKX_STAT_VOLTAGE & \c HKX_STAT_POSITION to start the LED alarm for any of this two errors.
   * \param[in] newTorquePolicy : Torque policy. See the user manual p.33 for more details. \n It can be easily setup thanks the below constants. For instance, the setting  \c HKX_STAT_TEMPERATURE & \c HKX_STAT_OVERLOAD & \c HKX_STAT_DRIVER to start the torque alarm for any of this three errors.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t setOnePolicies(uint8_t ID, uint8_t newAlarmLEDPolicy, uint8_t newTorquePolicy){
    if(ID > HKX_MAX_ID){
      errorPrint(F("setOnePolicies > Input not correct"));
      return 1;
    }
    return setPolicies(ID, newAlarmLEDPolicy, newTorquePolicy);
  };

  /**
   * \brief Set policies for all
   * \details Modify the current Alarm LED policy and Torque policy of all servos in both EEP and RAM. For easier definition the following constants can be used.
   * - \c HKX_STAT_VOLTAGE : Exceed Input Voltage limit
   * - \c HKX_STAT_POSITION : Exceed allowed POT limit
   * - \c HKX_STAT_TEMPERATURE : Exceed Temperature limit
   * - \c HKX_STAT_PACKET : Invalid Packet
   * - \c HKX_STAT_OVERLOAD : Overload detected
   * - \c HKX_STAT_DRIVER : Driver fault detected
   * - \c HKX_STAT_ROM_DISTORTED : EEP REG distorted
   * - \c HKX_STAT_ALL : all the list
   * \param[in] newAlarmLEDPolicy : Alarm LED policy. See the user manual p.33 for more details. \n It can be easily setup thanks the below constants. For instance, the setting  \c HKX_STAT_VOLTAGE & \c HKX_STAT_POSITION to start the LED alarm for any of this two errors.
   * \param[in] newTorquePolicy : Torque policy. See the user manual p.33 for more details. \n It can be easily setup thanks the below constants. For instance, the setting  \c HKX_STAT_TEMPERATURE & \c HKX_STAT_OVERLOAD & \c HKX_STAT_DRIVER to start the torque alarm for any of this three errors.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t setAllPolicies(uint8_t newAlarmLEDPolicy, uint8_t newTorquePolicy){return setPolicies(HKX_ID_ALL, newAlarmLEDPolicy, newTorquePolicy);};

  /**
   * \brief Get temperature and voltage
   * \details Get the current max temperature, and min and max voltages of the servo in the RAM. 
   * \param[in] ID : current id of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] maxTemp : Maximum allowed temperature. Exceeding this temperature switch the servo to error. This parameter is optional, either set the address of \c uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] minVoltage : Minimum allowed voltage value. Under this input voltage, the servo switches to error. This parameter is optional, either set the address of \c uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] maxVoltage : Maximum allowed voltage value. Over this input voltage, the servo switches to error. This parameter is optional, either set the address of \c uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   */
  uint8_t getTemperatureVoltage(uint8_t ID, HkxMaybe<int16_t> maxTemp, HkxMaybe<uint16_t> minVoltage, HkxMaybe<uint16_t> maxVoltage);

  /**
   * \brief Set temperature and voltage
   * \details Modify the max temperature, and min and max voltages of the servo in both EEP and RAM. 
   * \param[in] ID : current id of the servo to address the request. The value shall be [0 ; 253].
   * \param[in] newMaxTemp : Maximum allowed temperature. Exceeding this temperature switch the servo to error. 
   * \param[in] newMinVoltage : Minimum allowed voltage value. Under this input voltage, the servo switches to error. 
   * \param[in] newMaxVoltage : Maximum allowed voltage value. Over this input voltage, the servo switches to error.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setOneTemperatureVoltage(uint8_t ID, int16_t newMaxTemp, uint16_t newMinVoltage, uint16_t newMaxVoltage){
    if(ID > HKX_MAX_ID){
      errorPrint(F("setOneMaxTemperature > Input value not correct"));
      return 1;
    }
    return setTemperatureVoltage(ID, newMaxTemp, newMinVoltage, newMaxVoltage);
  };

  /**
   * \brief Set temperature and voltage for all
   * \details Modify the max temperature, and min and max voltages of all servos in both EEP and RAM. 
   * \param[in] newMaxTemp : Maximum allowed temperature. Exceeding this temperature switch the servo to error. 
   * \param[in] newMinVoltage : Minimum allowed voltage value. Under this input voltage, the servo switches to error. 
   * \param[in] newMaxVoltage : Maximum allowed voltage value. Over this input voltage, the servo switches to error.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setAllTemperatureVoltage(int16_t newMaxTemp, int16_t newMinVoltage, int16_t newMaxVoltage){return setTemperatureVoltage(HKX_ID_ALL, newMaxTemp, newMinVoltage, newMaxVoltage);};

  /**
   * \brief Get Acceleration
   * \details Get the current acceleration parameters of the servo in the RAM. The servo generates a trapezoidal velocity trajectory profile (see figure below) according to play time given by the user (see for instance HkxPosControl.movePosition()) where: \c acceleration \c time = \c deceleration \c time = \c min (\c playtime  x \c accelerationRatio / \c 100, \c maxAccelerationTime). \verbatim
if (playtime x accelerationRatio / 100) < maxAccelerationTime   if (playtime x accelerationRatio / 100) > maxAccelerationTime
            V |                                                        V |          ______________                   
            e |         /*\                                            e |         /*            *\                      
            l |        / * \                                           l |        / *            * \                 
            o |       /  *  \                                          o |       /  *            *  \                
            c |      /   *   \                                         c |      /   *            *   \                   
            i |     /    *    \                                        i |     /    *            *    \                  
            t |    /     *     \                                       t |    /     *            *     \                 
            y |   /      *      \                                      y |   /      *            *      \                
              |  /       *       \                                       |  /       *            *       \               
              | /        *        \                                      | /        *            *        \              
              |/_________*_________\________ Time                        |/_________*____________*_________\________ Time
              < Acc time> <Dec time >                                    < Acc time >            < Dec time >            
              <----- Play time ----->                                    <----------- Play time ------------>            \endverbatim
   * \param[in] ID : id of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] accelerationRatio : Acceleration ratio considered to generate velocity trajectory. Its value is given in percent (%). This parameter is optional, either set the address of \c uint8_t (or \c byte) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] maxAccelerationTime : Maximum acceleration time to generate velocity trajectory. Its value is given in milliseconds. This parameter is optional, either set the address of \c uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   */
  uint8_t getAcceleration(uint8_t ID, HkxMaybe<uint8_t> accelerationRatio, HkxMaybe<uint16_t> maxAccelerationTime);
 
   /**
   * \brief Set Acceleration
   * \details Modify the acceleration parameters of the servo in both EEP and RAM. The servo generates a trapezoidal velocity trajectory profile (see figure below) according to play time given by the user (see for instance HkxPosControl.movePosition()) where: \c acceleration \c time = \c deceleration \c time = \c min (\c playtime  x \c accelerationRatio / \c 100, \c maxAccelerationTime). \verbatim
if (playtime x accelerationRatio / 100) < maxAccelerationTime   if (playtime x accelerationRatio / 100) > maxAccelerationTime
            V |                                                        V |          ______________                   
            e |         /*\                                            e |         /*            *\                      
            l |        / * \                                           l |        / *            * \                 
            o |       /  *  \                                          o |       /  *            *  \                
            c |      /   *   \                                         c |      /   *            *   \                   
            i |     /    *    \                                        i |     /    *            *    \                  
            t |    /     *     \                                       t |    /     *            *     \                 
            y |   /      *      \                                      y |   /      *            *      \                
              |  /       *       \                                       |  /       *            *       \               
              | /        *        \                                      | /        *            *        \              
              |/_________*_________\________ Time                        |/_________*____________*_________\________ Time
              < Acc time> <Dec time >                                    < Acc time >            < Dec time >            
              <----- Play time ----->                                    <----------- Play time ------------>            \endverbatim
   * \param[in] ID : id of the servo to address the request. The value shall be [0 ; 253]. 
   * \param[in] newAccelerationRatio : Acceleration ratio considered to generate velocity trajectory. Its value shall be set in percent (%) within the range [0 ; 50%].
   * \param[in] newMaxAccelerationTime : Maximum acceleration time to generate velocity trajectory. Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   */  
  uint8_t setOneAcceleration(uint8_t ID, uint8_t newAccelerationRatio, uint16_t newMaxAccelerationTime){
    if(ID > HKX_MAX_ID){
      errorPrint(F("setOneAcceleration > acceleration ratio value or max acceleration time not correct"));
      return 1;
    }
    return setAcceleration(ID, newAccelerationRatio, newMaxAccelerationTime);
  };

  /**
   * \brief Set Acceleration for all
   * \details Modify the acceleration parameters of the servo in both EEP and RAM. The servo generates a trapezoidal velocity trajectory profile (see figure below) according to play time given by the user (see for instance HkxPosControl.movePosition()) where: \c acceleration \c time = \c deceleration \c time = \c min (\c playtime  x \c accelerationRatio / \c 100, \c maxAccelerationTime). \verbatim
if (playtime x accelerationRatio / 100) < maxAccelerationTime   if (playtime x accelerationRatio / 100) > maxAccelerationTime
            V |                                                        V |          ______________                   
            e |         /*\                                            e |         /*            *\                      
            l |        / * \                                           l |        / *            * \                 
            o |       /  *  \                                          o |       /  *            *  \                
            c |      /   *   \                                         c |      /   *            *   \                   
            i |     /    *    \                                        i |     /    *            *    \                  
            t |    /     *     \                                       t |    /     *            *     \                 
            y |   /      *      \                                      y |   /      *            *      \                
              |  /       *       \                                       |  /       *            *       \               
              | /        *        \                                      | /        *            *        \              
              |/_________*_________\________ Time                        |/_________*____________*_________\________ Time
              < Acc time> <Dec time >                                    < Acc time >            < Dec time >            
              <----- Play time ----->                                    <----------- Play time ------------>            \endverbatim
   * \param[in] newAccelerationRatio : Acceleration ratio considered to generate velocity trajectory. Its value shall be set in percent (%) within the range [0 ; 50%].
   * \param[in] newMaxAccelerationTime : Maximum acceleration time to generate velocity trajectory. Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */   
  uint8_t setAllAcceleration(uint8_t newAccelerationRatio, uint16_t newMaxAccelerationTime){return setAcceleration(HKX_ID_ALL, newAccelerationRatio, newMaxAccelerationTime);};
  
  /**
   * \brief Get load control
   * \details Get the current load control parameters of the servo in the RAM. The load control parameters define the saturation curve according to the figure below. \verbatim
          PWM+
           |  ___________........................................ PWM max
           |             \
           |              \ Saturator slope
           |               \..................................... Saturator offset
           |                |<dead zone> <dead zone>
           |                |___________ ........................ PWM min
           |                            |
           |  --------------------------|------------------------ PWM offset
           |                            |__________
 Position- |____________________________*__________|_____________ Position+
           |                          goal         |
           |                        position        \
           |                                         \__________
           |
          PWM- \endverbatim
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] deadZone : The dead zone is the angle error before the servo applies a load compensation to maintain its position (see the figure above). Its value is given in 10^-1 degrees. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] saturatorOffSet : The saturator offset is the step PWM value of the saturator once deadzone the position goes out of the dead zone (see the figure above). Its value is given set in PWM. This parameter is optional, either set the address of \c uint8_t (or \c byte) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] saturatorSlope : The saturator slop is a gradient of the saturator, going from the saturator offset and increasing gradually the PWM with the error position to the goal (see the above figure). It provides a flexible and elastic response of the servo to external forces. Its value is given in 10^-2 PWM / degree. This parameter is optional, either set the address of \c uint32_t (or \c unsigned \c long) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] PWMoffset : The PWM offset allows compensating permanent load (e.g. gravity) by shifting the saturator by the same value (see the above figure). Its value is given in PWM. This parameter is optional, either set the address of \c uint8_t (or \c byte) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] minPWM : The min PWM is the minimum PWM that is applied by the servo (see the above figure). As specified in the user manual p.37, this may lead to unstable system. The value is given in PWM. This parameter is optional, either set the address of \c uint8_t (or \c byte) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] maxPWM : The max PWM is the maximum PWM that is applied by the servo (see the above figure). It could be used to limit the load of the servo, to avoid damages or injuries for instance. The value is given in PWM. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   */
  uint8_t getLoadControl(uint8_t ID, HkxMaybe<uint16_t> deadZone, HkxMaybe<uint8_t> saturatorOffSet, HkxMaybe<uint32_t> saturatorSlope, HkxMaybe<int8_t> PWMoffset, HkxMaybe<uint8_t> minPWM, HkxMaybe<uint16_t> maxPWM);

  /**
   * \brief Set load control parameters
   * \details Modify the load control parameters of the servo in both EEP and RAM. The load control parameters define the saturation curve according to the figure below.\verbatim
          PWM+
           |  ___________........................................ PWM max
           |             \
           |              \ Saturator slope
           |               \..................................... Saturator offset
           |                |<dead zone> <dead zone>
           |                |___________ ........................ PWM min
           |                            |
           |  --------------------------|------------------------ PWM offset
           |                            |__________
 Position- |____________________________*__________|_____________ Position+
           |                          goal         |
           |                        position        \
           |                                         \__________
           |
          PWM- \endverbatim
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[in] newDeadZone : The dead zone is the angle error before the servo applies a load compensation to maintain its position (see the above figure). Its value shall be set in 10^-1 degrees within the range [0 ; 82.8°].
   * \param[in] newSaturatorOffset : The saturator offset is the step PWM value of the saturator once deadzone the position goes out of the dead zone (see the above figure). Its value shall be set in PWM within the range [0 ; 254].
   * \param[in] newSaturatorSlope : The saturator slop is a gradient of the saturator, going from the saturator offset and increasing gradually the PWM with the error position to the goal (see the above figure). It provides a flexible and elastic response of the servo to external forces. Its value shall be set in 10^-2 PWM / degree  within the range [0 ; 393 PWM/degree].
   * \param[in] newPWMOffset : The PWM offset allows compensating permanent load (e.g. gravity) by shifting the saturator by the same value (see the above figure). Its value shall be set in PWM within the range [-128 ; 127].
   * \param[in] newMinPWM : The min PWM is the minimum PWM that is applied by the servo (see the above figure). As specified in the user manual p.37, this may lead to unstable system. Use 0 as default value. The value shall be set in PWM within the range [0 ; 254].
   * \param[in] newMaxPWM : The max PWM is the maximum PWM that is applied by the servo (see the above figure). It could be used to limit the load of the servo, to avoid damages or injuries or instance. The value shall be set in PWM within the range [0 ; 1023].
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */ 
  uint8_t setOneLoadControl(uint8_t ID, uint16_t newDeadZone, uint8_t newSaturatorOffset, const uint32_t& newSaturatorSlope, int8_t newPWMOffset, uint8_t newMinPWM, uint16_t newMaxPWM){
    if(ID > HKX_MAX_ID){
      errorPrint(F("setOneLoadControl > Input not correct"));
      return 1;
    }
    return setLoadControl(ID, newDeadZone, newSaturatorOffset, newSaturatorSlope, newPWMOffset, newMinPWM, newMaxPWM);
  };

  /**
   * \brief Set load control parameters for all
   * \details Modify the load control parameters of the servo in both EEP and RAM. The load control parameters define the saturation curve according to the figure below.\verbatim
          PWM+
           |  ___________........................................ PWM max
           |             \
           |              \ Saturator slope
           |               \..................................... Saturator offset
           |                |<dead zone> <dead zone>
           |                |___________ ........................ PWM min
           |                            |
           |  --------------------------|------------------------ PWM offset
           |                            |__________
 Position- |____________________________*__________|_____________ Position+
           |                          goal         |
           |                        position        \
           |                                         \__________
           |
          PWM- \endverbatim
   * \param[in] newDeadZone : The dead zone is the angle error before the servo applies a load compensation to maintain its position (see the above figure). Its value shall be set in 10^-1 degrees within the range [0 ; 82.8°].
   * \param[in] newSaturatorOffset : The saturator offset is the step PWM value of the saturator once dead zone the position goes out of the dead zone (see the above figure). Its value shall be set in PWM within the range [0 ; 254].
   * \param[in] newSaturatorSlope : The saturator slop is a gradient of the saturator, going from the saturator offset and increasing gradually the PWM with the error position to the goal (see the above figure). It provides a flexible and elastic response of the servo to external forces. Its value shall be set in 10^-2 PWM / degree  within the range [0 ; 393 PWM/degree].
   * \param[in] newPWMOffset : The PWM offset allows compensating permanent load (e.g. gravity) by shifting the saturator by the same value (see the above figure). Its value shall be set in PWM within the range [-128 ; 127].
   * \param[in] newMinPWM : The min PWM is the minimum PWM that is applied by the servo (see the above figure). As specified in the user manual p.37, this may lead to unstable system. Use 0 as default value. The value shall be set in PWM within the range [0 ; 254].
   * \param[in] newMaxPWM : The max PWM is the maximum PWM that is applied by the servo (see the above figure). It could be used to limit the load of the servo, to avoid damages or injuries or instance. The value shall be set in PWM within the range [0 ; 1023].
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setAllLoadControl(uint16_t newDeadZone, uint8_t newSaturatorOffset, const uint32_t& newSaturatorSlope, int8_t newPWMOffset, uint8_t newMinPWM, uint16_t newMaxPWM){return setLoadControl(HKX_ID_ALL, newDeadZone, newSaturatorOffset, newSaturatorSlope, newPWMOffset, newMinPWM, newMaxPWM);};

  /**
   * \brief Get PWM and position limits
   * \details Get the current PWM and position limits of the servo in the RAM. Over these defined limits, the servo is switching to error mode. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] overlaodPWMThreshold : The overload PWM threshold is the limits off PWM load from which the servo switches to error mode. Its value is given in PWM. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] minPosition : The min position is the lower limit position (given compare to the neutral position = 512 raw) the servo is allowed to move before switching to error mode. It can be used to avoid collisions for instance. Its value is given set in 10^-1 degrees. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] maxPosition : The max position is the upper limit position (given compare to the neutral position = 512 raw) the servo is allowed to move before switching to error mode. It can be used to avoid collisions for instance. Its value is given set in 10^-1 degrees. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   */
  uint8_t getPWMPositionLimits(uint8_t ID, HkxMaybe<uint16_t> overlaodPWMThreshold, HkxMaybe<int16_t> minPosition, HkxMaybe<int16_t> maxPosition);

  /**
   * \brief Set PWM and position limits
   * \details Modify the PWM and position limits of the servo both EEP and RAM. Over these defined limits, the servo is switching to error mode. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] newOverlaodPWMThreshold : The overload PWM threshold is the limits off PWM load from which the servo switches to error mode. Its value is given in PWM within the range [0 ; 1023 PWM]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] newMinPosition : The min position is the lower limit position (given compare to the neutral position = 512 raw) the servo is allowed to move before switching to error mode. It can be used to avoid collisions for instance. Its value is given set in 10^-1 degrees within the range [-166.7° ; 166.7°]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] newMaxPosition : The max position is the upper limit position (given compare to the neutral position = 512 raw) the servo is allowed to move before switching to error mode. It can be used to avoid collisions for instance. Its value is given set in 10^-1 degrees within the range [-166.7° ; 166.7°]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t setOnePWMPositionLimits(uint8_t ID, uint16_t newOverlaodPWMThreshold, int16_t newMinPosition, int16_t newMaxPosition){
    if(ID > HKX_MAX_ID){
      errorPrint(F("setOnePWMPositionLimits > Input not correct"));
      return 1;
    }
    return setPWMPositionLimits(ID, newOverlaodPWMThreshold, newMinPosition, newMaxPosition);
  }; 

  /**
   * \brief Set PWM and position limits for all
   * \details Modify the PWM and position limits of the servo both EEP and RAM. Over these defined limits, the servo is switching to error mode. 
   * \param[out] newOverlaodPWMThreshold : The overload PWM threshold is the limits off PWM load from which the servo switches to error mode. Its value is given in PWM within the range [0 ; 1023 PWM]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] newMinPosition : The min position is the lower limit position (given compare to the neutral position = 512 raw) the servo is allowed to move before switching to error mode. It can be used to avoid collisions for instance. Its value is given in 10^-1 degrees within the range [-166.7° ; 166.7°]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] newMaxPosition : The max position is the upper limit position (given compare to the neutral position = 512 raw) the servo is allowed to move before switching to error mode. It can be used to avoid collisions for instance. Its value is given in 10^-1 degrees within the range [-166.7° ; 166.7°]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t setAllPWMPositionLimits(uint16_t newOverlaodPWMThreshold, int16_t newMinPosition, int16_t newMaxPosition){return setPWMPositionLimits(HKX_ID_ALL, newOverlaodPWMThreshold, newMinPosition, newMaxPosition);};

  /**
   * \brief Get Control System
   * \details Get the current control system of the servo in the RAM. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] kProportionnal : Proportional gain, when its value increase, the response time decreases but may lead to over response (vibration, overshoot). Its value is given with no unit. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] kDerivative : Derivative gain, when its value increase, the over response is attenuated but may lead to instability. Its value is given with no unit. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] kInteger : Integral gain, when its value increase, reduce the position offset error in steady state  but may lead response lag. Its value is given with no unit. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] feedforwardGain1 : Position feed forward 1st gain, when its value increase, the response time decreases. Its value is given with no unit. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] feedforwardGain2 : Position feed forward 2nd gain, when its value increase, the response time decreases. Its value is given with no unit. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   */
  uint8_t getControlSystem(uint8_t ID, HkxMaybe<uint16_t> kProportionnal, HkxMaybe<uint16_t> kDerivative, HkxMaybe<uint16_t> kInteger, HkxMaybe<uint16_t> feedforwardGain1, HkxMaybe<uint16_t> feedforwardGain2);

  /**
   * \brief Set Control System
   * \details Modify the control system of the servo in both EEP and RAM. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[in] kProportionnal : Proportional gain, when its value increase, the response time decreases but may lead to over response (vibration, overshoot). Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] kDerivative : Derivative gain, when its value increase, the over response is attenuated but may lead to instability. Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] kInteger : Integral gain, when its value increase, reduce the position offset error in steady state  but may lead response lag. Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] feedforwardGain1 : Position feed forward 1st gain, when its value increase, the response time decreases. Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] feedforwardGain2 : Position feed forward 2nd gain, when its value increase, the response time decreases. Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setOneControlSystem(uint8_t ID, uint16_t kProportionnal, uint16_t kDerivative, uint16_t kInteger, uint16_t feedforwardGain1, uint16_t feedforwardGain2){
    if(ID > HKX_MAX_ID){
      errorPrint(F("setOneControlSystem > Input not correct"));
      return 1;
    }
    return setControlSystem(ID, kProportionnal, kDerivative, kInteger, feedforwardGain1, feedforwardGain2);
  };

  /**
   * \brief Set Control System for all
   * \details Modify the control system of the servo in both EEP and RAM. 
   * \param[in] kProportionnal : Proportional gain, when its value increase, the response time decreases but may lead to over response (vibration, overshoot). Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] kDerivative : Derivative gain, when its value increase, the over response is attenuated but may lead to instability. Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] kInteger : Integral gain, when its value increase, reduce the position offset error in steady state  but may lead response lag. Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] feedforwardGain1 : Position feed forward 1st gain, when its value increase, the response time decreases. Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] feedforwardGain2 : Position feed forward 2nd gain, when its value increase, the response time decreases. Its value shall be set with no unit within the range [0 ; 32767]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setAllControlSystem(uint16_t kProportionnal, uint16_t kDerivative, uint16_t kInteger, uint16_t feedforwardGain1, uint16_t feedforwardGain2){return setControlSystem(HKX_ID_ALL, kProportionnal, kDerivative, kInteger, feedforwardGain1, feedforwardGain2);};

  /**
   * \brief Get check periods
   * \details Get the current check periods for de different errors of the servo in the RAM. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] LEDBlink : Check period for the alarm LED policy to activate LED blinking. Its value is given in milliseconds. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] ADCFault : Check period for temperature and input voltage error (when temperature > max temperature or voltage < min voltage or voltage > max voltage). Its value is given in milliseconds. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] packetGarbage : Check period for incomplete packet error. Its value is given in milliseconds. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] stopDetection : Check period for the servo stoppage. Its value is given in milliseconds. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] overloadDetection : Check period for the overload PWM threshold. Its value is given in milliseconds. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   */
  uint8_t getErrorsCheckPeriod(uint8_t ID, HkxMaybe<uint16_t> LEDBlink, HkxMaybe<uint16_t> ADCFault, HkxMaybe<uint16_t> packetGarbage, HkxMaybe<uint16_t> stopDetection, HkxMaybe<uint16_t> overloadDetection);

  /**
   * \brief Set check periods
   * \details Modify the check periods for de different errors of the servo in both EEP and RAM. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[in] LEDBlink : Check period for the alarm LED policy to activate LED blinking. Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \param[in] ADCFault : Check period for temperature and input voltage error (when temperature > max temperature or voltage < min voltage or voltage > max voltage). Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \param[in] packetGarbage : Check period for incomplete packet error. Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \param[out] stopDetection : Check period for the servo stoppage. Its value shall be set in milliseconds within the range [0 ; 2845ms]. 
   * \param[in] overloadDetection : Check period for the overload PWM threshold. Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setOneErrorsCheckPeriod(uint8_t ID, uint16_t LEDBlink, uint16_t ADCFault, uint16_t packetGarbage, uint16_t stopDetection, uint16_t overloadDetection){
    if(ID > HKX_MAX_ID){
      errorPrint(F("setOneErrorsCheckPeriod > Input not correct"));
      return 1;
    }
    return setErrorsCheckPeriod(ID, LEDBlink, ADCFault, packetGarbage, stopDetection, overloadDetection);
  };

  /**
   * \brief Set check periods for all
   * \details Modify the check periods for de different errors of the servo in both EEP and RAM. 
   * \param[in] LEDBlink : Check period for the alarm LED policy to activate LED blinking. Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \param[in] ADCFault : Check period for temperature and input voltage error (when temperature > max temperature or voltage < min voltage or voltage > max voltage). Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \param[in] packetGarbage : Check period for incomplete packet error. Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \param[out] stopDetection : Check period for the servo stoppage. Its value shall be set in milliseconds within the range [0 ; 2845ms]. 
   * \param[in] overloadDetection : Check period for the overload PWM threshold. Its value shall be set in milliseconds within the range [0 ; 2845ms].
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setAllErrorsCheckPeriod(uint16_t LEDBlink, uint16_t ADCFault, uint16_t packetGarbage, uint16_t stopDetection, uint16_t overloadDetection){return setErrorsCheckPeriod(HKX_ID_ALL, LEDBlink, ADCFault, packetGarbage, stopDetection, overloadDetection);};

  /**
   * \brief Get in position criteria
   * \details Get the current in position criteria of the servo in the RAM. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] stopThreshold : Under this angle displacement, the servo is seen as not moving. Its value is given in 10^-1 degrees. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] inPositionMargin : Minimum angle distance to determine the goal position reached. Its value is given in 10^-1 degrees. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   */
  uint8_t getInPositionCriteria(uint8_t ID, HkxMaybe<uint16_t> stopThreshold, HkxMaybe<uint16_t> inPositionMargin);

  /**
   * \brief Set in position criteria
   * \details Modify the in position criteria of the servo in both EEP and RAM. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] stopThreshold : Under this angle displacement, the servo is seen as not moving. Its value shall be set in 10^-1 degrees within the range [0 ; 82.8°]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] inPositionMargin : Minimum angle distance to determine the goal position reached. Its value shall be set in 10^-1 degrees within the range [0 ; 82.8°]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setOneInPositionCriteria(uint8_t ID, uint16_t stopThreshold, uint16_t inPositionMargin){
    if(ID > HKX_MAX_ID){
      errorPrint(F("setOneInPositionCriteria > Input not correct"));
      return 1;
    }
    return setInPositionCriteria(ID, stopThreshold, inPositionMargin);
  };

  /**
   * \brief Set in position criteria for all
   * \details Modify the in position criteria of the servo in both EEP and RAM. 
   * \param[in] stopThreshold : Under this angle displacement, the servo is seen as not moving. Its value shall be set in 10^-1 degrees within the range [0 ; 82.8°]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[in] inPositionMargin : Minimum angle distance to determine the goal position reached. Its value shall be set in 10^-1 degrees within the range [0 ; 82.8°]. This parameter is optional, either set the address of uint16_t (or \c unsigned \c int) variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setAllInPositionCriteria(uint16_t stopThreshold, uint16_t inPositionMargin){return setInPositionCriteria(HKX_ID_ALL, stopThreshold, inPositionMargin);};

  /**
   * \brief Get torque actuation and LED colour
   * \details Get the current torque actuation and LED colour of the servo in the RAM. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] torqueControl : Torque actuator. This parameter is optional, either set the address of \ref hkxTorqueControl variable, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] LEDControl : LED colour. This parameter is optional, either set the address of \ref hkxLEDControl variable, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   */
  uint8_t getTorqueLEDControl(uint8_t ID, HkxMaybe<hkxTorqueControl> torqueControl, HkxMaybe<hkxLEDControl> LEDControl);

  /**
   * \brief Set torque actuation and LED colour
   * \details Modify the torque actuation and LED colour of the servo in both EEP and RAM. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[in] newTorqueControl : Torque actuator to apply. Possible values are:
   * - \c HKX_TORQUE_FREE: no resistance to the movements (or let say mechanical resistance only)
   * - \c HKX_TORQUE_BREAK: the motor resists but allows the movements
   * - \c HKX_TORQUE_ON: the motor maintain its position (don't allow movements)
   * \param[in] newLEDControl : LED colour to set. Possible values are:
   * - \c HKX_LED_OFF,
   * - \c HKX_LED_GREEN,
   * - \c HKX_LED_BLUE,
   * - \c HKX_LED_RED,
   * - \c HKX_LED_CYAN,
   * - \c HKX_LED_YELLOW,
   * - \c HKX_LED_PINK,
   * - \c HKX_LED_WHITE,
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setOneTorqueLEDControl(uint8_t ID, hkxTorqueControl newTorqueControl, hkxLEDControl newLEDControl){
    if(ID > HKX_MAX_ID){
      errorPrint(F("setOneTorqueLEDControl > Input not correct"));
      return 1;
    }
    return setTorqueLEDControl(ID, newTorqueControl, newLEDControl);
  };

  /**
   * \brief Set torque actuation and LED colour for all
   * \details Modify the torque actuation and LED colour of the servo in both EEP and RAM. 
   * \param[in] newTorqueControl : Torque actuator to apply. Possible values are:
   * - \c HKX_TORQUE_FREE: no resistance to the movements (or let say mechanical resistance only)
   * - \c HKX_TORQUE_BREAK: the motor resists but allows the movements
   * - \c HKX_TORQUE_ON: the motor maintain its position (don't allow movements)
   * \param[in] newLEDControl : LED colour to set. Possible values are:
   * - \c HKX_LED_OFF,
   * - \c HKX_LED_GREEN,
   * - \c HKX_LED_BLUE,
   * - \c HKX_LED_RED,
   * - \c HKX_LED_CYAN,
   * - \c HKX_LED_YELLOW,
   * - \c HKX_LED_PINK,
   * - \c HKX_LED_WHITE,
   * \return 
   * 0 = OK \n
   * 1 = Input not correct 
   */
  uint8_t setAllTorqueLEDControl(hkxTorqueControl newTorqueControl,hkxLEDControl newLEDControl){return setTorqueLEDControl(HKX_ID_ALL, newTorqueControl, newLEDControl);};

  /**
   * \brief Get behaviour
   * \details Get the current behaviour of the servo. 
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] voltage : Current servo input voltage. Its value is given in millivolts. This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] temperature : Current temperature of the servo. Its value is given in 10^-2 °C (degree Celsius). This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] controlMode : Control mode of the current move, can be position control (\c HKX_CTRL_POSITION), or continuous rotation (\c HKX_CTRL_ROTATION). This parameter is optional, either set the address of a \ref hkxControlMode variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] tick : Current operating time of the servo. Its value is given in milliseconds). This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] absolutePosition : Current angle position of the servo. Its value is given in 10^-1 degrees. This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] velocity : Current angle velocity of the servo. Its value is given in degree / second. This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] PWM : Current PWM (load) of the servo. Its value is given in PWM (raw). This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] absoluteGoalPosition : Goal position of the current move of the servo. Its value is given in 10^-1 degrees. This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] absoluteDesiredTrajectoryPosition : Current trajectory position - desired position at a given time according the trajectory - of the servo. Its value is given in 10^-1 degrees. This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] desiredVelocity : Current trajectory velocity - desired velocity at a given time according the trajectory - of the servo. Its value is given in degree / second. This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   */
  uint8_t getBehaviour(uint8_t ID, HkxMaybe<uint16_t> voltage, HkxMaybe<uint16_t> temperature, HkxMaybe<hkxControlMode> controlMode, HkxMaybe<uint16_t> tick, HkxMaybe<int16_t> absolutePosition, HkxMaybe<int16_t> velocity, HkxMaybe<uint16_t> PWM, HkxMaybe<int16_t> absoluteGoalPosition, HkxMaybe<int16_t> absoluteDesiredTrajectoryPosition, HkxMaybe<int16_t> desiredVelocity);

  /**
   * \brief Get status
   * \details Get the status of the servo.
   * \param[in] ID : id of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] statusED : Current status of the servo.
   * \return
   *          0 = OK \n
   *          1 = bad input parameter \n
   *          2 = no data received \n
   *          3 = received data are not consistent
   */
  uint8_t getStatus(uint8_t ID, HkxStatus& statusED); 

  /**
   * \brief Clear status
   * \details Clear the status of the servo to leave the "error state" and stop the LED blink and torque deactivation.
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t clearOneStatus(uint8_t ID){
    if(ID > HKX_MAX_ID){
      errorPrint(F("clearOneStatus > Input not correct"));
      return 1;
    }
    return clearStatus(ID);
  };

  /**
   * \brief Clear status for all
   * \details Clear the status of the servo to leave the "error state" and stop the LED blink and torque deactivation.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t clearAllStatus(){return clearStatus(HKX_ID_ALL);};

  /**
   * \brief Scan IDs
   * \details Scan all the IDs to check witch servos are connected and their ID.
   * \param[out] IDs[] : Array of 254 booleans. When IDs[i] = \c true, the servo with ID = i is connected. When \c false, the servo is NOT connected.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = no data received \n
   * 3 = received data are not consistent
   */
  void scanIDs(boolean IDs[]);

  /**
   * \brief Factory reset
   * \details Reset all the parameters to factory default values.
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \param[in] IDskip : If \c true, the ID is NOT reset. If \c false, the ID is reset.
   * \param[in] baudrateSkip : If \c true, the baud rate is NOT reset. If \c false, the baud rate is reset.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t factoryResetOne(uint8_t ID, boolean IDskip, boolean baudrateSkip){
    if(ID > HKX_MAX_ID){
      errorPrint(F("factoryResetOne > Input not correct"));
      return 1;
    }
    return factoryReset(ID, IDskip, baudrateSkip);
  };

  /**
   * \brief Factory reset for all
   * \details Reset all the parameters to factory default values.
   * \param[in] IDskip : If \c true, the ID is NOT reset. If \c false, the ID is reset.
   * \param[in] baudrateSkip : If \c true, the baud rate is NOT reset. If \c false, the baud rate is reset.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t factoryResetAll(boolean IDskip, boolean baudrateSkip){return factoryReset(HKX_ID_ALL, IDskip, baudrateSkip);};

  /**
   * \brief Reboot
   * \details Reboot the servo.
   * \param[in] ID : ID of the servo to address the request. The value shall be [0 ; 253].
   * \return 
   * 0 = OK \n
   * 1 = Input not correct
   */
  uint8_t rebootOne(uint8_t ID){
    if(ID > HKX_MAX_ID){
      errorPrint(F("rebootOne > Input not correct"));
    return 1;
    }
    return reboot(ID);
  };

  /**
   * \brief Reboot
   * \details Reboot all the servo.
   */
  void rebootAll(){reboot(HKX_ID_ALL);};
  
};

#endif    // _HKXSETUP_H

