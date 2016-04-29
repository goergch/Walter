#ifndef _HKXPOSCONTROL_H
#define _HKXPOSCONTROL_H

/**
 * \file HkxPosControl.h
 * \brief Servo position control of the Arduino library for HerkuleX servo (DRS-0101 and DRS-0201 models)
 * \author Benoit Puel (contact@benoitpuel.com)
 * \version 1.0
 * \date 11 December 2015
 * \copyright This project is released under the GNU GPL V3.
 */

#include "Arduino.h"
#include "HkxCommunication.h"
#include "HkxStatus.h"

/**
 * \class HkxPosControl
 * \brief Class to drive one servo in position control mode.
 * \details This class allows to drive one servo in position control mode.
 */
class HkxPosControl{
private:
  /** Communication with servos  */
  HkxCommunication& _herkXCom; 
  /** Communication to print messages   */
  HkxPrint& _print; 
  /** Connexion with the servo to drive  */
  boolean _connected; 
  /** \cond developer */
  /** Class name to print the messages */
  const String _className;
  /** \endcond */
  /** Shift of the neutral position [in degrees]  */
  int16_t _zeroPosition;
  /** ID of the servo  */
  uint8_t _id;
  /** Current RAM value of the dead zone (saturator curve) of the servo [raw]  */
  uint8_t _deadZone;
  /** Current RAM value of the saturator offset (saturator curve) of the servo [raw]  */
  uint8_t _saturatorOffset;
  /** Current RAM value of the saturator slope (saturator curve) of the servo [raw]  */
  uint16_t _saturatorSlope;
  /** Current RAM value of the PWM offset (saturator curve) of the servo [raw]  */
  int8_t _pwmOffset;
  /** Current RAM value of the min PWM (saturator curve) of the servo [raw]  */
  uint8_t _minPWM;
  /** Current RAM value of the max PWM (saturator curve) of the servo [raw]  */
  uint16_t _maxPWM;
  /** Min Position the servo is allowed to reach [raw]  */
  uint16_t _minPosition;
  /** Max Position the servo is allowed to reach [raw]  */
  uint16_t _maxPosition;
  /** Current RAM value of the torque control of the servo  */
  hkxTorqueControl _torqueControl;
  /** Current RAM value of the LED of the servo  */
  hkxLEDControl _ledControl;
  /** Last status received from the servo  */
  HkxStatus _statusED;

  /** \cond developer */  
  /**
   * \brief Update the status
   * \details Perform a status request to update the stored status \ref _statusED.
   * \return 
   *          0 = OK \n
   *          1 = Input not correct \n
   *          2 = Servo not connected \n
   *          3 = Data not consistent
   */
  uint8_t updateStatus();

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
  /** \endcond */
public:  
  /**
   * \brief Constructor
   * \details The constructor of HkxPosControl.
   * \param[in] ID : id of the servo to drive. The value shall be [0 ; 253].
   * \param[in] herkXCom : Communication with the servos. 
   * \param[in] print : Communication to print messages.
   *
   * Example:
   * \code 
   * HkxPrint print = HkxPrint(Serial, 9600); 
   * HkxCommunication communication = HkxCommunication(HKX_115200, Serial1, print); 
   * HkxPosControl servoPosControl = HkxPosControl(10, communication, print); // ID=10, print can be different to communication
   * \endcode
   */
  HkxPosControl(uint8_t ID, HkxCommunication& herkXCom, HkxPrint& print);
  HkxPosControl();
  void setup(uint8_t ID, HkxCommunication& herkXCom, HkxPrint& print);
  /** \cond developer */
  /**
   * \brief Try connect servo
   * \details Try to connect to the servo.
   * \return
   * - \c true: if the servo is connected, \n
   * - \c false: if the servo is NOT connected. 
   */
  boolean tryConnect();
  /** \endcond */

  /**
   * \brief Get the ID
   * \details Get the ID of the driven servo.
   * \return Return the ID. 
   *
   * Example:
   * \code 
   * ... 
   * HkxPosControl servoPosControl = HkxPosControl(10, communication, print);
   * ...
   * byte idOfMyServo = servoPosControl.getID();
   * \endcode
   */
  uint8_t getID() const {return this->_id;}; 

  /**
   * \brief Get torque control
   * \details Get the current torque control of the driven servo (see \ref hkxTorqueControl for more details about the options).
   * \return Return the torque control 
   *
   * Example:
   * \code 
   * ... 
   * HkxPosControl servoPosControl = HkxPosControl(10, communication, print);
   * hkxTorqueControl torqueControlOfMyServo = servoPosControl.getTorqueControl();
   * \endcode
   */
  hkxTorqueControl getTorqueControl() const {return this->_torqueControl;};

  /**
   * \brief Get LED control
   * \details Get the current LED control (colour) of the driven servo (see \ref hkxLEDControl for more details about the options).
   * \return Return the LED control. 
   *
   * Example:
   * \code 
   * ... 
   * HkxPosControl servoPosControl = HkxPosControl(10, communication, print);
   * hkxLEDControl ledControlOfMyServo = servoPosControl.getLEDControl();
   * \endcode
   */
  hkxLEDControl getLEDControl() const {return this->_ledControl;};

  /** \cond developer */
  /**
   * \brief Get servo communication
   * \details Get the communication of the servo.
   * \return Return the communication.
   */
  HkxCommunication& getCommunication() const {return this->_herkXCom;};

  /**
   * \brief Is the servo connected
   * \details Return if the servo is connected.
   * \return 
   * - \c true: if the servo is connected (last communication worked), \n
   * - \c false: if the servo is NOT connected (last communication failed). 
   */
  boolean isConnected() const {return this->_connected;};

  /**
   * \brief Convert relative to absolute position
   * \details Conversion from the relative position to the absolute position: \c absolute = \c relative + \ref _zeroPosition [degree]
   * \return Return the absolute position (in degrees).
   */
  int16_t positionRelativeToAbsolute(int16_t positionRelative) const {return positionRelative + _zeroPosition;};

  /**
   * \brief Convert absolute to relative position
   * \details Conversion from the absolute position to the relative position: \c relative = \c absolute - \ref _zeroPosition [degree]
   * \return Return the relative position (in degrees).
   */
  int16_t positionAbsoluteToRelative(int16_t positionAbsolute) const {return positionAbsolute - _zeroPosition;};

  /**
   * \brief Get min position
   * \details Get minimum position (raw unit) the servo is allowed to reach.
   * \return Return the min position.
   */
  uint16_t getMinPosition() const {return _minPosition;};

  /**
   * \brief Get max position
   * \details Get maximum position (raw unit) the servo is allowed to reach.
   * \return Return the max position.
   */
  uint16_t getMaxPosition() const {return _maxPosition;};

  /**
   * \brief Set the LED variable
   * \details Set the LED variable in the class without applying. 
   * \warning This member is for \ref HkxGroupPosControl use only.
   * \param[in] newLEDControl : Colour of the led (see \ref hkxLEDControl for more details).
   */
  void setLEDControlVariable(hkxLEDControl newLEDControl){this->_ledControl = newLEDControl;};
  /** \endcond */

  /**
   * \brief Check if the servo arrived
   * \details Check if the servo is arrived at its goal position.
   * \return 
   * - \c true if the servo reached its goal position.
   * - \c false if the servo is still moving.
   *
   * Example:
   * \code 
   * ... 
   * HkxPosControl servoPosControl = HkxPosControl(10, communication, print);
   * servoPosControl.move(900, 2000, HKX_NO_VALUE, false); // move to 90° in 2 seconds
   * ... // different actions done during the movement
   * while(!servoPosControl.isInPosition()){} // wait the servo to be arrived in position
   * \endcode
   */
  boolean isInPosition(){
    this->updateStatus();
    return this->_statusED.isDetail(HKX_STAT_INPOSITION_FLAG);
  };

  /**
   * \brief Get the status
   * \details Get the last received status from the servo.
   * \param[out] statusED : Last received status of the servo.
   * \param[out] update : 
   * - \c true to update the status, then to get the current status.
   * - \c false to get the last received status.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected \n
   * 3 = Data not consistent
   *
   * Example:
   * \code 
   * ... 
   * HkxPosControl servoPosControl = HkxPosControl(10, communication, print);
   * HkxStatus currentStatus;
   * byte error = servoPosControl.getStatus();
   * if(error==0 && currentStatus.isError(HKX_STAT_VOLTAGE)){ // check voltage error
   *   print.errorPrint(F("Please check input voltage")); 
   * }
   * \endcode
   */
  uint8_t getStatus(HkxStatus& statusED, boolean update);

  /**
   * \brief Clear the status
   * \details Clear the status of the servo to leave the "error state" and stop the LED blink and torque deactivation (see p.33 of the user manual for more details about LED and torque policies).
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected
   *
   * Example:
   * \code 
   * ... 
   * HkxPosControl servoPosControl = HkxPosControl(10, communication, print);
   * HkxStatus currentStatus;
   * byte error = servoPosControl.getStatus();
   * if(error==0 && currentStatus.isError(HKX_STAT_ALL)){ // check if any error
   *   servoPosControl.clearStatus(); 
   * }
   * \endcode
   */
  uint8_t clearStatus();

  /**
   * \brief Set load control parameters
   * \details Set the load control parameters of the servo on the RAM (not kept after servo reboot).\verbatim
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
   * 1 = Input not correct \n
   * 2 = Servo not connected
   *
   * Example:
   * \code 
   * ... 
   * HkxPosControl servoPosControl = HkxPosControl(10, communication, print);
   * servoPosControl.setLoad(9, 0, 39300, 0, 0, 1023); // non-flexible control
   * servoPosControl.move(900, 2000, HKX_NO_VALUE, true); // move to 90° in 2 seconds and wait arrival
   * servoPosControl.setLoad(9, 0, 1000, 0, 0, 1023); // flexible control
   * servoPosControl.move(-900, 500, HKX_NO_VALUE, true); // move to -90° in 0.5 seconds and wait arrival
   * \endcode
   */
  uint8_t setLoad(uint16_t newDeadZone, uint8_t newSaturatorOffset, const uint32_t& newSaturatorSlope, int8_t newPWMOffset, uint8_t newMinPWM, uint16_t newMaxPWM);

  /**
   * \brief Set torque actuation and LED colour
   * \details Set the torque actuation (\ref hkxTorqueControl for more details) and the LED colour (\ref hkxLEDControl for more details). 
   * \param[in] newTorqueControl : Torque control to apply. This parameter is optional, either set a \ref hkxTorqueControl variable, the value (list below), or set \c HKX_NO_VALUE to ignore it.
   * - \c HKX_TORQUE_FREE: no resistance to the movements (or let say mechanical resistance only)
   * - \c HKX_TORQUE_BREAK: the motor resists but allows the movements
   * - \c HKX_TORQUE_ON: the motor maintain its position (don't allow movements)
   * \param[in] newLEDControl : LED control to apply. This parameter is optional, either set a \ref hkxLEDControl variable, the value (list below), or set \c HKX_NO_VALUE to ignore it.
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
   * 1 = Input not correct \n
   * 2 = Servo not connected
   *
   * Example:
   * \code 
   * ... 
   * HkxPosControl servoPosControl = HkxPosControl(10, communication, print);
   * servoPosControl.setTorqueLEDControl(HKX_LED_GREEN, HKX_NO_VALUE); // set the led to green
   * servoPosControl.setTorqueLEDControl(HKX_LED_WHITE, HKX_TORQUE_BREAK); // set the led to white and the torque to break 
   * \endcode
   */
  uint8_t setTorqueLEDControl(HkxMaybe<hkxTorqueControl> newTorqueControl, HkxMaybe<hkxLEDControl> newLEDControl);

  /**
   * \brief Set current position
   * \details Set the current position of the servo in order to change its relative position (by modifying the \ref _zeroPosition).\n
   * \ref _zeroPosition = \c absolute - \c current;
   * \param[in] currentPosition : The current position to calibrate the servo. The value shall be set in 10^-1 degrees within the range [-360° ; 360°]. 
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected
   *
   * Example:
   * \code 
   * ... 
   * HkxPosControl servoPosControl = HkxPosControl(10, communication, print);
   * servoPosControl.setCurrentPositionTo(0); // set current position as the reference
   * servoPosControl.move(-450, 500, HKX_NO_VALUE, true); // move to +45° (compared to reference) in 0.5 seconds and wait arrival 
   * \endcode
   */
  uint8_t setCurrentPositionTo(int16_t currentPosition);

  /**
   * \brief Move position
   * \details Move the servo.
   * \param[in] destinationAngle : Relative position to reach. Its value shall be set in 10^-1 degrees. Its range depends on \ref _zeroPosition, \ref _minPosition and \ref _maxPosition. 
   * \param[in] playTime : Time of the action trajectory for the move. Its value shall be set in milliseconds within the range [0 ; 2845ms]. 
   * \param[in] LEDControl : LED control to apply. This parameter is optional, either set a \ref hkxLEDControl variable, the value (list below), or set \c HKX_NO_VALUE to ignore it.
   * - \c HKX_LED_OFF,
   * - \c HKX_LED_GREEN,
   * - \c HKX_LED_BLUE,
   * - \c HKX_LED_RED,
   * - \c HKX_LED_CYAN,
   * - \c HKX_LED_YELLOW,
   * - \c HKX_LED_PINK,
   * - \c HKX_LED_WHITE,
   * \param[in] waitStop : 
   * - \c true to wait the servo reach the goal position \c destinationAngle (\ref isInPosition()).
   * - \c false to continue running the programme during the move. This could same computation time but the user shall check with \ref isInPosition() that the servo reached the goal before setting the next move.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected
   *
   * Example:
   * \code 
   * ... 
   * HkxPosControl servoPosControl = HkxPosControl(10, communication, print);
   * servoPosControl.move(1200, 1500, HKX_NO_VALUE, true); // move to position 120° in 1.5 seconds and wait arrival 
   * \endcode
   */
  uint8_t movePosition(int16_t destinationAngle, uint16_t playTime, HkxMaybe<hkxLEDControl> LEDControl, bool waitStop);

  /**
   * \brief Get the servo behaviour
   * \details Get the current servo behaviour. See the user manual for more details about the parameters.
   * \param[out] inputVoltage : Current servo input voltage. Its value is given in millivolts. This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] temperature : Current temperature of the servo. Its value is given in 10^-2 °C (degree Celsius). This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] position : Current relative position of the servo. Its value is given in 10^-1 degrees. This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] velocity : Current angle velocity of the servo. Its value is given in degree / second. This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] PWM : Current PWM (load) of the servo. Its value is given in PWM (raw). This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] goalPosition : Goal position of the current move of the servo. Its value is given in 10^-1 degrees. This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] trajectoryPosition : Current trajectory position - desired position at a given time according the trajectory - of the servo. Its value is given in 10^-1 degrees. This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \param[out] trajectoryVelocity : Current trajectory velocity - desired velocity at a given time according the trajectory - of the servo. Its value is given in degree / second. This parameter is optional, either set the address of a \c uint16_t (or \c unsigned \c int) variable to return the value, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected
   *
   * Example 1: How to get all the values.
   * \code
   * ... 
   * HkxPosControl servoPosControl = HkxPosControl(10, communication, print);
   * uint16_t inputVoltage, temperature, PWM;
   * int16_t position, velocity, goalPosition, trajectoryPosition, trajectoryVelocity;
   * servoPosControl->getBehaviour(&inputVoltage, &temperature, &position, &velocity, &PWM, &goalPosition, &trajectoryPosition, &trajectoryVelocity);
   * \endcode
   * Example 2: How to get the current position only.
   * \code
   * int16_t position;
   * servoPosControl->getBehaviour(HKX_NO_VALUE, HKX_NO_VALUE, &position, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE);
   * \endcode
   */
  uint8_t getBehaviour(HkxMaybe<uint16_t> inputVoltage, HkxMaybe<uint16_t> temperature, HkxMaybe<int16_t> position, HkxMaybe<int16_t> velocity, HkxMaybe<uint16_t> PWM, HkxMaybe<int16_t> goalPosition, HkxMaybe<int16_t> trajectoryPosition, HkxMaybe<int16_t> trajectoryVelocity);

  /**
   * \brief Reboot servo
   * \details Reboot the servo. This action will erase the ram, then reinitialize the parameters with the EEP (ROM) values. Wait some time (> 500 ms) before asking any action from the servo.
   * \return 
   * 0 = OK \n
   * 2 = Servo not connected
   *
   * Example:
   * \code
   * ... 
   * HkxPosControl servoPosControl = HkxPosControl(10, communication, print);
   * servoPosControl->reboot();
   * wait(500); // wait for the servo to restart
   * ...
   * \endcode
   */
  uint8_t reboot();
};

#endif    // _HKXPOSCONTROL_H



