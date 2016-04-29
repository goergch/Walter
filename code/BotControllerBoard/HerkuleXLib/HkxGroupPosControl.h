#ifndef _HKXGROUPPOSCONTROL_H
#define _HKXGROUPPOSCONTROL_H

/**
 * \file HkxGroupPosControl.h
 * \brief Group of servos position control of the Arduino library for HerkuleX servo (DRS-0101 and DRS-0201 models)
 * \author Benoit Puel (contact@benoitpuel.com)
 * \version 1.0
 * \date 11 December 2015
 * \copyright This project is released under the GNU GPL V3.
 */

#include "HkxPosControl.h"

/**
 * \class HkxGroupPosControl
 * \brief Class to drive a group of servos in position control mode.
 * \details This class allows to drive a group of servos in position control mode.
 */
class HkxGroupPosControl{
private:
  /** Array of pointers to \ref HkxPosControl instances  */
  HkxPosControl** _servos;
  /** Number of servos in the table \ref _servos  */
  const uint8_t _length;
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
  /** \endcond */

public:  
  /**
   * \brief Constructor
   * \details The constructor of HkxGroupPosControl.
   * \warning \ref HkxCommunication shall be the same for all the servos of the group. If not, an error will be returned and the instance will be corrupted.
   * \param[in] length : Number of servos in the table \ref _servos. 
   * \warning It shall not exceed 53 servos, otherwise it cannot execute \ref moveSyncAllPosition() and \ref moveAsyncAllPosition(). 
   * \warning If it exceeds 43 servos then it cannot execute \ref moveAsyncAllPosition().
   * \param[in] arrayServos[] : Array of pointers to the servos. 
   * \param[in] print : Communication to print messages.
   */
  HkxGroupPosControl(uint8_t length, HkxPosControl* arrayServos[], HkxPrint& print);
  
  /**
   * \brief Get the nth servo
   * \details Get the nth servo.
   * \param[in] number : number of servo to get (from the table \ref _servos). 
   * \warning \c number < _ref _length otherwise return void.
   * \return Return a reference to the nth servo. 
   */
  HkxPosControl& getServo(uint8_t number){if(number < _length){return *_servos[number];}};

  /**
   * \brief Get number of servos
   * \details Get the number of servos manage by the instance.
   * \return Return the number of servos. 
   */
  uint8_t getNbServos(){return this->_length;};

  /**
   * \brief Set load control parameters
   * \details Set to all the servos the same load control parameters on their RAM (not kept after servo reboot).\verbatim
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
   * \param[in] newSaturatorOffset : The saturator offset is the step PWM value of the saturator once deadzone the position goes out of the deadzone (see the above figure). Its value shall be set in PWM within the range [0 ; 254].
   * \param[in] newSaturatorSlope : The saturator slop is a gradient of the saturator, going from the saturator offset and increasing gradually the PWM with the error position to the goal (see the above figure). It provides a flexible and elastic response of the servo to external forces. Its value shall be set in 10^-2 PWM / degree  within the range [0 ; 393 PWM/degree].
   * \param[in] newPWMOffset : The PWM offset allows compensating permanent load (e.g. gravity) by shifting the saturator by the same value (see the above figure). Its value shall be set in PWM within the range [-128 ; 127].
   * \param[in] newMinPWM : The min PWM is the minimum PWM that is applied by the servo (see the above figure). As specified in the user manual p.37, this may lead to unstable system. Use 0 as default value. The value shall be set in PWM within the range [0 ; 254].
   * \param[in] newMaxPWM : The max PWM is the maximum PWM that is applied by the servo (see the above figure). It could be used to limit the load of the servo, to avoid damages or injuries or instance. The value shall be set in PWM within the range [0 ; 1023].
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected
   */
  uint8_t setAllLoad(uint16_t newDeadZone, uint8_t newSaturatorOffset, const uint32_t& newSaturatorSlope, int8_t newPWMOffset, uint8_t newMinPWM, uint16_t newMaxPWM);


  /**
   * \brief Set torque actuation and LED colour
   * \details Set the same torque actuation (\ref hkxTorqueControl for more details) and the LED colour (\ref hkxLEDControl for more details) to all the servos. 
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
   */
  uint8_t setAllTorqueLEDControl(HkxMaybe<hkxTorqueControl> newTorqueControl, HkxMaybe<hkxLEDControl> newLEDControl);

  /**
   * \brief Synchronous move position 
   * \details Perform a synchronous (with the same \c playTime) move of all the servos.
   * \param[in] destinationAngle[] : Array of relative position to reach. Its value shall be set in 10^-1 degrees. Its range depends for each servo on \ref HkxPosControl._zeroPosition, \ref HkxPosControl._minPosition and \ref HkxPosControl._maxPosition. 
   * \param[in] playTime : Time of the action trajectory for the move. Its value shall be set in milliseconds within the range [0 ; 2845ms]. 
   * \param[in] LEDControl[] : Array of LED control to apply. This parameter is optional, either set a \ref hkxLEDControl variable, the value (list below), or set \c HKX_NO_VALUE to ignore it.
   * - \c HKX_LED_OFF,
   * - \c HKX_LED_GREEN,
   * - \c HKX_LED_BLUE,
   * - \c HKX_LED_RED,
   * - \c HKX_LED_CYAN,
   * - \c HKX_LED_YELLOW,
   * - \c HKX_LED_PINK,
   * - \c HKX_LED_WHITE,
   * \param[in] waitStop : 
   * - \c true to wait all the servos reach its goal position \c destinationAngle (\ref HkxPosControl.isInPosition()).
   * - \c false to continue running the programme during the move. This could same computation time but the user shall check with \ref HkxPosControl.isInPosition() that all the servos reached their goal before setting the next move.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected
   */
  uint8_t moveSyncAllPosition(int16_t destinationAngle[], uint16_t playTime, HkxMaybe<hkxLEDControl> LEDControl[], bool waitStop); 

  /**
   * \brief Asynchronous move position 
   * \details Move the servo asynchronous (with different \c playTime).
   * \param[in] destinationAngle[] : Array of relative position to reach. Its value shall be set in 10^-1 degrees. Its range depends for each servo on \ref HkxPosControl._zeroPosition, \ref HkxPosControl._minPosition and \ref HkxPosControl._maxPosition. 
   * \param[in] playTime : Array of time of the action trajectory for the move. Its value shall be set in milliseconds within the range [0 ; 2845ms]. 
   * \param[in] LEDControl[] : Array of LED control to apply. This parameter is optional, either set a \ref hkxLEDControl variable, the value (list below), or set \c HKX_NO_VALUE to ignore it.
   * - \c HKX_LED_OFF,
   * - \c HKX_LED_GREEN,
   * - \c HKX_LED_BLUE,
   * - \c HKX_LED_RED,
   * - \c HKX_LED_CYAN,
   * - \c HKX_LED_YELLOW,
   * - \c HKX_LED_PINK,
   * - \c HKX_LED_WHITE,
   * \param[in] waitStop : 
   * - \c true to wait all the servos reach its goal position \c destinationAngle (\ref HkxPosControl.isInPosition()).
   * - \c false to continue running the programme during the move. This could same computation time but the user shall check with \ref HkxPosControl.isInPosition() that all the servos reached their goal before setting the next move.
   * \return 
   * 0 = OK \n
   * 1 = Input not correct \n
   * 2 = Servo not connected
   */
  uint8_t moveAsyncAllPosition(int16_t destinationAngle[], uint16_t playTime[], HkxMaybe<hkxLEDControl> LEDControl[], bool waitStop); 


  /**
   * \brief Reboot 
   * \details Reboot all the servos.
   */
  void rebootAll(){
    for(int i=0 ; i < this->_length ; i++){
      this->_servos[i]->reboot();
    }
  };
};

#endif    // _HKXGROUPPOSCONTROL_H

