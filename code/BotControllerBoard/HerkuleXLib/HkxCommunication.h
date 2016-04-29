#ifndef _HKXCOMMUNICATION_H
#define _HKXCOMMUNICATION_H

/**
 * \file HkxCommunication.h
 * \brief Communication management of the Arduino library for HerkuleX servo (DRS-0101 and DRS-0201 models)
 * \author Benoit Puel (contact@benoitpuel.com)
 * \version 1.0
 * \date 11 December 2015
 * \copyright This project is released under the GNU GPL V3.
 */

#include "Arduino.h"
#include "HkxStatus.h"

/** \cond developer */
/** Activation increase the number of checks that are necessary is the library is stable */
const bool HKX_DEV = true;
/** Maximum time (in millisecond per byte to read) to wait before stopping the reading. */
const uint8_t HKX_WAIT_TIME = 10; 
/** Number of trials to send a request. */
const uint8_t HKX_NUMBER_SEND_TRIALS = 5; 
/** Number of trials to connect to a servo. */
const uint8_t HKX_NUMBER_CONNECT_TRIALS = 2; 
/** Size (bytes) of the serial buffer. \warning Do not modify unless you changed the serial buffer size (specific procedure) */
const uint8_t HKX_SERIAL_BUFFER = 64; 

/** Maximum possible value of the ID that can be attributed to the servo */
const uint8_t HKX_MAX_ID = 0xFD;
/** ID value to call all the servos */
const uint8_t HKX_ID_ALL = 0xFE;
/** Maximum possible value for the polices (LED and Torque)  */
const uint8_t HKX_MAX_POLICY = 0x7F;
/** Maximum possible value of the limit temperature allowed to run the servo */
const uint8_t HKX_MAX_TEMPERATURE = 0xFE;
/** Maximum possible value of the maximum voltage allowed to run the servo */
const uint8_t HKX_MAX_VOLTAGE = 0xFE;
/** Maximum possible value of the acceleration ratio */
const uint8_t HKX_MAX_ACCELERATION_RATIO = 50;
/** Maximum possible value of the time */
const uint8_t HKX_MAX_TIME = 0xFE;
/** Maximum possible value for the positioning (dead zone, stop threshold and in position margin) */
const uint8_t HKX_MAX_POSITIONNING = 0xFE;
/** Maximum possible value for the saturator offset */
const uint8_t HKX_MAX_SATURATOR_OFFSET = 0xFE;
/** Maximum possible value for the minimum PWM of the saturator */
const uint8_t HKX_MAX_MIN_PWM = 0xFE;
/** Maximum possible value for the PWM */
const uint16_t HKX_MAX_PWM = 0x3FF;
/** Maximum possible value for the position */
const uint16_t HKX_MAX_POSITION = 0x3FF;
/** Maximum possible value for the control parameters (KP, KD, KI, FFG1 and FFG2)  */
const uint16_t HKX_MAX_CONTROL_PARAM = 0x7FFF;
/** Maximum possible value for the check periods (LED blink, ADC fault check, packet garbage check, stop detection and overload detection)  */
const uint8_t HKX_MAX_CHECK_PERIOD = 0xFE;


/**
 * \enum hkxCommand
 * \brief Available commands
 * \details Available commands allowed for the servo requests (see Manual p40).
 * - \c HKX_CMD_ROM_WRITE: write registers in the ROM (non-volatile),
 * - \c HKX_CMD_ROM_READ: read registers in the ROM (non-volatile),
 * - \c HKX_CMD_RAM_WRITE: write registers in the RAM (volatile),
 * - \c HKX_CMD_RAM_READ: read registers in the RAM (volatile),
 * - \c HKX_CMD_I_JOG: move the servos with individual playtime for each (asynchronous),
 * - \c HKX_CMD_S_JOG: move the servos with the same playtime for all (synchronous),
 * - \c HKX_CMD_STAT: read the status,
 * - \c HKX_CMD_ROLLBACK: reset to factory default,
 * - \c HKX_CMD_REBOOT: reboot.
 */
enum hkxCommand:uint8_t{
  HKX_CMD_ROM_WRITE   = 0x01,
  HKX_CMD_ROM_READ    = 0x02,
  HKX_CMD_RAM_WRITE   = 0x03,
  HKX_CMD_RAM_READ    = 0x04,
  HKX_CMD_I_JOG       = 0x05,
  HKX_CMD_S_JOG       = 0x06,
  HKX_CMD_STAT        = 0x07,
  HKX_CMD_ROLLBACK    = 0x08,
  HKX_CMD_REBOOT      = 0x09,
};

/**
 * \enum hkxMemory
 * \brief Memory type
 * \details Memory type:
 * - \c HKX_ROM: non-volatile memory (EEP)
 * - \c HKX_RAM: volatile memory (RAM)
 */
enum hkxMemory:bool{
  HKX_ROM  = 0,
  HKX_RAM = 1,
};
/** \endcond */

/**
 * \enum hkxBaudrate
 * \brief Baud rate
 * \details Baud rate of the serial communication with the servos.
 * - \c HKX_57600: 57600 (from experience, this baud rate does NOT work properly),
 * - \c HKX_115200: 115200 bps,
 * - \c HKX_200000: 200000 bps,
 * - \c HKX_250000: 250000 bps,
 * - \c HKX_400000: 400000 bps,
 * - \c HKX_500000: 500000 bps,
 * - \c HKX_666666: 666666 bps.
 */
enum hkxBaudrate:uint32_t{
  HKX_57600 = 57600,
  HKX_115200 = 115200,
  HKX_200000 = 200000,
  HKX_250000 = 250000,
  HKX_400000 = 400000,
  HKX_500000 = 500000,
  HKX_666666 = 666666,
};

/**
 * \enum hkxTorqueControl
 * \brief Torque control
 * \details Torque control to apply:
 * - \c HKX_TORQUE_FREE: no resistance to the movements (or let say mechanical resistance only)
 * - \c HKX_TORQUE_BREAK: the motor resists but allows the movements
 * - \c HKX_TORQUE_ON: the motor maintain its position (don't allow movements)
 */
enum hkxTorqueControl:uint8_t{
  HKX_TORQUE_FREE  = 0x00,
  HKX_TORQUE_BREAK = 0x40,
  HKX_TORQUE_ON = 0x60,
};

/**
 * \enum hkxLEDControl
 * \brief LED control
 * \details LED control to apply: off (switch off), green, blue, red, cyan, yellow, pink or white.
 * - \c HKX_LED_OFF,
 * - \c HKX_LED_GREEN,
 * - \c HKX_LED_BLUE,
 * - \c HKX_LED_RED,
 * - \c HKX_LED_CYAN,
 * - \c HKX_LED_YELLOW,
 * - \c HKX_LED_PINK,
 * - \c HKX_LED_WHITE,
 */
enum hkxLEDControl:uint8_t{
  HKX_LED_OFF  = 0x00,
  HKX_LED_GREEN = 0x01,
  HKX_LED_BLUE = 0x02,
  HKX_LED_RED = 0x04,
  HKX_LED_CYAN = 0x03,
  HKX_LED_YELLOW = 0x05,
  HKX_LED_PINK = 0x06,
  HKX_LED_WHITE = 0x07,
};

/**
 * \enum hkxControlMode
 * \brief Control mode
 * \details Control mode to apply:
 * - \c HKX_CTRL_POSITION: the servo is control in angle position (angle position)
 * - \c HKX_CTRL_ROTATION: the servo is control in continues rotation (angle velocity)
 */
enum hkxControlMode:uint8_t{
  HKX_CTRL_POSITION = 0x00,
  HKX_CTRL_ROTATION = 0x01,
};

/** \cond developer */
/** Conversion table for temperature measurement in 10^-2 °C (degree Celcius) */
static const int16_t temperatureConversion[] = {-7947,-7178,-6320,-5781,-5380,-5058,-4786,-4549,-4340,-4151,-3979,-3820,-3673,-3535, 
    -3406,-3283,-3167,-3057,-2951,-2850,-2753,-2659,-2569,-2482,-2397,-2315,-2236,-2159,-2083,-2010,-1938,-1868,-1800,-1733,-1667,-1603,
    -1539,-1477,-1417,-1357,-1298,-1240,-1183,-1126,-1071,-1016,-962,-909,-856,-804,-753,-702,-652,-602,-553,-504,-456,-408,-361,-314,
    -267,-221,-175,-129,-84,-39,5,49,93,137,181,224,267,310,352,394,437,478,520,562,603,644,686,727,767,808,849,889,929,970,1010,1050,
    1090,1130,1170,1209,1249,1289,1328,1368,1407,1447,1486,1526,1565,1605,1644,1684,1723,1762,1802,1841,1881,1920,1960,1999,2039,2079,
    2119,2158,2198,2238,2278,2318,2359,2399,2439,2480,2520,2561,2602,2643,2684,2725,2766,2808,2850,2891,2933,2976,3018,3060,3103,3146,
    3189,3232,3276,3320,3364,3408,3453,3497,3542,3588,3633,3679,3725,3772,3818,3866,3913,3961,4009,4057,4106,4156,4205,4256,4306,4357,
    4409,4461,4513,4566,4619,4673,4728,4783,4839,4895,4952,5009,5068,5127,5186,5247,5308,5370,5433,5496,5561,5626,5693,5760,5828,5898,
    5968,6040,6113,6187,6263,6339,6417,6497,6578,6861,6746,6832,6920,7010,7102,7196,7292,7391,7492,7596,7703,7812,7925,8041,8160,8284,
    8411,8542,8679,8820,8966,9118,9276,9441,9613,9793,9982,10181,10390,10611,10845,11093,11359,11643,11949,12280,12641,13036,13472,
    13959,14509,15139,15873,16750,17829,19218,21132,24101,30091
}; 
/** \endcond */

/**
 * \enum hkxMaybeHasNoValue
 * \brief No value
 * \details No value for HkxMaybe class.
 * - \c HKX_NO_VALUE
 */
enum hkxMaybeHasNoValue:bool{
  HKX_NO_VALUE,
};

/** \cond developer */
/**
 * \class HkxMaybe
 * \brief Class to manage optional parameters
 * \details This class manages the optional parameters of the functions in order to avoid null pointer approach.
 */
template <typename T>
class HkxMaybe {
private:
  /** If empty => parameter to considered */
  const bool _empty; 
  /** Value of the parameter (if not empty)*/
  T* const _value;

public:
  /**
   * \brief Default constructor
   * \details The default constructor of HkxMaybe creates an empty parameter.
   */
  HkxMaybe() : _empty(true), _value(0){};

  /**
   * \brief Empty constructor
   * \details The empty constructor of HkxMaybe using \c HKX_NO_VALUE in order to simplify the writing of the code.
   *
   * A function \code{.cpp}foo(HkxMaybe<int> parameter)\endcode can be called with an empty parameter. \code{.cpp}foo(HkxMaybe<int>())\endcode
   * This constructor allow the simpler implementation thanks to the implicit conversion. \code{.cpp}foo(HKX_NO_VALUE)\endcode 
   * \param[in] noValue : this parameter can only have the value \c HKX_NO_VALUE.
   */
  HkxMaybe(hkxMaybeHasNoValue noValue) : _empty(true), _value(0){};

  /**
   * \brief Copy value constructor
   * \details This constructor is used for input optional parameters of function. Its value is copied and then cannot be returned when using implicit conversion.
   * \param[in] value : value of any type.
   */
  HkxMaybe(T value) : _empty(false), _value(new T(value)){};

  /**
   * \brief Pointer value constructor
   * \details This constructor is used for output optional parameters of function. Its value is pointed and then can be returned when using implicit conversion.
   * \param[in] addressValue : address of the value (any type).
   */
  HkxMaybe(T* addressValue) : _empty(false), _value(addressValue){};

  /**
   * \brief Default destructor
   * \details The destructor delete the pointed value.
   */
  ~HkxMaybe(){_value->~T();};

  /**
   * \brief Check if empty
   * \details Check if the parameter is empty or if it contains a value.
   * \return 
   * \c true if the parameter is empty. \n
   * \c false if the parameter contains a value.
   */
  bool isEmpty() const {return _empty; };

  /**
   * \brief Get the value
   * \details Get the value of the parameter if it has one.
   * \return 
   * \c nothing if the parameter is empty. \n
   * \c a copy of the value if the parameter has one.
   */
  T& getValue() const {if(!_empty){return *_value;}};

  /**
   * \brief Set the value
   * \details Set the value of the parameter if it is not empty. This member shall be use only when it was created with the pointer value constructor (for output optionnal parameter).
   * \param[in] value : value to copy to the pointed value if the parameter is not empty.
   */
  void setValue(T value) {if(!_empty){*_value = value;}};
};
/** \endcond */

/**
 * \class HkxPrint
 * \brief Class to manage the print of the messages.
 * \details This class manages the print of the messages (error, warning and info) for a serial monitor.
 */
class HkxPrint {
public:
  /**
   * \brief Default constructor
   * \details The default constructor is used when no messages should be (user decision) or can be (for certain Arduino boards) sent.
   *
   * Example for void print communication:
   * \code HkxPrint print = HkxPrint(); \endcode
   */
  HkxPrint() : _errorMessages(false), _warningMessages(false), _infoMessages(false), _serialPrint(0){};

  /**
   * \brief Active constructor with default values
   * \details The active constructor with default values setup the serial communication for only error messages.
   * \param[in] serialPrint : Serial port to use to send the messages. It will be most ot the time \c Serial.
   * \param[in] baudrate : value of the bauderate to send the messages. The same value shall be setup for the serial monitor. 
   *
   * Example to display only error messages over \c Serial port:
   * \code HkxPrint print = HkxPrint(Serial, 9600); \endcode
   */
  HkxPrint(HardwareSerial& serialPrint, long baudrate) : _errorMessages(true), _warningMessages(false), _infoMessages(false), _serialPrint(&serialPrint){
    _serialPrint->begin(baudrate);
    infoPrint(F("start > Start the communication for printing"));
  };

  /**
   * \brief Active constructor
   * \details The active constructor setup the serial communication and the type of messages to send (error, warning and info).
   * \param[in] serialPrint : Serial port to use to send the messages. It will be most ot the time \c Serial.
   * \param[in] baudrate : value of the bauderate to send the messages. The same value shall be setup for the serial monitor. 
   * \param[in] errorMessages : set \c true to display error messages, \c false otherwise. 
   * \param[in] warningMessages : set \c true to display warning messages, \c false otherwise. 
   * \param[in] infoMessages : set \c true to display info messages, \c false otherwise.  
   *
   * Example to display error and warning messages (but not info) over \c Serial port:
   * \code HkxPrint print = HkxPrint(Serial, 9600, true, true, false); \endcode
   */
  HkxPrint(HardwareSerial& serialPrint, long baudrate, bool errorMessages, bool warningMessages, bool infoMessages) : _errorMessages(errorMessages), _warningMessages(warningMessages), _infoMessages(infoMessages), _serialPrint(&serialPrint){
    _serialPrint->begin(baudrate);
    infoPrint(F("start > Start the communication for printing"));
  };

  /**
   * \brief Copy constructor
   * \details The copy constructor copy the serial communication from reference hkxPrint, and setup the type of messages to send (error, warning and info).
   * \param[in] hkxPrint : Reference HkxPrint for the copy.
   * \param[in] errorMessages : set \c true to display error messages, \c false otherwise. 
   * \param[in] warningMessages : set \c true to display warning messages, \c false otherwise. 
   * \param[in] infoMessages : set \c true to display info messages, \c false otherwise.
   *
   * Example to copy \c print1 (that display only errors) but display error, warning and info messages over \c Serial port:
   * \code 
   * HkxPrint print1 = HkxPrint(Serial, 9600);
   * HkxPrint print2 = HkxPrint(print1, true, true, true); 
   * \endcode
   */
  HkxPrint(HkxPrint hkxPrint, bool errorMessages, bool warningMessages, bool infoMessages) : _errorMessages(errorMessages), _warningMessages(warningMessages), _infoMessages(infoMessages), _serialPrint(hkxPrint._serialPrint){};
  
  /**
   * \brief Print error
   * \details Print an error message if setup.
   * \param[in] message : content to print as an error.  
   *
   * Example:
   * \code 
   * HkxPrint print = HkxPrint(print1, true, true, true); 
   * print.errorPrint(F("Hello world")); // F("...") stores the text in flash memory to save RAM memory
   * \endcode
   */
  void errorPrint(const String& message) const {if(_errorMessages){_serialPrint->println(String(F("Error:   ")) + message);}};

  /**
   * \brief Print warning
   * \details Print a warning message if setup.
   * \param[in] message : content to print as a warning.  
   *
   * Example:
   * \code 
   * HkxPrint print = HkxPrint(print1, true, true, true); 
   * print.warningPrint(F("Hello world")); // F("...") stores the text in flash memory to save RAM memory
   * \endcode
   */
  void warningPrint(const String& message) const {if(_warningMessages){_serialPrint->println(String(F("Warning: ")) + message);}};

  /**
   * \brief Print info
   * \details Print an info message if setup.
   * \param[in] message : content to print as an info.  
   *
   * Example:
   * \code 
   * HkxPrint print = HkxPrint(print1, true, true, true); 
   * print.infoPrint(F("Hello world")); // F("...") stores the text in flash memory to save RAM memory
   * \endcode
   */
  void infoPrint(const String& message) const {if(_infoMessages){_serialPrint->println(String(F("Info:    ")) + message);}};
  /** \cond developer */

  /** Print the errors */
  const boolean _errorMessages;      
  /** Print the warnings */
  const boolean _warningMessages;    
  /** Print the infos */
  const boolean _infoMessages;       
  /** Serial port to print messages */
  HardwareSerial* const _serialPrint; 
/** \endcond */
};

/** \cond developer */
/**
 * \class HkxUnitConversion
 * \brief Class to manage unit conversions.
 * \details This class manages the unit conversions between raw values as sent by the servo and physical quantity.
 */
class HkxUnitConversion {
public:
  /**
   * \brief Voltage from raw to value
   * \details Conversion from raw (servo unit) to value (physical quantity) of the voltage. 
   * \f[ voltage = raw \times 0.074 [volts] \f]
   * \param[in] voltageRaw : voltage in raw unit.
   * \return the voltage in millivolts.
   */
  static int16_t voltageRawToValue(uint8_t voltageRaw){return (uint16_t)voltageRaw*74;};

  /**
   * \brief Voltage from value to raw
   * \details Conversion from value (physical quantity) to raw (servo unit) of the voltage.
   * \f[ voltage = raw \times 0.074 [volts] \f]
   * \param[in] voltageValue : voltage in millivolts.
   * \return the voltage in raw unit.
   */
  static uint8_t voltageValueToRaw(int16_t voltageValue){return (uint8_t)(voltageValue/74);};

  /**
   * \brief Temperature from raw to value
   * \details Conversion from raw (servo unit) to value (physical quantity) of the temperature. See the conversion table p.55 of the user manual for more details.
   * \param[in] temperatureRaw : temperature in raw unit.
   * \return the temperature in 10^-2 °C (degree Celsius).
   */
  static int16_t temperatureRawToValue(uint8_t temperatureRaw){return temperatureConversion[temperatureRaw];};

  /**
   * \brief Temperature from value to raw
   * \details Conversion from value (physical quantity) to raw (servo unit) of the temperature. See the conversion table p.55 of the user manual for more details.
   * \param[in] temperatureValue : temperature 10^-2 °C (degree Celsius).
   * \return the temperature in raw unit.
   */
  static uint8_t temperatureValueToRaw(int16_t temperatureValue){
    uint8_t temperatureRaw = 128;
    uint8_t sup = 254;
    uint8_t inf = 0;
    while((temperatureConversion[temperatureRaw] != temperatureValue) && ((sup - inf) > 1) ){ //dichotomy
      if(temperatureConversion[temperatureRaw] > temperatureValue) {sup = temperatureRaw;}
	  else {inf = temperatureRaw;}
      temperatureRaw = (sup + inf)/2;
    }
	return temperatureRaw; 
  }; // lower value approximation

  /**
   * \brief Angle from raw to value
   * \details Conversion from raw (servo unit) to value (physical quantity) of the angle. 
   * \f[ angle = raw \times 0.326 [degree] \f]
   * \param[in] angleRaw : angle in raw unit.
   * \return the angle in 10^-1 degrees.
   */
  static int16_t angleRawToValue(int16_t angleRaw){return ((int32_t)angleRaw)*326/100;};

  /**
   * \brief Angle from value to raw
   * \details Conversion from value (physical quantity) to raw (servo unit) of the angle.
   * \f[ angle = raw \times 0.326 [degree] \f]
   * \param[in] angleValue : angle in 10^-1 degrees.
   * \return the angle in raw unit.
   */
  static int16_t angleValueToRaw(int16_t angleValue){return ((int32_t)angleValue*100/326)+(angleValue<0?-1:(angleValue>0?1:0));}; // +1 because of the rounding

  /**
   * \brief Position from raw to value
   * \details Conversion from raw (servo unit) to value (physical quantity) of the angle position. The angle position refers to the figure p.32 of the user manual: 512 (raw) = 0°, 0 (raw) = -166.7°, 1023 (raw) = 166.7°.
   * \f[ position = (raw - 512) \times 0.326 [degree] \f]
   * \param[in] positionRaw : position in raw unit.
   * \return the position in 10^-1 degrees (reference to the central position).
   */
  static int16_t positionRawToValue(uint16_t positionRaw){return angleRawToValue((int16_t)positionRaw - 512);};

  /**
   * \brief Position from value to raw
   * \details Conversion from value (physical quantity) to raw (servo unit) of the angle position. The angle position refers to the figure p.32 of the user manual: 512 (raw) = 0°, 0 (raw) = -166.7°, 1023 (raw) = 166.7°. 
   * \f[ position = (raw - 512) \times 0.326 [degree] \f]
   * \param[in] positionValue : position in 10^-1 degrees (reference to the central position).
   * \return the position in raw unit.
   */
  static uint16_t positionValueToRaw(int16_t positionValue){return angleValueToRaw(positionValue) + 512;};

  /**
   * \brief Velocity from raw to value
   * \details Conversion from raw (servo unit) to value (physical quantity) of the angle velocity.
   * \f[ velocity = raw \times 29.09 [degree/second] \f]
   * \param[in] velocityRaw : velocity in raw unit.
   * \return the angle velocity in degrees/second.
   */
  static int16_t velocityRawToValue(int16_t velocityRaw){return ((int32_t)velocityRaw)*2909/100;};

  /**
   * \brief Velocity from value to raw
   * \details Conversion from value (physical quantity) to raw (servo unit) of the angle velocity.
   * \f[ velocity = raw \times 29.09 [degree/second] \f]
   * \param[in] velocityValue : velocity in degrees/second.
   * \return the velocity in raw unit.
   */
  static int16_t velocityValueRaw(int16_t velocityValue){return ((int32_t)velocityValue)*100/2909+(velocityValue<0?-1:(velocityValue>0?1:0));}; // +1 because of the rounding

  /**
   * \brief Time from raw to value
   * \details Conversion from raw (servo unit) to value (physical quantity) of the time.
   * \f[ time = raw \times 11.2 [millisecond] \f]
   * \param[in] timeRaw : time in raw unit.
   * \return the time in milliseconds.
   */
  static uint16_t timeRawToValue(uint8_t timeRaw){return ((uint16_t)timeRaw*112)/10;};

  /**
   * \brief Time from value to raw
   * \details Conversion from value (physical quantity) to raw (servo unit) of the time.
   * \f[ time = raw \times 11.2 [millisecond] \f]
   * \param[in] timeValue : time in degrees/second.
   * \return the time in raw unit.
   */
  static uint8_t timeValueToRaw(uint16_t timeValue){return (uint8_t)((timeValue*10)/112)+1;}; // +1 because of the rounding

  /**
   * \brief Slope from raw to value
   * \details Conversion from raw (servo unit) to value (physical quantity) of the PWM/angle slope.
   * \f[ slope = \frac{raw}{256 \times 0.326} [PWM/degree] \f]
   * \warning Caution should be taken with the slope raw value: 
   * - 0 => no slope (infinity slope), 
   * - 1 => minimum slope, 
   * - 32767 maximum value for a slope
   * \param[in] slopeRaw : slope in raw unit.
   * \return the slope in 10^-2 PWM/degree.
   */
  static uint16_t slopeRawToValue(uint16_t slopeRaw){return ((uint32_t)(slopeRaw?slopeRaw:32768)*1000)/835;};

  /**
   * \brief Slope from value to raw
   * \details Conversion from value (physical quantity) to raw (servo unit) of the PWM/angle slope.
   * \f[ slope = raw \times 256 \times 0.326 [PWM/degree] \f]
   * \warning Caution should be taken with the slope raw value: 
   * - 0 => no slope (infinity slope), 
   * - 1 => minimum slope, 
   * - 32767 maximum value for a slope
   * \param[in] slopeValue : slope in 10^-2 PWM/degree.
   * \return the slope in raw unit.
   */
  static uint16_t slopeValueToRaw(uint16_t slopeValue){
    uint16_t slopeRaw = (((uint32_t)slopeValue*835)/1000) + 1; // +1 because of the rounding (+ avoid 0 reserved for no slope)
    return slopeRaw>32767?0:slopeRaw;
  };

  /**
   * \brief Baud rate from raw to value
   * \details Conversion from raw (servo unit) to value (physical quantity) of the baud rate. 
   *
   * \verbatim
 ---------------------------------------------------------------------------------------------------------
 | hkxBaudrate | HKX_57600 | HKX_115200 | HKX_200000 | HKX_250000 | HKX_400000 | HKX_500000 | HKX_666666 |
 ---------------------------------------------------------------------------------------------------------
 |  baud rate  |   57600   |   115200   |   200000   |   250000   |   400000   |   500000   |   666666   |
 ---------------------------------------------------------------------------------------------------------
 |  raw value  |   0x22    |    0x10    |    0x09    |    0x07    |    0x04    |    0x03    |    0x02    | 
 --------------------------------------------------------------------------------------------------------- \endverbatim
   * \param[in] baudrateRaw : baudrateRaw in raw unit.
   * \return the baud rate in bps (bytes per second).
   */
  static hkxBaudrate baudrateRawToValue(uint8_t baudrateRaw){
    switch (baudrateRaw){
    case 0x02 : return HKX_666666;
    case 0x03 : return HKX_500000;
    case 0x04 : return HKX_400000;
    case 0x07 : return HKX_250000;
    case 0x09 : return HKX_200000;
    case 0x10 : return HKX_115200;
    case 0x22 : return HKX_57600; }
  };

  /**
   * \brief Baud rate from value to raw
   * \details Conversion from value (physical quantity) to raw (servo unit) of the baud rate.
   *
   * \verbatim
 ---------------------------------------------------------------------------------------------------------
 | hkxBaudrate | HKX_57600 | HKX_115200 | HKX_200000 | HKX_250000 | HKX_400000 | HKX_500000 | HKX_666666 |
 ---------------------------------------------------------------------------------------------------------
 |  baud rate  |   57600   |   115200   |   200000   |   250000   |   400000   |   500000   |   666666   |
 ---------------------------------------------------------------------------------------------------------
 |  raw value  |   0x22    |    0x10    |    0x09    |    0x07    |    0x04    |    0x03    |    0x02    | 
 --------------------------------------------------------------------------------------------------------- \endverbatim
   * \param[in] baudrate : baud rate in bps (bytes per second).
   * \return the baudrate in raw unit.
   */
  static uint8_t baudrateValueToRaw(hkxBaudrate baudrate){
    switch (baudrate){
    case HKX_666666 : return 0x02;
    case HKX_500000 : return 0x03;
    case HKX_400000 : return 0x04;
    case HKX_250000 : return 0x07;
    case HKX_200000 : return 0x09;
    case HKX_115200 : return 0x10;
    case HKX_57600  : return 0x22; }
  };
};

/** \endcond */
/**
 * \class HkxCommunication
 * \brief Class to manage the communication.
 * \details This class manages the communications with the servos. It manages some possible communications problems and proposes simple functions to send requests to the servos.
 */
class HkxCommunication {
  /** \cond developer */
private:
  /** Allocate longest possible size of packet to be read */
  uint8_t _packetRead[HKX_SERIAL_BUFFER]; 
  /** Serial port to communicate with the servos */
  HardwareSerial& _serialServos; 
  /** Communication to print messages */
  HkxPrint& _print; 
  /** Class name to print the messages */
  const String _className; 

  /**
   * \brief Send a packet
   * \details Send a packet through the servo communication port.
   * \param[in] ID : id of the servo to who the packet is addressed. The value should be [0 ; 254] (254 means all servos).
   * \param[in] CMD : Instruction command:
   * - \c HKX_CMD_ROM_WRITE: write registers in the ROM (non-volatile),
   * - \c HKX_CMD_ROM_READ: read registers in the ROM (non-volatile),
   * - \c HKX_CMD_RAM_WRITE: write registers in the RAM (volatile),
   * - \c HKX_CMD_RAM_READ: read registers in the RAM (volatile),
   * - \c HKX_CMD_I_JOG: move the servos with individual playtime for each (asynchronous),
   * - \c HKX_CMD_S_JOG: move the servos with the same playtime for all (synchronous),
   * - \c HKX_CMD_STAT: read the status,
   * - \c HKX_CMD_ROLLBACK: reset to factory default,
   * - \c HKX_CMD_REBOOT: reboot.
   * \param[in] data[] : Array of bytes containing data according to the CMD.
   * \param[in] dataLength : Length of the data array.
   */
  void sendPacket(uint8_t ID, hkxCommand CMD, const uint8_t data[], uint8_t dataLength);

  /**
   * \brief Receive a packet
   * \details Receive a packet from the servo communication port.
   * \param[out] dataLength : Length of the data array of the received packet.
   * \return 
   * 0 if the process went well \n
   * 1 if headers not found \n
   * 2 if packet size not found \n
   * 3 if not enough data available \n
   * 4 if checksum error
   */
  uint8_t receivePacket(uint8_t& dataLength);

  /**
   * \brief Read the received packet
   * \details Read the received packet from the servo communication port.
   * \warning Prior using this function, the function ReceivePacket() shall be executed
   * \param[out] ID : id of the servo who sent the packet.
   * \param[out] CMD : Instruction that is answered:
   * - \c HKX_CMD_ROM_WRITE: write registers in the ROM (non-volatile),
   * - \c HKX_CMD_ROM_READ: read registers in the ROM (non-volatile),
   * - \c HKX_CMD_RAM_WRITE: write registers in the RAM (volatile),
   * - \c HKX_CMD_RAM_READ: read registers in the RAM (volatile),
   * - \c HKX_CMD_I_JOG: move the servos with individual playtime for each (asynchronous),
   * - \c HKX_CMD_S_JOG: move the servos with the same playtime for all (synchronous),
   * - \c HKX_CMD_STAT: read the status,
   * - \c HKX_CMD_ROLLBACK: reset to factory default,
   * - \c HKX_CMD_REBOOT: reboot.
   * \param[out] data[] : Array of bytes containing data.
   * \param[out] dataLength : Length of the data array.
   * \param[out] statusED : Current status of the servo. This parameter is optional either set the address of a HkxStatus variable to get the status, or set \c HKX_NO_VALUE to ignore it.
   * \return 
   * 0 if the process went well \n
   * 1 if dataLength is not consistent with the packet size \n
   * 2 if no data received
   */
  uint8_t readPacket(uint8_t& ID, hkxCommand& CMD, uint8_t data[], uint8_t dataLength, HkxMaybe<HkxStatus> statusED);

  /**
   * \brief Checksum1
   * \details Calculation of the Checksum1 for the packet communication integrity check.
   * \f[ checksum1 = (packetSize \wedge ID \wedge CMD \wedge data[0] \wedge data[1] \wedge ... \wedge data[n]) \& 0xFE \f]
   * \f$ A \wedge B \f$ is the bit exclusive OR operator (XOR).
   * \param[in] packetSize : size of the packet.
   * \param[in] ID : id of the servo who sent the packet. The value shall be [0 ; 254] (254 means all servos).
   * \param[in] CMD : Instruction of the packet:
   * - \c HKX_CMD_ROM_WRITE: write registers in the ROM (non-volatile),
   * - \c HKX_CMD_ROM_READ: read registers in the ROM (non-volatile),
   * - \c HKX_CMD_RAM_WRITE: write registers in the RAM (volatile),
   * - \c HKX_CMD_RAM_READ: read registers in the RAM (volatile),
   * - \c HKX_CMD_I_JOG: move the servos with individual playtime for each (asynchronous),
   * - \c HKX_CMD_S_JOG: move the servos with the same playtime for all (synchronous),
   * - \c HKX_CMD_STAT: read the status,
   * - \c HKX_CMD_ROLLBACK: reset to factory default,
   * - \c HKX_CMD_REBOOT: reboot.
   * \param[in] data[] : Array of bytes containing data.
   * \param[in] dataLength : Length of the data array.
   * \return the checksum1 value
   */
  uint8_t checkSum1(uint8_t packetSize, uint8_t ID, hkxCommand CMD, const uint8_t data[], uint8_t dataLength){
    uint8_t cks = packetSize ^ ID ^ CMD;
    for(int i = 0 ; i < dataLength ; i++){
      cks = cks ^ data[i];
    }
    return cks & 0xFE;
  };

  /**
   * \brief Checksum2
   * \details Calculation of the Checksum2 for the packet communication integrity check.
   * \f[ checksum2 = (~checksum1) \& 0xFE \f]
   * \f$ ~A \f$ is the NOT operator.
   * \param[in] checkSum1 : result obtained with \c checkSum1().
   * \return the checksum2 value
   */
  uint8_t checkSum2(uint8_t checkSum1){return (~checkSum1) & 0xFE;};
  
  /**
   * \brief Reinitialization of _packetRead 
   * \details Reinitialization of the _packetRead array. Set all the values to 0;
   */
  void reinitPacketRead(){memset(_packetRead,0,sizeof(_packetRead));};

  /**
   * \brief Clean the buffer
   * \details Clean the buffer of the serial to avoid asynchronised packets.
   */
  void cleanSerialBuffer(){while(this->_serialServos.available()){this->_serialServos.read();}};

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
   * \details The constructor of HkxCommunication.
   * \param[in] baudrate : Value of the baud rate to communicate with the servos. The same value shall be setup in the servos. See below the available values 
   *
   * \verbatim
 ---------------------------------------------------------------------------------------------------------
 | hkxBaudrate | HKX_57600 | HKX_115200 | HKX_200000 | HKX_250000 | HKX_400000 | HKX_500000 | HKX_666666 |
 ---------------------------------------------------------------------------------------------------------
 |  baud rate  |   57600   |   115200   |   200000   |   250000   |   400000   |   500000   |   666666   |
 --------------------------------------------------------------------------------------------------------- \endverbatim
   * \warning From experience, the baud rate 57600 does NOT work properly. Please avoid using it.
   * \param[in] serialServos : Serial port to use to communicate with the servos. It could be \c Serial, \c Serial1, \c Serial2 or \c Serial3 depending on the Arduino board and the wiring.
   * \param[in] print : Communication to print messages
   *
   * Example:
   * \code 
   * HkxPrint print = HkxPrint(Serial, 9600); 
   * HkxCommunication communication = HkxCommunication(HKX_115200, Serial1, print); 
   * \endcode
   */
  HkxCommunication(hkxBaudrate baudrate, HardwareSerial& serialServos, HkxPrint& print);

  /** \cond developer */
  /**
   * \brief Read request
   * \details Read the registers of the servo 
   * \param[in] ID : id of the servo to address the request. The value shall be [0 ; 253].
   * \param[in] mem : Memory type:
   * - ROM: non-volatile memory
   * - RAM: volatile memory
   * \param[in] start : Address of the register to read.
   * \param[in] length : Length (number of) registers to read.
   * \param[out] returnData[] : Array with size of \c length to return the read registers.
   * \param[out] statusED : Current status of the servo. This parameter is optional, either set the address of a HkxStatus variable to get the status, or set \c HKX_NO_VALUE to ignore it.
   * \return
   *          0 = OK \n
   *          1 = bad input parameter \n
   *          2 = no data received \n
   *          3 = received data are not consistent
   */
  uint8_t readRequest(uint8_t ID, hkxMemory mem, uint8_t start, uint8_t length, uint8_t returnData[], HkxMaybe<HkxStatus> statusED);

  /**
   * \brief Write request
   * \details Write the registers of the servo 
   * \param[in] ID : id of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \param[in] mem : Memory type:
   * - ROM: non-volatile memory
   * - RAM: volatile memory
   * \param[in] start : Address of the register to read.
   * \param[in] length : Length (number of) registers to read.
   * \param[in] writeData[] : Array with size of \c length to write registers.
   * \return
   *          0 = OK \n
   *          1 = bad input parameter
   */
  uint8_t writeRequest(uint8_t ID, hkxMemory mem, uint8_t start, uint8_t length, const uint8_t writeData[]);

  /**
   * \brief Status request
   * \details Read the status of the servo 
   * \param[in] ID : id of the servo to address the request. The value shall be [0 ; 253].
   * \param[out] statusED : Current status of the servo.
   * \return
   *          0 = OK \n
   *          1 = bad input parameter \n
   *          2 = no data received \n
   *          3 = received data are not consistent
   */
  uint8_t statusRequest(uint8_t ID, HkxStatus& statusED);

  /**
   * \brief I_JOG request
   * \details Individual JOG: move the servos with individual playtime for each (asynchronous).
   * \param[in] length : Number of servos to move.
   * \param[in] jog[] : Array with size of \c length of jogs for each servo:
   *      - if mode is position control: jog = goal position [0 ; 1023],
   *      - if mode is continuous rotation: jog = PWM [0 ; 1023] + bit 14 for the sign.
   * \param[in] set[] : array with size of \c length of sets for each servo:
   *      - Bit 0 = Stop flag
   *      - Bit 1 = Control mode (cf. \ref hkxControlMode)
   *      - Bit 2 to 4 = LED (cf. \ref hkxLEDControl)
   *      - Bit 5 = JOG invalid (no action)
   * \param[in] ID[] : Array with size of \c length of ids of the servo to address the request. The values shall be [0 ; 253].
   * \param[in] playTime[] : Array with size of \c length of the playtime of each servo.
   * \return
   *          0 = OK \n
   *          1 = bad input parameter
   */
  uint8_t iJogRequest(uint8_t length, const uint16_t jog[], const uint8_t set[], const uint8_t ID[], const uint8_t playTime[]);

  /**
   * \brief S_JOG request
   * \details Several JOG: move the servos with the same playtime for all (synchronous). 
   * \param[in] playTime : Playtime for all servos.
   * \param[in] length : Number of servos to move.
   * \param[in] jog[] : Array with size of \c length of jogs for each servo:
   *      - if mode is position control: jog = goal position [0 ; 1023],
   *      - if mode is continuous rotation: jog = PWM [0 ; 1023] + bit 14 for the sign.
   * \param[in] set[] : array with size of \c length of sets for each servo:
   *      - Bit 0 = Stop flag
   *      - Bit 1 = Control mode (cf. \ref hkxControlMode)
   *      - Bit 2 to 4 = LED (cf. \ref hkxLEDControl)
   *      - Bit 5 = JOG invalid (no action)
   * \param[in] ID[] : Array with size of \c length of ids of the servo to address the request. The values shall be [0 ; 253].
   * \return
   *          0 = OK \n
   *          1 = bad input parameter
   */
  uint8_t sJogRequest(uint8_t playTime, uint8_t length, const uint16_t jog[], const uint8_t set[], const uint8_t ID[]);

 /**
  * \brief Start communication
  * \details Start the communication with the servos.
  * \param[in] newBaudrate : Value of the baud rate to communicate with the servos. The same value shall be setup in the servos. See below the available values 
   *
   * \verbatim
 ---------------------------------------------------------------------------------------------------------
 | hkxBaudrate | HKX_57600 | HKX_115200 | HKX_200000 | HKX_250000 | HKX_400000 | HKX_500000 | HKX_666666 |
 ---------------------------------------------------------------------------------------------------------
 |  baud rate  |   57600   |   115200   |   200000   |   250000   |   400000   |   500000   |   666666   |
 --------------------------------------------------------------------------------------------------------- \endverbatim
  * \warning From experience, the baud rate 57600 does NOT work properly. Please avoid using it.
  */
  void start(hkxBaudrate newBaudrate){
    _serialServos.begin(newBaudrate);
    infoPrint(F("start > Start the communication with the servos"));
  }

 /**
  * \brief Stop communication
  * \details Stop the communication with the servos.
  */
  void stop(){
    this->_serialServos.end();
    infoPrint(F("stop > Stop the communication with the servos"));
  }

  /**
   * \brief Reboot request
   * \details Reboot the servo
   * \param[in] ID : id of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \return
   *          0 = OK \n
   *          1 = bad input parameter
   */
  uint8_t rebootRequest(uint8_t ID){
    if(HKX_DEV && ID > 0xFE){
      errorPrint(F("rebootRequest > input values not correct"));
      return 1;
    }
    uint8_t sData[0];
    sendPacket(ID, HKX_CMD_REBOOT, sData, 0);  
    return 0;
  };

  /**
   * \brief Roll-back request
   * \details Reset the servo to factory default.
   * \param[in] ID : id of the servo to address the request. The value shall be [0 ; 254] (254 means all servos).
   * \param[in] IDskip : Set false to reset also ID to factory.
   * \param[in] bandSkip : Set false if reset also baud rate to factory.
   * \return
   *          0 = OK \n
   *          1 = bad input parameter
   */
  uint8_t rollbackRequest(uint8_t ID, boolean IDskip, boolean bandSkip){
    if(HKX_DEV && ID > 0xFE){
      errorPrint(F("rollbackRequest > input values not correct"));
      return 1;
    }
    uint8_t data[] = {IDskip?(uint8_t)1:(uint8_t)0 , bandSkip?(uint8_t)1:(uint8_t)0};
    sendPacket(ID, HKX_CMD_ROLLBACK, data, 2);
    return 0;
  };

  /**
   * \brief Get the Serial
   * \details Return a reference to the Serial port used for the communication with the servos.
   * \return reference to the Serial port used for the communication with the servos.
   */
  HardwareSerial& getSerial(){return _serialServos;};
/** \endcond */
};


#endif    // _HKXCOMMUNICATION_H

