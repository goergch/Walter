#ifndef _HKXSTATUS_H
#define _HKXSTATUS_H

/**
 * \file HkxStatus.h
 * \brief Status management of the Arduino library for HerkuleX servo (DRS-0101 and DRS-0201 models)
 * \author Benoit Puel (contact@benoitpuel.com)
 * \version 1.0
 * \date 11 December 2015
 * \copyright This project is released under the GNU GPL V3.
 */

#include "Arduino.h"

// Status - See. Manual p39.

/** Status error: Exceed Input Voltage limit  */
const uint8_t HKX_STAT_VOLTAGE       = 0x01;
/** Status error: Exceed allowed POT limit  */
const uint8_t HKX_STAT_POSITION      = 0x02;
/** Status error: Exceed Temperature limit  */
const uint8_t HKX_STAT_TEMPERATURE   = 0x04;
/** Status error: Invalid Packet  */
const uint8_t HKX_STAT_PACKET        = 0x08;
/** Status error: Overload detected  */
const uint8_t HKX_STAT_OVERLOAD      = 0x10;
/** Status error: Driver fault detected  */
const uint8_t HKX_STAT_DRIVER        = 0x20;
/** Status error: EEP REG distorted  */
const uint8_t HKX_STAT_ROM_DISTORTED = 0x40;

/** Status detail: Moving flag  */
const uint8_t HKX_STAT_MOVING_FLAG     = 0x01;
/** Status detail: Inposition flag */
const uint8_t HKX_STAT_INPOSITION_FLAG = 0x02;
/** Status detail: Checksum Error */
const uint8_t HKX_STAT_CKECKSUM        = 0x04;
/** Status detail: Unknown Command */
const uint8_t HKX_STAT_COMMAND         = 0x08;
/** Status detail: Exceed REG range */
const uint8_t HKX_STAT_EXCEED_REGISTER = 0x10;
/** Status detail: Garbage detected */
const uint8_t HKX_STAT_GARBAGE         = 0x20;
/** Status detail: \c MOTOR_ON flag */
const uint8_t HKX_STAT_MOTOR_FLAG      = 0x40;

/** Status error and detail: all the list */
const uint8_t HKX_STAT_ALL = 0x7F;

/**
 * \class HkxStatus
 * \brief Class to manage the status of the servos
 * \details This class manages the status of the servos with an explicit syntax. 
 */
class HkxStatus{
private:
  /** status table ([0] errors and [1] details) */
  uint8_t _statusED[2]; 
  
public:
  /**
   * \brief Default constructor
   * \details Default constructor of HkxStatus.
   */
  HkxStatus(){
    _statusED[0] = 0;
    _statusED[1] = 0;
  };
  
  /**
   * \brief Check for errors
   * \details Check if the status has an error according to the mask.
   * \param[in] mask : 
   * The mask is defined with the parameters :
   * - \c HKX_STAT_VOLTAGE : Exceed Input Voltage limit
   * - \c HKX_STAT_POSITION : Exceed allowed POT limit
   * - \c HKX_STAT_TEMPERATURE : Exceed Temperature limit
   * - \c HKX_STAT_PACKET : Invalid Packet
   * - \c HKX_STAT_OVERLOAD : Overload detected
   * - \c HKX_STAT_DRIVER : Driver fault detected
   * - \c HKX_STAT_ROM_DISTORTED : EEP REG distorted
   * - \c HKX_STAT_ALL : all the list
   *
   * Example 1 : check if any error => \c mask = \c HKX_STAT_ALL.
   *
   * Example 2 : check if any error from the temperature or the voltage => \c mask = \c HKX_STAT_VOLTAGE | \c HKX_STAT_TEMPERATURE.
   *
   * \return 
   * \c true if the status contains any error from to the mask (at least one). \n
   * \c false if the status contains none of error from the mask.
   */
  boolean isError(uint8_t mask) const {
    return (mask &= this->_statusED[0])?true:false;
  };
  
  /**
   * \brief Check for details
   * \details Check if the details has an error according to the mask.
   * \param[in] mask : 
   * The mask is defined with the parameters :
   * - \c HKX_STAT_MOVING_FLAG : Moving flag
   * - \c HKX_STAT_INPOSITION_FLAG : In position flag
   * - \c HKX_STAT_CKECKSUM : Checksum Error
   * - \c HKX_STAT_COMMAND : Unknown Command
   * - \c HKX_STAT_EXCEED_REGISTER : Exceed REG range
   * - \c HKX_STAT_GARBAGE : Garbage detected
   * - \c HKX_STAT_MOTOR_FLAG : \c MOTOR_ON flag
   * - \c HKX_STAT_ALL : all the list
   *
   * Example 1 : check if any detail => \c mask = \c HKX_STAT_ALL.
   *
   * Example 2 : check if any detail from the checksum or garbage => \c mask = \c HKX_STAT_CKECKSUM | \c HKX_STAT_GARBAGE.
   *
   * \return 
   * \c true if the status contains any detail from to the mask (at least one). \n
   * \c false if the details contains none of detail from the mask.
   */
  boolean isDetail(uint8_t mask) const {
    return (mask &= this->_statusED[1])?true:false;
  };
  
  /**
   * \brief Get the status raw data
   * \details Get the raw data of the status
   * \param[out] statusED[] : table of 2 elements to return the raw data of the status.
   */  
  void getRawData(uint8_t statusED[]) const {
    statusED[0] = this->_statusED[0];
    statusED[1] = this->_statusED[1];
  };

  /**
   * \brief Set the status raw data
   * \details Set the raw data of the status.
   * \param[in] statusED[] : table of 2 elements to return the raw data of the status.
   */  
  void setRawData(const uint8_t statusED[]){
    this->_statusED[0] = statusED[0];
    this->_statusED[1] = statusED[1];
  };
};

#endif    // _HKXSTATUS_H


