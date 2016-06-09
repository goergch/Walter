/**
 * \file HkxPosControl.cpp
 * \brief Servo position control of the Arduino library for HerkuleX servo (DRS-0101 and DRS-0201 models)
 * \author Benoit Puel (contact@benoitpuel.com)
 * \version 1.0
 * \date 11 December 2015
 * \copyright This project is released under the GNU GPL V3.
 */

#include "HkxPosControl.h"


HkxPosControl::HkxPosControl(uint8_t ID, HkxCommunication& herkXCom, HkxPrint& print) : _herkXCom(herkXCom), _print(print){
  if(ID > 0xFD){
    // errorPrint(F("HkxPosControl > ID not correct"));
    return;
  }

  this->_id = ID;
  this->_statusED = HkxStatus();
  this->_torqueControl = HKX_TORQUE_FREE;
  this->_ledControl = HKX_LED_OFF;
  this->_zeroPosition = 0;
  this->_connected = false;
  
  tryConnect();
}


boolean HkxPosControl::tryConnect(){
  uint8_t error = _herkXCom.statusRequest(this->_id, _statusED);
  if (error==2){
	  /*
    String message = F("tryConnect > Servo ID=");
    message += this->_id;
    message += F(" is not connected");
    warningPrint(message);
	*/
    _connected = false;
    return false;
  } else {
    if(!_connected){ //if the servo was not connected before
      _connected = true;
      // Get setting data from the RAM
      uint8_t ramDataStart = 0;
      uint8_t ramDataLength = 24;
      uint8_t ramData[ramDataLength];
      uint8_t errorRAM = _herkXCom.readRequest(this->_id, HKX_RAM, ramDataStart, ramDataLength, ramData, _statusED);

      if(ramData[1] != 1){ // Force ACK Policy to be 1
        uint8_t ACKdata[] = {1};
        _herkXCom.writeRequest(this->_id, HKX_ROM, 7, 1, ACKdata);
        _herkXCom.writeRequest(this->_id, HKX_RAM, 1, 1, ACKdata);
      }

      this->_deadZone = ramData[10];
      this->_saturatorOffset = ramData[11];
      this->_saturatorSlope = ((uint16_t)ramData[13]<<8) | ramData[12];
      this->_pwmOffset = ramData[14];
      this->_minPWM = ramData[15];
      this->_maxPWM = ((uint16_t)ramData[17]<<8) | ramData[16];
      this->_minPosition = ramData[21]<<8 | ramData[20];
      this->_maxPosition = ramData[23]<<8 | ramData[22];

      // if the Servo has an error, then clean
      if (this->_statusED.isError(HKX_STAT_ALL) || this->_statusED.isDetail(HKX_STAT_ALL)){
        clearStatus();
      }
    }
    return true;
  }
}


uint8_t HkxPosControl::updateStatus(){
  if(!_connected){
    boolean connected = tryConnect();
    if(!connected){
      return 2;
    }
  }
  uint8_t error = _herkXCom.statusRequest(this->_id, _statusED);
  if(error == 2){// no data received
    String message = F("updateStatus > Servo ID=");
    message += this->_id;
    message += F(" is not connected");
    _connected = false;
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }

  return 0;
}


uint8_t HkxPosControl::getStatus(HkxStatus& statusED, boolean update){
  if(!_connected){
    boolean connected = tryConnect();
    if(!connected){
      return 2;
    }
  }

  uint8_t error;
  if(update){
    error = updateStatus();
  }else{
    error = 0;
  }
  
  uint8_t statusRaw[2];
  this->_statusED.getRawData(statusRaw);
  statusED.setRawData(statusRaw);
  
  return error;
}


uint8_t HkxPosControl::clearStatus(){
  if(!_connected){
    boolean connected = tryConnect();
    if(!connected){
      return 2;
    }
  }
  // Set the status to 0
  uint8_t clearedStatus[] = {0x00 , 0x00};
  _herkXCom.writeRequest(this->_id, HKX_RAM, 48, 2, clearedStatus);
  // Switch back the torque and the LED to their previous status
  uint8_t torqueLEDControl[] = {this->_torqueControl, this->_ledControl};
  _herkXCom.writeRequest(this->_id, HKX_RAM, 52, 2, torqueLEDControl);
  
  return 0;
}


uint8_t HkxPosControl::setLoad(uint16_t newDeadZone, uint8_t newSaturatorOffset, const uint32_t& newSaturatorSlope, int8_t newPWMOffset, uint8_t newMinPWM, uint16_t newMaxPWM){
  if(!_connected){
    boolean connected = tryConnect();
    if(!connected){
      return 2;
    }
  }
  
  uint8_t newDeadZoneRaw = HkxUnitConversion::angleValueToRaw(newDeadZone);
  uint16_t newSaturatorSlopeRaw = HkxUnitConversion::slopeValueToRaw(newSaturatorSlope); 

  // if uncorrect values
  if(newDeadZoneRaw > HKX_MAX_POSITIONNING || newSaturatorOffset > HKX_MAX_SATURATOR_OFFSET || newMinPWM > HKX_MAX_MIN_PWM || newMinPWM > newMaxPWM || newMaxPWM > HKX_MAX_PWM){
    // errorPrint(F("setLoad > Input not correct"));
    return 1;
  }

  // if same as current
  if (this->_deadZone == newDeadZoneRaw && this->_saturatorOffset == newSaturatorOffset && this->_saturatorSlope == newSaturatorSlopeRaw
      && this->_pwmOffset == newPWMOffset && this->_minPWM == newMinPWM && this->_maxPWM == newMaxPWM){
    return 0;
  }
  
  uint8_t data[] = {newDeadZoneRaw,
                    newSaturatorOffset,
                    (uint8_t)(newSaturatorSlopeRaw&0xFF), 
                    (uint8_t)((newSaturatorSlopeRaw&0xFF00)>>8),
                    (uint8_t) newPWMOffset,
                    newMinPWM,
                    (uint8_t)(newMaxPWM&0xFF), 
                    (uint8_t)((newMaxPWM&0xFF00)>>8)};
  _herkXCom.writeRequest(_id, HKX_RAM, 10, 8, data);

  this->_deadZone = newDeadZoneRaw;
  this->_saturatorOffset = newSaturatorOffset;
  this->_saturatorSlope = newSaturatorSlopeRaw;
  this->_pwmOffset = newPWMOffset;
  this->_minPWM = newMinPWM;
  this->_maxPWM = newMaxPWM;

  return 0;
}


uint8_t HkxPosControl::setTorqueLEDControl(HkxMaybe<hkxTorqueControl> newTorqueControl, HkxMaybe<hkxLEDControl> newLEDControl){
  if(!_connected){
    boolean connected = tryConnect();
    if(!connected){
      return 2;
    }
  }

  if (newTorqueControl.isEmpty() && newLEDControl.isEmpty()){
    return 1;
  }

  uint8_t sizeData = 0;
  if (!newTorqueControl.isEmpty() && !newLEDControl.isEmpty()){
    sizeData = 2;
  } else {
    sizeData = 1;
  }

  uint8_t data[sizeData];
  uint8_t start;
  if (!newTorqueControl.isEmpty() && !newLEDControl.isEmpty()){
    _torqueControl = newTorqueControl.getValue();
    _ledControl = newLEDControl.getValue();
    data[0] = newTorqueControl.getValue();
    data[1] = newLEDControl.getValue();
    start = 52;
  } else {
    if (newTorqueControl.isEmpty()){
      data[0] = newLEDControl.getValue();
      start = 53;
      _ledControl = newLEDControl.getValue();
    } else {
      data[0] = newTorqueControl.getValue();
      start = 52;
    _torqueControl = newTorqueControl.getValue();
    }
  }

  _herkXCom.writeRequest(_id, HKX_RAM, start, sizeData, data);
    
  return 0;
}


uint8_t HkxPosControl::setCurrentPositionTo(int16_t currentPosition){
  if(!_connected){
    boolean connected = tryConnect();
    if(!connected){
      return 2;
    }
  }
  if(abs(currentPosition) > 3600){
    // errorPrint(F("setCurrentPositionTo > Input not correct"));
    return 1;
  }
  uint8_t positionData[2];
  uint8_t error = _herkXCom.readRequest(_id, HKX_RAM, 60, 2, positionData, _statusED);

  if(error == 2){// no data received
	  /*
    String message = F("setCurrentPositionTo > Servo ID=");
    message += this->_id;
    message += F(" is not connected");
    warningPrint(message);
	*/
    this->_connected = false;
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }
  
  uint16_t absolutePositionRaw = (positionData[1]&3)<<8 | positionData[0];
  uint16_t absolutePosition = HkxUnitConversion::positionRawToValue(absolutePositionRaw);
  _zeroPosition = absolutePosition - currentPosition;
  
  return 0;
}


uint8_t HkxPosControl::movePosition(int16_t destinationAngle, uint16_t playTime, HkxMaybe<hkxLEDControl> LEDControl, bool waitStop){
  if(!_connected){
    boolean connected = tryConnect();
    if(!connected){
      return 2;
    }
  }

  uint16_t destinationAngleRaw[] = {HkxUnitConversion::positionValueToRaw(positionRelativeToAbsolute(destinationAngle))};
  uint8_t playTimeRaw = HkxUnitConversion::timeValueToRaw(playTime);

  if(destinationAngleRaw[0] < _minPosition || destinationAngleRaw[0] > _maxPosition || playTimeRaw > HKX_MAX_TIME){
    // errorPrint(F("movePosition > Input not correct"));
    return 1;
  }

  // current torque and LED control
  hkxTorqueControl currentTorqueControl = this->_torqueControl;
  hkxLEDControl currentLEDControl = this->_ledControl;
  // new LED control
  hkxLEDControl newLEDControl = LEDControl.isEmpty()?_ledControl:LEDControl.getValue();
  setLEDControlVariable(newLEDControl);
  // torque on if not activated
  if (currentTorqueControl != HKX_TORQUE_ON){
    setTorqueLEDControl(HKX_TORQUE_ON, HKX_NO_VALUE);
  }

  // data for the sJog command
  uint8_t set[] = {0};
  set[0] |= newLEDControl<<2;
  uint8_t IDs[] = {this->_id};
  _ledControl = newLEDControl;
  _herkXCom.sJogRequest(playTimeRaw, 1, destinationAngleRaw, set, IDs);
  
  if(waitStop){
    // wait stop
    delay(playTime);
    while(!this->isInPosition()){
    }

    // back to previous torque and LED states
    if (currentTorqueControl != HKX_TORQUE_ON || currentLEDControl != newLEDControl){
      this->setTorqueLEDControl(currentTorqueControl, currentLEDControl);
    }
  }

  return 0;
}


uint8_t HkxPosControl::getBehaviour(HkxMaybe<uint16_t> inputVoltage, HkxMaybe<uint16_t> temperature, HkxMaybe<int16_t> position,HkxMaybe<int16_t> velocity, HkxMaybe<uint16_t> PWM, HkxMaybe<int16_t> goalPosition, HkxMaybe<int16_t> trajectoryPosition, HkxMaybe<int16_t> trajectoryVelocity){
  if(!_connected){
    boolean connected = tryConnect();
    if(!connected){
      return 2;
    }
  }

  uint8_t behaviousData[20];
  uint8_t error = _herkXCom.readRequest(_id, HKX_RAM, 54, 20, behaviousData, _statusED);

  if(error == 2){// no data received
    // String message = F("getBehaviour > Servo ID=");
    // message += this->_id;
    // message += F(" is not connected");
    // warningPrint(message);
    this->_connected = false;
    return 2;
  }

  if(error == 3){// data received not consistent
    return 3;
  }
  
  if(!inputVoltage.isEmpty()){
    inputVoltage.setValue(HkxUnitConversion::voltageRawToValue(behaviousData[0]));
  }

/*
  if(!temperature.isEmpty()){
    temperature.setValue(HkxUnitConversion::temperatureRawToValue(behaviousData[1]));
  }
  */
  if(!position.isEmpty()){
    position.setValue(positionAbsoluteToRelative(HkxUnitConversion::positionRawToValue((behaviousData[7]&3)<<8 | behaviousData[6])));
  }

  if(!velocity.isEmpty()){
    velocity.setValue(HkxUnitConversion::velocityRawToValue(behaviousData[9]<<8 | behaviousData[8]));
  }
  if(!PWM.isEmpty()){
    PWM.setValue((behaviousData[11]&3)<<8 | behaviousData[10]);
  }
  if(!goalPosition.isEmpty()){
    goalPosition.setValue(positionAbsoluteToRelative(HkxUnitConversion::positionRawToValue((behaviousData[15]&3)<<8 | behaviousData[14])));
  }
  if(!trajectoryPosition.isEmpty()){
    trajectoryPosition.setValue(positionAbsoluteToRelative(HkxUnitConversion::positionRawToValue((behaviousData[17]&3)<<8 | behaviousData[16])));
  }
  if(!trajectoryVelocity.isEmpty()){
    trajectoryVelocity.setValue(HkxUnitConversion::velocityRawToValue(behaviousData[19]<<8 | behaviousData[18]));
  }
  return 0;
}


uint8_t HkxPosControl::reboot(){
  if(!_connected){
    boolean connected = tryConnect();
    if(!connected){
      return 2;
    }
  }
  _herkXCom.rebootRequest(this->_id);
  return 0;
}


