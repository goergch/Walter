/**
 * \file HkxGroupPosControl.cpp
 * \brief Group of servos position control of the Arduino library for HerkuleX servo (DRS-0101 and DRS-0201 models)
 * \author Benoit Puel (contact@benoitpuel.com)
 * \version 1.0
 * \date 11 December 2015
 * \copyright This project is released under the GNU GPL V3.
 */

#include "HkxGroupPosControl.h"


HkxGroupPosControl::HkxGroupPosControl(uint8_t length, HkxPosControl* arrayServos[], HkxPrint& print) : _herkXCom(arrayServos[0]->getCommunication()), _print(print), _length(length), _className(F("HkxGroupPosControl::")){
  this->_servos = new HkxPosControl*[length];
  uint8_t ids[length];
  for(int i = 0; i < length ; i++){
    this->_servos[i] = arrayServos[i];
    ids[i] = this->_servos[i]->getID();
    
    // Check the communication is the same for all servos
    if(i > 0){
      if(&(this->_servos[0]->getCommunication()) != &_herkXCom){
        errorPrint(F("HkxGroupPosControl > All servos need the same communication"));
        return;
      }
    }
    
    // Check IDs are different for the different servos
    for(int j = 0 ; j < i-1 ; j++){
      if(ids[i] == ids[j]){
        String messageID = F("HkxGroupPosControl > Servo id=");
        messageID += ids[i];
        messageID += F(" is set twice in the group");
        warningPrint(messageID);
      }
    }
    String messageADD = F("HkxGroupPosControl > Servo id=");
    messageADD += ids[i];
    messageADD += F(" is added to the group");
    infoPrint(messageADD);
  }
  if(length > 53){
    warningPrint(F("HkxGroupPosControl > Too many servos for asynchrone and synchrone moves"));
    return;
  } else if (length > 43){
    errorPrint(F("HkxGroupPosControl > Too many servos for asynchrone moves"));
    return;
  }
}


uint8_t HkxGroupPosControl::setAllLoad(uint16_t newDeadZone, uint8_t newSaturatorOffset, const uint32_t& newSaturatorSlope, int8_t newPWMOffset, uint8_t newMinPWM, uint16_t newMaxPWM){
  uint8_t error;
  boolean failConnect;
  uint8_t nbTryConnect;
  for(int i = 0 ; i < this->_length ; i++){
    failConnect = false;
    nbTryConnect = 0;
    do{
      nbTryConnect++;
      error = this->_servos[i]->setLoad(newDeadZone, newSaturatorOffset, newSaturatorSlope, newPWMOffset, newMinPWM, newMaxPWM);
      switch(error){
      case 0: //OK
        failConnect = false;
        break;
      case 1: //Input not correct
        return error;
        break;
      case 2: //problem of servo connection
        String message = F("setAllLoad > Servo id=");
        message += this->_servos[i]->getID();
        message += F(" failed to connect (try ");
        message += nbTryConnect;
        message += F(")");
        warningPrint(message);
        failConnect = true;
        if(nbTryConnect == HKX_NUMBER_CONNECT_TRIALS){ // HKX_NUMBER_CONNECT_TRIALS defined in HkxCommunication.h
          return error;
        }
        break;
      }
    } while(failConnect);
  }
}


uint8_t HkxGroupPosControl::setAllTorqueLEDControl(HkxMaybe<hkxTorqueControl> newTorqueControl, HkxMaybe<hkxLEDControl> newLEDControl){
  uint8_t error;
  boolean failConnect;
  uint8_t nbTryConnect;
  for(int i = 0 ; i < this->_length ; i++){
    failConnect = false;
    nbTryConnect = 0;
    do{
      nbTryConnect++;
      error = this->_servos[i]->setTorqueLEDControl(newTorqueControl, newLEDControl);
      switch(error){
      case 0: //OK
        failConnect = false;
        break;
      case 1: //Input not correct
        return error;
        break;
      case 2: //problem of servo connection
        String message = F("setAllTorqueLEDControl > Servo id=");
        message += this->_servos[i]->getID();
        message += F(" failed to connect (try ");
        message += nbTryConnect;
        message += F(")");
        warningPrint(message);
        failConnect = true;
        if(nbTryConnect == HKX_NUMBER_CONNECT_TRIALS){ // HKX_NUMBER_CONNECT_TRIALS defined in HkxCommunication.h
          return error;
        }
        break;
      }
    } while(failConnect);
  }
  return 0;
}


uint8_t HkxGroupPosControl::moveSyncAllPosition(int16_t destinationAngle[], uint16_t playTime, HkxMaybe<hkxLEDControl> LEDControl[], bool waitStop){
  if(_length > 53){
    errorPrint(F("moveSyncAllPosition > Too many servos (> 53)"));
    return 4;
  }
  uint8_t playTimeRaw = HkxUnitConversion::timeValueToRaw(playTime);
  if(playTimeRaw > HKX_MAX_TIME){
    errorPrint(F("moveSyncAllPosition > Input not correct"));
    return 1;
  }
  // current torque and LED control
  hkxTorqueControl currentTorqueControl[this->_length];
  hkxLEDControl currentLEDControl[this->_length];
  hkxLEDControl newLEDControl[this->_length];
  // torque on if not activated
  for(int i=0 ; i < this->_length ; i++){
    currentTorqueControl[i] = this->_servos[i]->getTorqueControl();
    currentLEDControl[i] = this->_servos[i]->getLEDControl();
    newLEDControl[i] = LEDControl[i].isEmpty()?currentLEDControl[i]:LEDControl[i].getValue();
    if (currentTorqueControl[i] != HKX_TORQUE_ON){
      this->_servos[i]->setTorqueLEDControl(HKX_TORQUE_ON, HKX_NO_VALUE);
    }
  }

  uint8_t nbTryConnect;
  uint8_t error;
  // data for the sJog command
  uint16_t destinationAngleRaw[this->_length];
  uint8_t set[this->_length];
  uint8_t IDs[this->_length];
  for(int i=0 ; i < this->_length ; i++){
    if(!this->_servos[i]->isConnected()){
      nbTryConnect = 0;
      boolean connected;
      do{
        nbTryConnect++;
        String message = F("moveSyncAllPosition > Servo id=");
        message += this->_servos[i]->getID();
        message += F(" failed to connect (try ");
        message += nbTryConnect;
        message += F(")");
        warningPrint(message);
        connected = this->_servos[i]->tryConnect();
      } while(!connected && nbTryConnect < HKX_NUMBER_CONNECT_TRIALS); // HKX_NUMBER_CONNECT_TRIALS defined in HkxCommunication.h     
      if(nbTryConnect == HKX_NUMBER_CONNECT_TRIALS){
        return 2;
      }
    }
    destinationAngleRaw[i] = HkxUnitConversion::positionValueToRaw(_servos[i]->positionRelativeToAbsolute(destinationAngle[i]));

    if(destinationAngleRaw[i] < _servos[i]->getMinPosition() || destinationAngleRaw[i] > _servos[i]->getMaxPosition()){
      String message = F("moveSyncAllPosition > Try to move the servo id=");
      message += this->_servos[i]->getID();
      message += F(" out of the range (target:");
      message += destinationAngle[i];
      message += F(")");
      errorPrint(message);
      return 1;
    }
    set[i] = 0;
    set[i] |= newLEDControl[i]<<2;
    IDs[i] = this->_servos[i]->getID();
    this->_servos[i]->setLEDControlVariable(newLEDControl[i]);
  } 

  _herkXCom.sJogRequest(playTimeRaw, this->_length, destinationAngleRaw, set, IDs);

  if(waitStop){
    delay(playTime);
    // check stop
    boolean allInPosition = false;
    while(allInPosition){
      allInPosition = true;
      for(int i=0 ; i < this->_length ; i++){
        if(!this->_servos[i]->isInPosition()){
          allInPosition = false;
          break;
        }
      }
    }
  
    // back to previous torque and LED states
    for(int i=0 ; i < this->_length ; i++){
      if (currentTorqueControl[i] != HKX_TORQUE_ON || currentLEDControl[i] != newLEDControl[i]){ 
        this->_servos[i]->setTorqueLEDControl(currentTorqueControl[i], currentLEDControl[i]);
      }
    }
  }
  return 0;
}


uint8_t HkxGroupPosControl::moveAsyncAllPosition(int16_t destinationAngle[], uint16_t playTime[], HkxMaybe<hkxLEDControl> LEDControl[], bool waitStop){
  if(_length > 43){
    errorPrint(F("moveAsyncAllPosition > Too many servos (> 43)"));
    return 4;
  }
  // current torque and LED control
  hkxTorqueControl currentTorqueControl[this->_length];
  hkxLEDControl currentLEDControl[this->_length];
  hkxLEDControl newLEDControl[this->_length];
  // torque on if not activated
  for(int i=0 ; i < this->_length ; i++){
    currentTorqueControl[i] = this->_servos[i]->getTorqueControl();
    currentLEDControl[i] = this->_servos[i]->getLEDControl();
    newLEDControl[i] = LEDControl[i].isEmpty()?currentLEDControl[i]:LEDControl[i].getValue();
    if (currentTorqueControl[i] != HKX_TORQUE_ON){ 
      this->_servos[i]->setTorqueLEDControl(HKX_TORQUE_ON, HkxMaybe<hkxLEDControl>());
    }
  }
	
  uint8_t nbTryConnect;
  uint8_t error;
  // data for the sJog command
  uint16_t maxPlayTime = 0;
  uint8_t playTimeRaw[this->_length];
  uint16_t destinationAngleRaw[this->_length];
  uint8_t set[this->_length];
  uint8_t IDs[this->_length];
  for(int i=0 ; i < this->_length ; i++){
    // check the servo is connected
    if(!this->_servos[i]->isConnected()){
      nbTryConnect = 0;
      boolean connected;
      do{
        nbTryConnect++;
        String message = F("moveAsyncAllPosition > Servo id=");
        message += this->_servos[i]->getID();
        message += F(" failed to connect (try ");
        message += nbTryConnect;
        message += F(")");
        warningPrint(message);
        connected = this->_servos[i]->tryConnect();
      } while(!connected && nbTryConnect < HKX_NUMBER_CONNECT_TRIALS); // HKX_NUMBER_CONNECT_TRIALS defined in HkxCommunication.h     
      if(nbTryConnect == HKX_NUMBER_CONNECT_TRIALS){
        return 2;
      }
    }
    // check the playtime value
    playTimeRaw[i] = HkxUnitConversion::timeValueToRaw(playTime[i]);
    maxPlayTime = max(maxPlayTime, playTime[i]);
    if(playTimeRaw[i] > HKX_MAX_TIME){
      String message = F("moveAsyncAllPosition > Playtime for servo id=");
      message += this->_servos[i]->getID();
      message += F(" is too long");
      errorPrint(message);
      return 1;
    }
    // check the position value
    destinationAngleRaw[i] = HkxUnitConversion::positionValueToRaw(_servos[i]->positionRelativeToAbsolute(destinationAngle[i]));

    if(destinationAngleRaw[i] < _servos[i]->getMinPosition() || destinationAngleRaw[i] > _servos[i]->getMaxPosition()){
       String message = F("moveAsyncAllPosition > Try to move the servo id=");
      message += this->_servos[i]->getID();
      message += F(" out of the range (target:");
      message += destinationAngle[i];
      message += F(")");
      errorPrint(message);
      return 1;
    }
    set[i] = 0;
    set[i] |= newLEDControl[i]<<2;
    IDs[i] = this->_servos[i]->getID();
    this->_servos[i]->setLEDControlVariable(newLEDControl[i]);
  } 

  _herkXCom.iJogRequest(this->_length, destinationAngleRaw, set, IDs, playTimeRaw);

  if(waitStop){
	// check stop
    delay(maxPlayTime);
    boolean allInPosition = false;
    while(allInPosition){
      allInPosition = true;
      for(int i=0 ; i < this->_length ; i++){
        if(!this->_servos[i]->isInPosition()){
          allInPosition = false;
          break;
        }
      }
    }
  
    // back to previous torque and LED states
    for(int i=0 ; i < this->_length ; i++){
      if (currentTorqueControl[i] != HKX_TORQUE_ON || currentLEDControl[i] != newLEDControl[i]){ 
        this->_servos[i]->setTorqueLEDControl(currentTorqueControl[i], currentLEDControl[i]);
      }
    }
  }
  return 0;
}




