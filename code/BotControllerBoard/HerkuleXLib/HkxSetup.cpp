/**
 * \file HkxSetup.cpp
 * \brief Servo setup of the Arduino library for HerkuleX servo (DRS-0101 and DRS-0201 models)
 * \author Benoit Puel (contact@benoitpuel.com)
 * \version 1.0
 * \date 11 December 2015
 * \copyright This project is released under the GNU GPL V3.
 */

#include "HkxSetup.h"


uint8_t HkxSetup::getServoInfo(uint8_t ID, HkxMaybe<uint16_t> model, HkxMaybe<uint16_t> version){
  if(ID > HKX_MAX_ID || (model.isEmpty() && version.isEmpty())){
    errorPrint(F("getServoInfo > Input not correct"));
    return 1;
  }
  
  uint8_t infoData[4];
  HkxStatus statusED;
  uint8_t error = _herkXCom.readRequest(ID, HKX_ROM, 0, 4 , infoData, statusED);

  if(error == 2){// no data received
    warningPrint(F("getServoInfo > servo not connected"));
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }

  model.setValue(infoData[0]<<8 | infoData[1]);
  version.setValue(infoData[2]<<8 | infoData[3]);
  
  return 0;
}


uint8_t HkxSetup::getBaudRate(uint8_t ID, hkxBaudrate* baudRate){
  if(ID > HKX_MAX_ID){
    errorPrint(F("getBaudRate > Input not correct"));
    return 1;
  }
  
  uint8_t daudRateRaw;
  uint8_t error = _herkXCom.readRequest(ID, HKX_ROM, 4, 1 , &daudRateRaw, HKX_NO_VALUE);

  if(error == 2){// no data received
    warningPrint(F("getBaudRate > servo not connected"));
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }
  
  *baudRate = HkxUnitConversion::baudrateRawToValue(daudRateRaw);

  return 0;
}

uint8_t HkxSetup::setAllBaudRate(const hkxBaudrate& baudrate){
  uint8_t baudRateByte = HkxUnitConversion::baudrateValueToRaw(baudrate);
  
  uint8_t data[] = {baudRateByte};
  _herkXCom.writeRequest(0xFE, HKX_ROM, 4, 1, data);
  delay(500);
  _herkXCom.rebootRequest(0xFE);
  
  // Restart the communication with the same baudrate
  _herkXCom.stop();
  _herkXCom.start(baudrate);
  delay(500);
  return 0;
}

uint8_t HkxSetup::setID(uint8_t ID, uint8_t newID){
  if(ID > HKX_MAX_ID || newID > HKX_MAX_ID){
    errorPrint(F("setID > Input not correct"));
    return 1;
  }
  
  HkxStatus tempStatusED;
  if(_herkXCom.statusRequest(newID, tempStatusED) != 2){
    errorPrint(F("setID > newID already used"));
    return 4;
  }
  
  uint8_t data[] = {newID};
  _herkXCom.writeRequest(ID, HKX_ROM, 6, 1, data);
  _herkXCom.writeRequest(ID, HKX_RAM, 0, 1, data);
    
  return 0;
}

uint8_t HkxSetup::getPolicies(uint8_t ID, HkxMaybe<uint8_t> alarmLEDPolicy, HkxMaybe<uint8_t> torquePolicy){
  if(ID > HKX_MAX_ID){
    errorPrint(F("getPolicies > Input not correct"));
    return 1;
  }
  uint8_t policyData[2];
  uint8_t error = _herkXCom.readRequest(ID, HKX_RAM, 2, 2 , policyData, HKX_NO_VALUE);

  if(error == 2){// no data received
    warningPrint(F("getPolicies > servo not connected"));
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }
  alarmLEDPolicy.setValue(policyData[0]);
  torquePolicy.setValue(policyData[1]);
  return 0;
}


uint8_t HkxSetup::setPolicies(uint8_t ID, uint8_t newAlarmLEDPolicy, uint8_t newTorquePolicy){
  if(ID > HKX_ID_ALL || newAlarmLEDPolicy > HKX_MAX_POLICY || newTorquePolicy > HKX_MAX_POLICY){
    errorPrint(F("setPolicies > Input not correct"));
    return 1;
  }
  
  uint8_t data[] = {newAlarmLEDPolicy, newTorquePolicy};
  _herkXCom.writeRequest(ID, HKX_ROM, 8, 2, data);
  _herkXCom.writeRequest(ID, HKX_RAM, 2, 2, data);
  
  return 0;
}

uint8_t HkxSetup::getTemperatureVoltage(uint8_t ID, HkxMaybe<int16_t> maxTemp, HkxMaybe<uint16_t> minVoltage, HkxMaybe<uint16_t> maxVoltage){
  if(ID > HKX_MAX_ID || (maxTemp.isEmpty() && minVoltage.isEmpty() && maxVoltage.isEmpty())){
    errorPrint(F("getTemperatureVoltage > Input not correct"));
    return 1;
  }

  uint8_t dataRaw[3];
  uint8_t error = _herkXCom.readRequest(ID, HKX_RAM, 5, 3 , dataRaw, HKX_NO_VALUE);

  if(error == 2){// no data received
    warningPrint(F("getTemperatureVoltage > servo not connected"));
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }
  maxTemp.setValue(HkxUnitConversion::temperatureRawToValue(dataRaw[0]));
  minVoltage.setValue(HkxUnitConversion::voltageRawToValue(dataRaw[1]));
  maxVoltage.setValue(HkxUnitConversion::voltageRawToValue(dataRaw[2]));
  return 0;
}


uint8_t HkxSetup::setTemperatureVoltage(uint8_t ID, int16_t newMaxTemp, uint16_t newMinVoltage, uint16_t newMaxVoltage){
  if(ID > HKX_ID_ALL || newMaxTemp > HkxUnitConversion::temperatureRawToValue(HKX_MAX_TEMPERATURE) || newMaxTemp < HkxUnitConversion::temperatureRawToValue(0) || newMinVoltage > newMaxVoltage || newMaxVoltage > HkxUnitConversion::voltageRawToValue(HKX_MAX_VOLTAGE)){
    errorPrint(F("setTemperatureVoltage > Input value not correct"));
    return 1;
  }
  
  uint8_t data[] = {HkxUnitConversion::temperatureValueToRaw(newMaxTemp), HkxUnitConversion::voltageValueToRaw(newMinVoltage), HkxUnitConversion::voltageValueToRaw(newMaxVoltage)};
  _herkXCom.writeRequest(ID, HKX_ROM, 11, 3, data);
  _herkXCom.writeRequest(ID, HKX_RAM, 5, 3, data);
    
  return 0;
}

uint8_t HkxSetup::getAcceleration(uint8_t ID, HkxMaybe<uint8_t> accelerationRatio, HkxMaybe<uint16_t> maxAccelerationTime){
  if(ID > HKX_MAX_ID || (accelerationRatio.isEmpty() && maxAccelerationTime.isEmpty())){
    errorPrint(F("getAcceleration > Input not correct"));
    return 1;
  }

  uint8_t accelerationData[2];
  uint8_t error = _herkXCom.readRequest(ID, HKX_RAM, 8, 2 , accelerationData, HKX_NO_VALUE);

  if(error == 2){// no data received
    warningPrint(F("getAcceleration > servo not connected"));
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }

  accelerationRatio.setValue(accelerationData[0]);
  maxAccelerationTime.setValue(HkxUnitConversion::timeRawToValue(accelerationData[1]));
  return 0;
}

uint8_t HkxSetup::setAcceleration(uint8_t ID, uint8_t newAccelerationRatio, uint16_t newMaxAccelerationTime){
  if(ID > HKX_ID_ALL || newAccelerationRatio > HKX_MAX_ACCELERATION_RATIO || newMaxAccelerationTime > HkxUnitConversion::timeRawToValue(0xFE)){
    errorPrint(F("setAcceleration > acceleration ratio value or max acceleration time not correct"));
    return 1;
  }

  uint8_t maxAccelTimeRaw = HkxUnitConversion::timeValueToRaw(newMaxAccelerationTime);
  
  uint8_t data[] = {newAccelerationRatio, maxAccelTimeRaw};
  _herkXCom.writeRequest(ID, HKX_ROM, 14, 2, data);
  _herkXCom.writeRequest(ID, HKX_RAM, 8, 2, data);
    
  return 0;
}


uint8_t HkxSetup::getLoadControl(uint8_t ID, HkxMaybe<uint16_t> deadZone, HkxMaybe<uint8_t> saturatorOffSet, HkxMaybe<uint32_t> saturatorSlope, HkxMaybe<int8_t> PWMoffset, HkxMaybe<uint8_t> minPWM, HkxMaybe<uint16_t> maxPWM){
  if(ID > HKX_MAX_ID || (deadZone.isEmpty() && saturatorOffSet.isEmpty() && saturatorSlope.isEmpty() && PWMoffset.isEmpty() && minPWM.isEmpty() && maxPWM.isEmpty())){
    errorPrint(F("getLoadControl > Input not correct"));
    return 1;
  }

  uint8_t loadData[8];
  uint8_t error = _herkXCom.readRequest(ID, HKX_RAM, 10, 8 , loadData, HKX_NO_VALUE);

  if(error == 2){// no data received
    warningPrint(F("getLoadControl > servo not connected"));
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }

  deadZone.setValue(HkxUnitConversion::angleRawToValue(loadData[0]));
  saturatorOffSet.setValue(loadData[1]);
  saturatorSlope.setValue(HkxUnitConversion::slopeRawToValue(loadData[3]<<8 | loadData[2]));
  PWMoffset.setValue(loadData[4]);
  minPWM.setValue(loadData[5]);
  maxPWM.setValue(loadData[7]<<8 | loadData[6]);
  return 0;
}

uint8_t HkxSetup::setLoadControl(uint8_t ID, uint16_t deadZone, uint8_t saturatorOffSet, const uint32_t& saturatorSlope, int8_t PWMoffset, uint8_t minPWM, uint16_t maxPWM){
  if(ID > HKX_ID_ALL || deadZone > HkxUnitConversion::angleRawToValue(HKX_MAX_POSITIONNING) || saturatorOffSet > HKX_MAX_SATURATOR_OFFSET || minPWM > HKX_MAX_MIN_PWM || maxPWM > HKX_MAX_PWM){
    errorPrint(F("setLoadControl > Input not correct"));
    return 1;
  }
  
  uint16_t saturatorSlopeRaw = HkxUnitConversion::slopeValueToRaw(saturatorSlope); 

  uint8_t data[] = {HkxUnitConversion::angleValueToRaw(deadZone),
                    saturatorOffSet, 
                    (uint8_t)(saturatorSlopeRaw&0xFF), 
                    (uint8_t)((saturatorSlopeRaw&0xFF00)>>8), 
                    (uint8_t)PWMoffset,
                    minPWM,
                    (uint8_t)(maxPWM&0xFF),
                    (uint8_t)((maxPWM&0xFF00)>>8)};
 
  _herkXCom.writeRequest(ID, HKX_ROM, 16, 8, data);
  _herkXCom.writeRequest(ID, HKX_RAM, 10, 8, data);
    
  return 0;
}

uint8_t HkxSetup::getPWMPositionLimits(uint8_t ID, HkxMaybe<uint16_t> overlaodPWMThreshold, HkxMaybe<int16_t> minPosition, HkxMaybe<int16_t> maxPosition){
  if(ID > HKX_MAX_ID){
    errorPrint(F("getPWMPositionLimits > Input not correct"));
    return 1;
  }

  uint8_t rData[6];
  uint8_t error = _herkXCom.readRequest(ID, HKX_RAM, 18, 6 , rData, HKX_NO_VALUE);

  if(error == 2){// no data received
    warningPrint(F("getPWMPositionLimits > servo not connected"));
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }
  
  overlaodPWMThreshold.setValue(((rData[1])<<8) | rData[0]);
  minPosition.setValue(HkxUnitConversion::positionRawToValue(rData[3]<<8 | rData[2])); 
  maxPosition.setValue(HkxUnitConversion::positionRawToValue(rData[5]<<8 | rData[4])); 
  return 0;
}

uint8_t HkxSetup::setPWMPositionLimits(uint8_t ID, uint16_t newOverlaodPWMThreshold, int16_t newMinPosition, int16_t newMaxPosition){
  if(ID > HKX_ID_ALL || newOverlaodPWMThreshold > HKX_MAX_PWM || newMinPosition > newMaxPosition || newMinPosition < HkxUnitConversion::positionRawToValue(0) || newMaxPosition > HkxUnitConversion::positionRawToValue(HKX_MAX_POSITION)){
    errorPrint(F("setPWMPositionLimits > Input not correct"));
    return 1;
  }

  uint16_t newMinPositionRaw = HkxUnitConversion::positionValueToRaw(newMinPosition);
  uint16_t newMaxPositionRaw = HkxUnitConversion::positionValueToRaw(newMaxPosition);

  uint8_t data[] = {(uint8_t)(newOverlaodPWMThreshold&0xFF), (uint8_t)((newOverlaodPWMThreshold&0xFF00)>>8), (uint8_t)(newMinPositionRaw&0xFF), (uint8_t)((newMinPositionRaw&0xFF00)>>8), (uint8_t)(newMaxPositionRaw&0xFF), (uint8_t)((newMaxPositionRaw&0xFF00)>>8)};
  _herkXCom.writeRequest(ID, HKX_ROM, 24, 6, data);
  _herkXCom.writeRequest(ID, HKX_RAM, 18, 6, data);
    
  return 0;
}


uint8_t HkxSetup::getControlSystem(uint8_t ID, HkxMaybe<uint16_t> kProportionnal, HkxMaybe<uint16_t> kDerivative, HkxMaybe<uint16_t> kInteger, HkxMaybe<uint16_t> feedforwardGain1, HkxMaybe<uint16_t> feedforwardGain2){
  if(ID > HKX_MAX_ID || (kProportionnal.isEmpty() && kDerivative.isEmpty() && kInteger.isEmpty() && feedforwardGain1.isEmpty() && feedforwardGain2.isEmpty())){
    errorPrint(F("getControlSystem > Input not correct"));
    return 1;
  }

  uint8_t controlSystemData[10];
  uint8_t error = _herkXCom.readRequest(ID, HKX_RAM, 24, 10 , controlSystemData, HKX_NO_VALUE);

  if(error == 2){// no data received
    warningPrint(F("getControlSystem > servo not connected"));
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }
  
  kProportionnal.setValue(((controlSystemData[1])<<8) | controlSystemData[0]);
  kDerivative.setValue(((controlSystemData[3])<<8) | controlSystemData[2]);
  kInteger.setValue(((controlSystemData[5])<<8) | controlSystemData[4]);
  feedforwardGain1.setValue(((controlSystemData[7])<<8) | controlSystemData[6]);
  feedforwardGain2.setValue(((controlSystemData[9])<<8) | controlSystemData[8]);
  return 0;
}


uint8_t HkxSetup::setControlSystem(uint8_t ID, uint16_t kProportionnal, uint16_t kDerivative, uint16_t kInteger, uint16_t feedforwardGain1, uint16_t feedforwardGain2){
  if(ID > HKX_ID_ALL || kProportionnal > HKX_MAX_CONTROL_PARAM || kDerivative > HKX_MAX_CONTROL_PARAM || kInteger > HKX_MAX_CONTROL_PARAM || feedforwardGain1 > HKX_MAX_CONTROL_PARAM || feedforwardGain2 > HKX_MAX_CONTROL_PARAM){
    errorPrint(F("setControlSystem > Input not correct"));
    return 1;
  }

  uint8_t data[] = {(uint8_t)(kProportionnal&0xFF), (uint8_t)((kProportionnal&0xFF00)>>8), 
                    (uint8_t)(kDerivative&0xFF), (uint8_t)((kDerivative&0xFF00)>>8),
                    (uint8_t)(kInteger&0xFF), (uint8_t)((kInteger&0xFF00)>>8),
                    (uint8_t)(feedforwardGain1&0xFF), (uint8_t)((feedforwardGain1&0xFF00)>>8),
                    (uint8_t)(feedforwardGain2&0xFF), (uint8_t)((feedforwardGain2&0xFF00)>>8)};
  _herkXCom.writeRequest(ID, HKX_ROM, 30, 10, data);
  _herkXCom.writeRequest(ID, HKX_RAM, 24, 10, data);
    
  return 0;
}

uint8_t HkxSetup::getErrorsCheckPeriod(uint8_t ID, HkxMaybe<uint16_t> LEDBlink, HkxMaybe<uint16_t> ADCFault, HkxMaybe<uint16_t> packetGarbage, HkxMaybe<uint16_t> stopDetection, HkxMaybe<uint16_t> overloadDetection){
  if(ID > HKX_MAX_ID){
    errorPrint(F("getErrorsCheckPeriod > Input not correct"));
    return 1;
  }

  uint8_t checkPeriodData[5];
  uint8_t error = _herkXCom.readRequest(ID, HKX_RAM, 38, 5 , checkPeriodData, HKX_NO_VALUE);

  if(error == 2){// no data received
    warningPrint(F("getErrorsCheckPeriod > servo not connected"));
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }
  
  LEDBlink.setValue(HkxUnitConversion::timeRawToValue(checkPeriodData[0]));
  ADCFault.setValue(HkxUnitConversion::timeRawToValue(checkPeriodData[1]));
  packetGarbage.setValue(HkxUnitConversion::timeRawToValue(checkPeriodData[2]));
  stopDetection.setValue(HkxUnitConversion::timeRawToValue(checkPeriodData[3]));
  overloadDetection.setValue(HkxUnitConversion::timeRawToValue(checkPeriodData[4]));
  return 0;
}


uint8_t HkxSetup::setErrorsCheckPeriod(uint8_t ID, uint16_t LEDBlink, uint16_t ADCFault, uint16_t packetGarbage, uint16_t stopDetection, uint16_t overloadDetection){
  uint16_t maxCheckPeriod = HkxUnitConversion::timeRawToValue(HKX_MAX_CHECK_PERIOD);
  if(ID > HKX_ID_ALL || LEDBlink > maxCheckPeriod || ADCFault > maxCheckPeriod || packetGarbage > maxCheckPeriod || stopDetection > maxCheckPeriod || overloadDetection > maxCheckPeriod){
    errorPrint(F("setErrorsCheckPeriod > Input not correct"));
    return 1;
  }

  uint8_t data[] = {HkxUnitConversion::timeValueToRaw(LEDBlink),
                    HkxUnitConversion::timeValueToRaw(ADCFault),
                    HkxUnitConversion::timeValueToRaw(packetGarbage),
                    HkxUnitConversion::timeValueToRaw(stopDetection),
                    HkxUnitConversion::timeValueToRaw(overloadDetection),};

  _herkXCom.writeRequest(ID, HKX_ROM, 44, 5, data);
  _herkXCom.writeRequest(ID, HKX_RAM, 38, 5, data);
    
  return 0;
}


uint8_t HkxSetup::getInPositionCriteria(uint8_t ID, HkxMaybe<uint16_t> stopThreshold, HkxMaybe<uint16_t> inPositionMargin){
  if(ID > HKX_MAX_ID){
    errorPrint(F("getInPositionCriteria > Input not correct"));
    return 1;
  }

  uint8_t inPositionData[2];
  uint8_t error = _herkXCom.readRequest(ID, HKX_RAM, 43, 2 , inPositionData, HKX_NO_VALUE);

  if(error == 2){// no data received
    warningPrint(F("getInPositionCriteria > servo not connected"));
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }
  
  stopThreshold.setValue(HkxUnitConversion::angleRawToValue(inPositionData[0]));
  inPositionMargin.setValue(HkxUnitConversion::angleRawToValue(inPositionData[1]));
  return 0;
}

uint8_t HkxSetup::setInPositionCriteria(uint8_t ID, uint16_t stopThreshold, uint16_t inPositionMargin){
  uint16_t maxPositionning = HkxUnitConversion::angleRawToValue(HKX_MAX_POSITIONNING);
  if(ID > HKX_ID_ALL || stopThreshold > maxPositionning || inPositionMargin > maxPositionning){
    errorPrint(F("setInPositionCriteria > Input not correct"));
    return 1;
  }

  uint8_t data[] = {(uint8_t)HkxUnitConversion::angleValueToRaw(stopThreshold),
                    (uint8_t)HkxUnitConversion::angleValueToRaw(inPositionMargin)};
  _herkXCom.writeRequest(ID, HKX_ROM, 49, 2, data);
  _herkXCom.writeRequest(ID, HKX_RAM, 43, 2, data);
    
  return 0;
}

uint8_t HkxSetup::getTorqueLEDControl(uint8_t ID, HkxMaybe<hkxTorqueControl> torqueControl, HkxMaybe<hkxLEDControl> LEDControl){
  if(ID > HKX_MAX_ID){
    errorPrint(F("getTorqueLEDControl > Input not correct"));
    return 1;
  }

  uint8_t data[2];
  uint8_t error = _herkXCom.readRequest(ID, HKX_RAM, 52, 2 , data, HKX_NO_VALUE);

  if(error == 2){// no data received
    warningPrint(F("getTorqueLEDControl > servo not connected"));
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }
  
  torqueControl.setValue(hkxTorqueControl(data[0]));
  LEDControl.setValue(hkxLEDControl(data[1]));
  return 0;
}

uint8_t HkxSetup::setTorqueLEDControl(uint8_t ID, hkxTorqueControl newTorqueControl, hkxLEDControl newLEDControl){
  if(ID > HKX_ID_ALL){
    errorPrint(F("setTorqueLEDControl > Input not correct"));
    return 1;
  }
  uint8_t data[] = {newTorqueControl, newLEDControl};
  _herkXCom.writeRequest(ID, HKX_RAM, 52, 2, data);
  return 0;
}



uint8_t HkxSetup::getBehaviour(uint8_t ID, HkxMaybe<uint16_t> voltage, HkxMaybe<uint16_t> temperature, HkxMaybe<hkxControlMode> controlMode, HkxMaybe<uint16_t> tick, HkxMaybe<int16_t> absolutePosition, HkxMaybe<int16_t> velocity, HkxMaybe<uint16_t> PWM, HkxMaybe<int16_t> absoluteGoalPosition, HkxMaybe<int16_t> absoluteDesiredTrajectoryPosition, HkxMaybe<int16_t> desiredVelocity){
  if(ID > HKX_MAX_ID){
    errorPrint(F("getBehaviour > Input not correct"));
    return 1;
  }
  uint8_t behaviousData[20];
  uint8_t error = _herkXCom.readRequest(ID, HKX_RAM, 54, 20, behaviousData, HKX_NO_VALUE);

  if(error == 2){// no data received
    warningPrint(F("getBehaviour > servo not connected"));
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }
  
  voltage.setValue(HkxUnitConversion::voltageRawToValue(behaviousData[0]));
  
  temperature.setValue(HkxUnitConversion::temperatureRawToValue(behaviousData[1]));
  
  controlMode.setValue(hkxControlMode(behaviousData[2]));
  
  tick.setValue(HkxUnitConversion::timeRawToValue(behaviousData[3]));
  
  absolutePosition.setValue(HkxUnitConversion::positionRawToValue((behaviousData[7]&3)<<8 | behaviousData[6]));
  
  velocity.setValue(HkxUnitConversion::velocityRawToValue(behaviousData[9]<<8 | behaviousData[8]));
  
  PWM.setValue((behaviousData[11]&3)<<8 | behaviousData[10]);
  
  absoluteGoalPosition.setValue(HkxUnitConversion::positionRawToValue((behaviousData[15]&3)<<8 | behaviousData[14]));
  
  absoluteDesiredTrajectoryPosition.setValue(HkxUnitConversion::positionRawToValue((behaviousData[17]&3)<<8 | behaviousData[16]));
  
  desiredVelocity.setValue(HkxUnitConversion::velocityRawToValue(behaviousData[19]<<8 | behaviousData[18]));
  
  return 0;
}

uint8_t HkxSetup::getStatus(uint8_t ID, HkxStatus& statusED){
  if(ID > HKX_MAX_ID){
    errorPrint(F("getStatus > Input not correct"));
    return 1;
  }

  uint8_t error = _herkXCom.statusRequest(ID, statusED);
  if(error == 2){// no data received
    warningPrint(F("getStatus > servo not connected"));
    return 2;
  }
  if(error == 3){// data received not consistent
    return 3;
  }

  return 0;
}


uint8_t HkxSetup::clearStatus(uint8_t ID){
  if(ID > HKX_ID_ALL){
    errorPrint(F("clearStatus > Input not correct"));
    return 1;
  }
  
  // Set the status to 0
  uint8_t clearedStatus[] = {0x00 , 0x00};
  _herkXCom.writeRequest(ID, HKX_RAM, 48, 2, clearedStatus);
  // Switch off the LED
  _herkXCom.writeRequest(ID, HKX_RAM, 53, 1, 0);
  
  return 0;
}

void HkxSetup::scanIDs(boolean IDs[]){
  for(int i = 0 ; i < 254 ; i++){
    HkxStatus statusED;
    uint8_t error = getStatus(i, statusED);
    if (error == 2){ // not connected
      IDs[i] = false;
    } else {         // connected
      IDs[i] = true;
    }
  }
  
  return;
}

uint8_t HkxSetup::factoryReset(uint8_t ID, boolean IDskip, boolean bandSkip){
  if(ID > HKX_ID_ALL){
    errorPrint(F("factoryReset > Input not correct"));
    return 1;
  }
  
  uint8_t error = _herkXCom.rollbackRequest(ID, IDskip, bandSkip);
  if(error != 0){
    return error;
  }

  return _herkXCom.rebootRequest(ID);
}


