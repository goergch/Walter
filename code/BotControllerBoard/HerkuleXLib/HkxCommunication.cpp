/**
 * \file HkxCommunication.cpp
 * \brief Communication management of the Arduino library for HerkuleX servo (DRS-0101 and DRS-0201 models)
 * \author Benoit Puel (contact@benoitpuel.com)
 * \version 1.0
 * \date 11 December 2015
 * \copyright This project is released under the GNU GPL V3.
 */

#include "HkxCommunication.h"

/*****************************************************************************************************************
**************************************         PRIVATE MEMBRES        ********************************************
*****************************************************************************************************************/


void HkxCommunication::sendPacket(uint8_t ID, hkxCommand CMD, const uint8_t data[], uint8_t dataLength){
  uint8_t packetSize = 7 + dataLength; // Lengh of the request packet
  uint8_t packetSend[packetSize]; // Request packet to send
  
  // Request packet structure
  packetSend[0] = 0xFF; // Header
  packetSend[1] = 0xFF; // Header
  packetSend[2] = packetSize; // Packet Size
  packetSend[3] = ID; // ID of the servo
  packetSend[4] = CMD; // Instruction
  packetSend[5] = checkSum1(packetSize, ID, CMD, data, dataLength); // Checksum1
  packetSend[6] = checkSum2(packetSend[5]); // Checksum2 
  
  // Data
  for(int i = 0 ; i < dataLength ; i++){
    packetSend[7 + i] = data[i];
  }
  
  // send the request packet and wait that the packet is actually sent
  this->_serialServos.write(packetSend,packetSize);  
    
  this->_serialServos.flush();
}


uint8_t HkxCommunication::receivePacket(uint8_t& dataLength){
  // in case the request packet is sent back to arduino
  do{
    reinitPacketRead(); // Clean the array
    
    // time variables
    long startTime = millis(); // start of the timer
    
    // detect the headers
    uint8_t header = 0;
    while( (header < 2) & ((millis() - startTime) < 2*HKX_WAIT_TIME )){
      if (this->_serialServos.available()){
        uint8_t temp = (uint8_t)this->_serialServos.read();
        // Check if it is an header
        if (temp == 0xFF){
          header++;
        } else {
          header = 0;
        }
      }
    }
    
    // If headers not found, 
    if (header < 2){
      // warningPrint(F("receivePacket > Header not found"));
      return 1;
    }
    
    // get the packet size
    startTime = millis();// reinitialise the timer
    uint8_t packetSize = 0;
    while((millis() - startTime) < HKX_WAIT_TIME ){
      if (this->_serialServos.available()){
        packetSize = (uint8_t)this->_serialServos.read();
        break;
      }
    }
    // If packet size not found, 
    if (packetSize == 0){
      // warningPrint(F("receivePacket > Packet size not found"));
      return 2;
    }
    
    //Fill _packetRead
    this->_packetRead[0] = 0xFF;
    this->_packetRead[1] = 0xFF;
    this->_packetRead[2] = packetSize;
    
    // Check enough data is available
    startTime = millis();// reinitialise the timer
    while( (this->_serialServos.available() < (packetSize-3)) & ((millis() - startTime) < packetSize*HKX_WAIT_TIME )){
    }
    // If not enough data is available
    if (this->_serialServos.available() < (packetSize-3)){
      // warningPrint(F("receivePacket > Not enough data available"));
      return 3;
    }
    
    uint8_t data[packetSize-7]; //for the checksum
    for(int i = 3 ; i < packetSize ; i++){
      this->_packetRead[i] = (uint8_t)this->_serialServos.read();
      if(i>6){
        data[i-7] = this->_packetRead[i];
      }
    }
    
    // Check the consistency of the packet integrity
    uint8_t cks1 = checkSum1(this->_packetRead[2], this->_packetRead[3], hkxCommand(this->_packetRead[4]), data, packetSize-7);
    uint8_t cks2 = checkSum2(cks1);
    
    // If the checksums do not match
    if ((cks1 != this->_packetRead[5]) || cks2 != this->_packetRead[6]){
      // warningPrint(F("receivePacket > Checksums do not match"));
      return 4;
    }
    
    
  }while(_packetRead[4] < 0x40);
  
  
  dataLength = _packetRead[2]-9;
  return 0;
}


uint8_t HkxCommunication::readPacket(uint8_t& ID, hkxCommand& CMD, uint8_t data[], uint8_t dataLength, HkxMaybe<HkxStatus> statusED){
  if(_packetRead[0]!=0xFF){
    // warningPrint(F("readPacket > No data received"));
    return 2;
  }
  if( (dataLength + 9) != this->_packetRead[2]){
    // warningPrint(F("readPacket > DataLength is consistent with the packet size"));
    return 1;
  }
  
  ID = this->_packetRead[3];
  CMD = hkxCommand(this->_packetRead[4]);
  for(int i=0 ; i < dataLength ; i++){
    data[i] = this->_packetRead[7+i];
  }
  if(!statusED.isEmpty()){
    uint8_t statusEDByte[2] = {1,1};
    statusEDByte[0] = this->_packetRead[dataLength+7];
    statusEDByte[1] = this->_packetRead[dataLength+7+1];
    statusED.getValue().setRawData(statusEDByte);
  }

  return 0;
}


/*****************************************************************************************************************
***************************************         PUBLIC MEMBRES          ******************************************
*****************************************************************************************************************/


HkxCommunication::HkxCommunication(hkxBaudrate baudrate, HardwareSerial& serialServos, HkxPrint& print) : _serialServos(serialServos), _print(print), _className(F("HkxCommunication::")){
  if(_print._serialPrint == &_serialServos){
    // errorPrint(F("start > The same serial port is used for the servo and the print"));
  }
  _serialServos.begin(baudrate);
}

uint8_t HkxCommunication::readRequest(uint8_t ID, hkxMemory mem, uint8_t start, uint8_t length, uint8_t returnData[], HkxMaybe<HkxStatus> statusED){
  if(HKX_DEV){
    if(ID > 0xFD || mem > 1 || (mem == HKX_ROM && (start+length) > 54) || (mem == HKX_RAM && (start+length) > 74) || length > 52){
      // errorPrint(F("readRequest > input values not correct"));
      return 1;
    }
  }
  
  // Clean the serial buffer to avoid asynchronised packets
  this->cleanSerialBuffer();

  uint8_t numberOfSendTry = 0;
  
  //Send request parameters
  uint8_t sDataLength = 2;
  uint8_t sData[] = {start, length}; 
  hkxCommand sCMD;
  if(mem == HKX_ROM){
    sCMD = HKX_CMD_ROM_READ;
  } else if(mem == HKX_RAM){
    sCMD = HKX_CMD_RAM_READ;
  }
  
  //Receive packet parameters
  uint8_t rDataLength = length + 2;
  uint8_t rData[rDataLength];
  uint8_t returnValue;

  
  do{ // if the packet shall be sent again
    numberOfSendTry++;
    sendPacket(ID, sCMD, sData, sDataLength);

    uint32_t startTime = millis();
    do{ // if the packet shall be read again
      //Receive ACK packet
      returnValue = 0;
      uint8_t rID;
      hkxCommand rCMD;
      uint8_t rrDataLength; 
      uint8_t error = this->receivePacket(rrDataLength);
      if(error == 1){ //headers not found
        returnValue = 2;
        continue;
      }
      if(error > 1){
        returnValue = 3;
        continue;
      }
      if(rrDataLength != rDataLength){
        // warningPrint(F("readRequest > Packet size different than expected"));
        returnValue = 3;
        continue;
      }
      
      error = readPacket(rID, rCMD, rData, rDataLength, statusED);
      if(error == 2){
        returnValue = 2;
        continue;
      }
      if(error == 1){
        returnValue = 1;
        continue;
      }
      if( (rID != ID) || (rCMD != (sCMD+0x40))){
        // warningPrint(F("readRequest > ID or CMD mismatch"));
        returnValue = 3;
        continue;
      }
      if( (sData[0] != rData[0]) || (sData[1] != rData[1]) ){
        // warningPrint(F("readRequest > Address or Length mismatch"));
        returnValue = 3;
        continue;
      }
    }while( ((millis() - startTime) < HKX_WAIT_TIME*(length+7)) && (returnValue == 2));
    
  }while(numberOfSendTry < HKX_NUMBER_SEND_TRIALS && returnValue == 3);
  
  //Return the data
  for(int i=0 ; i<length ; i++){
    returnData[i] = rData[i+2];
  }
  return returnValue;
}


uint8_t HkxCommunication::writeRequest(uint8_t ID, hkxMemory mem, uint8_t start, uint8_t length, const uint8_t writeData[]){
  if(HKX_DEV){
    if(ID > 0xFE || mem > 1){
	  // errorPrint(F("writeRequest > Input not correct"));
      return 1;
    }
  }
  
  //Send request packet
  uint8_t sDataLength = length + 2;
  uint8_t sData[sDataLength];
  sData[0] = start;
  sData[1] = length;
  for (int i=0 ; i < length ; i++){
    sData[i+2] = writeData[i];
  } 

  hkxCommand sCMD;
  if(mem == HKX_ROM){
    sCMD = HKX_CMD_ROM_WRITE;
  } else if(mem == HKX_RAM){
    sCMD = HKX_CMD_RAM_WRITE;
  }
  
  sendPacket(ID, sCMD, sData, sDataLength);
  
  return 0;
}


uint8_t HkxCommunication::statusRequest(uint8_t ID, HkxStatus& statusED) {
  if(HKX_DEV){
    if(ID > 0xFD){
      // errorPrint(F("statusRequest > input values not correct"));
      return 1;
    }
  }
  
  // Clean the serial buffer to avoid asynchronised packets
  this->cleanSerialBuffer();
  
  uint8_t numberOfSendTry = 0;
  
  //Send request parameters
  uint8_t sDataLength = 0;
  uint8_t sData[sDataLength];
  hkxCommand sCMD = HKX_CMD_STAT;
  
  //Receive packet parameters
  uint8_t rDataLength = 0;
  uint8_t rData[rDataLength];
  uint8_t returnValue;
  
  do{ // if the packet shall be sent again
    numberOfSendTry++;
    sendPacket(ID, sCMD, sData, sDataLength);
    
    uint32_t startTime = millis();
    do{ // if the packet shall be read again
      //Receive ACK packet
      returnValue = 0;
      uint8_t rID;
      hkxCommand rCMD;
      uint8_t rrDataLength;
      uint8_t error = this->receivePacket(rrDataLength);
      if(error == 1){ //headers not found
        returnValue = 2;
        continue;
      }
      if(error > 1){
        returnValue = 3;
        continue;
      }
      if(rrDataLength != rDataLength){
        // warningPrint(F("statusRequest > Packet size different than expected"));
        returnValue = 3;
        continue;
      }
      
      error = readPacket(rID, rCMD, rData, rDataLength, &statusED);
      if(error == 2){
        returnValue = 2;
        continue;
      }
      if(error == 1){
        returnValue = 1;
        continue;
      }
      if( (rID != ID) || (rCMD != (sCMD+0x40))){
        // warningPrint(F("statusRequest > ID or CMD mismatch"));
        returnValue = 3;
        continue;
      }

    }while( ((millis() - startTime) < HKX_WAIT_TIME*9) && (returnValue == 2) );

  }while(numberOfSendTry < HKX_NUMBER_SEND_TRIALS && returnValue == 3);

  return returnValue;
}


uint8_t HkxCommunication::iJogRequest(uint8_t length, const uint16_t jog[], const uint8_t set[], const uint8_t ID[], const uint8_t playTime[]){
  if(HKX_DEV){
    for(int i=0 ; i < length ; i++){
      if(ID[i] > 0xFD || playTime[i] > 0xFD){
        // errorPrint(F("iJogRequest > input values not correct"));
        return 1;
      }
    }
  }
  
  //Send request packet
  uint8_t pID = length==1?ID[0]:0xFE;
  hkxCommand sCMD = HKX_CMD_I_JOG;
  uint8_t sDataLength = length*5;
  uint8_t sData[sDataLength];
  for(int i=0 ; i < length ; i++){
    sData[5*i] = (uint8_t)(jog[i] & 0XFF);
    sData[5*i+1] = (uint8_t)((jog[i] & 0XFF00) >> 8);
    sData[5*i+2] = set[i];
    sData[5*i+3] = ID[i];
    sData[5*i+4] = playTime[i];
  }
 
  sendPacket(pID, sCMD, sData, sDataLength);
  
  return 0;
}


uint8_t HkxCommunication::sJogRequest(uint8_t playTime, uint8_t length, const uint16_t jog[], const uint8_t set[], const uint8_t ID[]){
  if(HKX_DEV){
    if(playTime > 0xFD){
      // errorPrint(F("sJogRequest > input values not correct"));
      return 1;
    }
    for(int i=0 ; i < length ; i++){
      if(ID[i] > 0xFD){
        // errorPrint(F("sJogRequest > input values not correct"));
        return 1;
      }
    }
  }
  
  //Send request packet
  uint8_t pID = length==1?ID[0]:0xFE;
  hkxCommand sCMD = HKX_CMD_S_JOG;
  uint8_t sDataLength = length*4+1;
  uint8_t sData[sDataLength];
  sData[0] = playTime;
  for(int i=0 ; i < length ; i++){
    sData[4*i+1] = (uint8_t)(jog[i] & 0XFF);
    sData[4*i+2] = (uint8_t)((jog[i] & 0XFF00) >> 8);
    sData[4*i+3] = set[i];
    sData[4*i+4] = ID[i];
  }
 
  sendPacket(pID, sCMD, sData, sDataLength);
  
  return 0;
}


