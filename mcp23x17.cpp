#undef _XDEBUG_
#include "mcp23x17.h"

void MCP23x17::setPinmap(word pinmap) {
  #ifdef _XDEBUG_
  Serial.println("@MCP23x17::setPinmap()");
  //Serial.print("PortA (MSB) = ");Serial.println((byte)(value>>8),BIN);
  //Serial.print("PortB (LSB) = ");Serial.println((byte)value,BIN);
  #endif    
  setRegW(_IODIR,pinmap);
}

void MCP23x17::setPullup(word pullup) { 
  setRegW(_GPPU, pullup);
//  _pullup = pullup;
}

void MCP23x17::setInvert(word invert) { 
  setRegW(_IPOL, invert);
//  _invert = invert;
}


// Read & Write 

word MCP23x17::pinRead(void) {   
  #ifdef _XDEBUG_
  Serial.println("@MCP23x17::pinRead()");
  Serial.print("PortA (MSB) = ");Serial.println((byte)(value>>8),BIN);
  Serial.print("PortB (LSB) = ");Serial.println((byte)value,BIN);
  #endif  
 return getRegW(_GPIO);
}

byte MCP23x17::pinRead(byte pin) {       
  if (pin < 1 | pin > 16) return 0x0;                // If the pin value is not valid (1-16) 
  return getRegW(_GPIO) & (1 << (pin - 1)) ? 1 : 0;  // Call the word reading function, extract HIGH/LOW information from the requested pin
}

void MCP23x17::pinWrite(word output) { 
  #ifdef _XDEBUG_
  Serial.println("@MCP23x17::pinWrite()");
  //Serial.print("PortA (MSB) = ");Serial.println((byte)(value>>8),BIN);
  //Serial.print("PortB (LSB) = ");Serial.println((byte)value,BIN);
  #endif   
  setRegW(_GPIO, output);
 // _output = output;
}



// **** LOW LEVEL ****


byte MCP23x17::init() {
 const byte _IOCON_ = (_IOCON_HAEN | _IOCON_SEQOP);
  if ( _cs ) { // SPI select
   ::pinMode(_cs, OUTPUT);               // Set SlaveSelect pin as an output
   ::digitalWrite(_cs, HIGH);            // Set SlaveSelect HIGH (chip de-selected)
   SPI.begin();                          // Start up the SPI bus
   //SPI.setClockDivider(CLOCK_DIVIDER); // Sets the SPI bus speed
   //SPI.setBitOrder(MSBFIRST);          // Sets SPI bus bit order (this is the default, setting it for good form!)
   //SPI.setDataMode(SPI_MODE0);         // Sets the SPI bus timing mode (this is the default, setting it for good form!)
   ::digitalWrite(_cs, LOW);             // Take slave-select low
    SPI.transfer(OPCODEW | _address );   // Send the MCP23S17 opcode, chip address, and read bit
    SPI.transfer(_IOCON);                // Send the address 
    SPI.transfer(_IOCON_);               // Send 8bits register's value  
   ::digitalWrite(_cs, HIGH);            // Take slave-select high      
 
  } else { // i2c bus
     
   Wire.begin();
   delay(2);
   Wire.beginTransmission( _address );
   Wire.write(_IOCON);                     // Send the address 
   Wire.write(_IOCON_);                    // Send 8bits register's value  
   Wire.endTransmission(); 
   
  };
   
  byte _iocon_ = (byte)getRegW(_IOCON);  // verifing...
  // setRegW(_IODIR,_pinmap);            // Call the word writer 
  
  #ifdef _XDEBUG_
  Serial.print("addr = ");Serial.println(_address,BIN );
  Serial.print("IOCON= ");Serial.println((_IOCON_),BIN);
  Serial.print("bus  = ");Serial.println((_cs)?"SPI":"I2C");
  Serial.print("ready= ");Serial.println(_iocon_ == _IOCON_);
  #endif
  return (_iocon_==_IOCON_);  // ret >0 if ok..
}


word MCP23x17::getRegW(const byte reg) {
    word value = 0;
    #ifdef _XDEBUG_
    Serial.print("@getRegW (SPI,cs=");
    Serial.print(_cs);
    Serial.println(")");
    #endif   
    
    if ( _cs ) {                     // SPI
      ::digitalWrite(_cs, LOW);                  // Take slave-select low
      SPI.transfer(OPCODER | _address);          // Send the MCP23S17 opcode, chip address, and read bit
      SPI.transfer(reg);                         // Send the address we want to read
      
      value = SPI.transfer(0x00);        // Read "high byte" (portA) and shift it up to the high location
      value = (value<<8) | SPI.transfer(0x00);                // Read "low byte" (portB) (register address pointer will auto-increment after write)
      ::digitalWrite(_cs, HIGH);                 // Take slave-select high        
      
    } else {                                    // I2C
      Wire.beginTransmission(_address);      //need to set up the proper register before reading it
      Wire.write(reg);                          // Even though we are doing a read, we first have to write the address of the register the read will be from
      Wire.endTransmission();
      Wire.requestFrom(_address,(byte)2);    //Now we start the actual read
      while (Wire.available()==0);              //wait unit the data gets back
      value = Wire.read();
      value = (value<<8) | Wire.read();
      Wire.endTransmission();
    }
      #ifdef _XDEBUG_
      Serial.println("Read:");
      Serial.print("PortA (MSB) = ");Serial.println((byte)(value>>8),BIN);
      Serial.print("PortB (LSB) = ");Serial.println((byte)value,BIN);
      #endif    
      
 return value;
}


void MCP23x17::setRegW(const byte reg, word value) { //, byte lbuf) {  
    if ( _cs ) {                     // spi
      ::digitalWrite(_cs, LOW);                            // Take slave-select low
      SPI.transfer( (OPCODEW |_address));                  // Send the MCP23S17 opcode, chip address, and write bit
      SPI.transfer(reg);                                   // Send the register we want to write
      SPI.transfer((byte)(value>>8));                      // Send the MSB byte     
      SPI.transfer((byte)value);                           // Send the LSB byte 
      ::digitalWrite(_cs, HIGH);                           // Take slave-select high
    } else {                         // i2c
      Wire.beginTransmission(_address);  //all I2C commands begin with this statement and the address of the chip.
      Wire.write(reg);            // select a write to the IODIRA register
      Wire.write(value>>8);        // 
      Wire.write((byte)value);     //
      Wire.endTransmission();     // all I2C commands must end with this statement  
    }
      #ifdef _XDEBUG_
      Serial.println("Write:");
      Serial.print("PortA (MSB) = ");Serial.println((byte)(value>>8),BIN);
      Serial.print("PortB (LSB) = ");Serial.println((byte)value,BIN); 
      #endif  
}


