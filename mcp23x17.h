/* +- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -+  
** |                                                                     |
** |                         Firmware MyPLC                              | 
** |                                                                     |
** +- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -+ 
** |                                                                     |
** |                Microchip MCP23S17 SPI I/O Expander                  | 
** |                Microchip MCP23O17 I2C I/O Expander                  |
** |                                                                     |
** +- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -+ 
**
** Author:  Paolo Lo Giacco ( plogiacco@smartlab.it )
**
** BSD License Usage
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution  and use in  source and binary forms, with or  without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of  source code  must  retain the above  copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form  must reproduce the above copyright
**     notice, this  list  of conditions  and the following disclaimer in
**     the  documentation  and/or  other   materials  provided  with  the
**     distribution.
**   * Neither   the   name   of   Author   nor  the    names    of   its
**     contributors  may  be used  to endorse or promote products derived
**     from this software without specific prior written permission.
**
** THIS  SOFTWARE IS  PROVIDED  BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS"  AND  ANY  EXPRESS  OR IMPLIED  WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE  ARE  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL,  EXEMPLARY,  OR  CONSEQUENTIAL  DAMAGES  (INCLUDING,  BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY  OF  LIABILITY, WHETHER  IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE)  ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
*/

#include <Arduino.h>
#include <avr/io.h>

#include <SPI.h>
#include <Wire.h>

#ifndef MCP23x17_h
#define MCP23x17_h
                             
                                       // Address "0 1 0 0 A A A X", A address bit, X read/write flag  
#define     OPCODEW      (0b01000000)  // Opcode for MCP23S17 set LSB's bit 0 to write (0), address OR'd in later, bits 1-3
#define     OPCODER      (0b01000001)  // Opcode to read (1), 
                                                
#define     _IOCON         0x0A        // MCP23x17 Configuration Register 
#define     _IOCON_BANK1  (0b10000000) // (bit 7) Default BANK=0, the A/B registers are paired
#define     _IOCON_MIRROR (0b01000000) // (bit 6) MIRROR bit controls how the INTA and INTB pins function with respect to each other.
#define     _IOCON_SEQOP  (0b00100000) // (bit 5) Sequential Operation (SEQOP) controls the incrementing function of the Address Pointer after each byte is clocked
#define     _IOCON_DISSLW (0b00010000) // (bit 4) Slew Rate (DISSLW) bit controls the slew rate function on the SDA pin.
#define     _IOCON_HAEN   (0b00001000) // (bit 3) Hardware Address Enable (HAEN) bit enables/disables hardware addressing on the MCP23S17 only.
#define     _IOCON_ODR    (0b00000100) // (bit 2) Open-Drain (ODR) control bit enables/disables the INT pin for open-drain configuration. Erasing this bit overrides the INTPOL bit.
#define     _IOCON_INTPOL (0b00000010) // (bit 1) Interrupt Polarity (INTPOL) sets the polarity of the INT pin. (1 = Active-high.)

#define  _GPIO    0x12           // Data Register read data from or write output data
 
                                 // MCP23x17 port & pins
#define  _IODIR   0x00           // Direction register. Write a 0 to make a pin an output or 1 to make it an input
#define  _IODIRA  0x00
#define  _IODIRB  0x01

#define  _IPOL    0x02           // Input Polarity Register. 1=GPIO register bit will reflect the opposite logic state of the input pin.
#define  _GPPU    0x0C           // Pull-up resistors on pin, 1=pullup enabled (100 kΩ resistor)
#define  _OLAT    0x14           // Output Latch Register 1 = Latch High, 0 = Latch Low (default) Reading Returns Latch State, Not Port Value!
                                      
                                  // MCP23x17 Interrupt 
#define  _GPINTEN 0x04           // GPINTENA – INTERRUPT-ON-CHANGE PINS (ADDR 0x02)  on Change Pin Assignements 
#define  _DEFVAL  0x06           // MCP23x17 Default Compare Register for Interrupt on Change
#define  _INTCON  0x08           // MCP23x17 Interrupt on Change Control Register
#define  _INTF    0x0E           // Interrupt Flag Register READ ONLY: 1 = Pin's Int Triggered
#define  _INTCAP  0x10           // READ ONLY: State of the Pin at the Time the Interrupt Occurred


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MCP23017 internal registers - See the Microchip MCP23017 datasheet for full list
//
//      byte  IOX_BASE_ADR = 0x23;     //  A0=+5,A1=+5,A2=GND, 0x20 all set to zero (grounded)
//const byte  IOCONA = 0x0A;           // Bank = 0  (ADDR: 0x05 Bank=1,0x0A Bank=0 )
//const byte  IOCONB = 0x0B;           // Bank = 0  (ADDR: 0x05 Bank=1,0x0A Bank=0 )
//const byte  IODIRA = 0x00;           // Port A direction register. Write a 0 to make a pin an output, a 1 to make it an input
//const byte  IODIRB = 0x01;           // Port B direction register. Write a 0 to make a pin an output, a 1 to make it an input
//const byte  GPPUA =  0x0C;           // pull up resistors on Port A. 1 = pull up enabled (100 kΩ resistor) 
//const byte  GPPUB =  0x0D;           // pull up resistors on Port A. 1 = pull up enabled (100 kΩ resistor) 
//const byte  GPIOA  = 0x12;           // Register Address of Port A - read data from or write output data to this port
//const byte  GPIOB  = 0x13;           // Register Address of Port B
//const byte  GPINTENA = 0x04;         // GPINTENA – INTERRUPT-ON-CHANGE PINS (ADDR 0x02)  
//const byte  GPINTENB = 0x05;
//
// IPOL – INPUT POLARITY PORT REGISTER (ADDR 0x01)
// INTF – INTERRUPT FLAG REGISTER (ADDR 0x07)   // interrogare per sapere quale pin ha generato l'Int
//* The INTCON register controls how the associated pin value is compared for the interrupt-on-change feature.
//    If a bit is set, the corresponding I/O pin is compared against the associated bit in the DEFVAL register. 
//    If a bit value is clear, the corresponding I/O pin is compared against the previous value.                 */
// INTCON – INTERRUPT-ON-CHANGE CONTROL REGISTER (ADDR 0x04)
// DEFVAL - DEFAULT COMPARE REGISTER FOR INTERRUPT-ON-CHANGE
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Tools
// #define _BV(bit) (1 << (bit))  // Converts a bit number into a byte value.


class MCP23x17 {                  // Microchip MCP23S17 SPI I/O Expander RMS Library

  private:
    byte _cs=0;                     // chip-select pin *** 0 = i2c bus ***  
    byte _address;                  // Address of the MCP23S17 in use
    //word _pinmap;                 // Caches the mode (input/output) configuration of I/O pins
    //word _pullup;                 // Caches the internal pull-up configuration of input pins (values persist across mode changes)
    //word _invert;                 // Caches the input pin inversion selection (values persist across mode changes)
    //word _output;                 // Caches the output pin state of pins
    
    word getRegW(byte reg);               // Read 16bit register
    void setRegW(byte reg,word value);    // Write16bit register

  public:
    MCP23x17(void) {};                            // Basic constructor
    MCP23x17(byte hwaddr)                         // I2C: address 0-7
            { _address=(hwaddr<<1);};        
    MCP23x17(byte hwaddr, byte cspin)             // SPI: address 0-7, chip select pin
            { _address=(hwaddr<<1);_cs=cspin;}
            
    byte init();                        // return > 0 device ready
    byte init(byte hwaddr){             // start I2C 
      _address=(hwaddr<<1);_cs=0; 
      return init();      
    };
    byte init(byte hwaddr,byte cspin){  // start SPI 
      _address=(hwaddr<<1); _cs=cspin;
      return init();
    };
    
    void setPinmap(word);           // Set pin direction
    void setPullup(word);           // Selects internal 100k input pull-up of all I/O pins at once 
    void setInvert(word);           // Selects input state inversion of all I/O pins at once (writing a 1 turns on inversion)
    
    void pinWrite(word);              // Sets all output pins at once. If some pins are configured as input, those bits will be ignored on write
    void pinWrite(byte, byte);        // Sets an individual output pin HIGH or LOW
      
    word pinRead(void);               // This function will read all 16 bits of I/O, and return them as a word in the format 0x(portB)(portA)    
    byte pinRead(byte);               // Reads an individual input pin
    
};


#endif //MCP23x17
