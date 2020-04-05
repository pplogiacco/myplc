/* +- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -+  
** |                                                                     |
** |                         Firmware MyPLC                              | 
** |                                                                     |
** +- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -+ 
** |  Railways Modelling System                https://www.rail2rail.eu  |
** +---------------------------------------------------------------------+
**
** Author:  Paolo Lo Giacco ( plogiacco@smartlab.it )
*/

#ifndef MODULES_H
#define MODULES_H

#include "MCP23x17.h" // SPI-I2C 16 Bit Port Expander 

// Loconet
//
//#define LN_TX_PIN   NB_LCNTX    // LN_RX_PIN Hardcoded in library for UNO and MEGA
#if (LOCONET_CONTROLLER)  
  #define LCN_MODE_CONTROLLER    //  Send  OPC_SW_REQ
#endif


// -----------------------------------------------------------------
// ModuleNB30 - NanoBoard 3.0 (Arduino Nano + RMS Shield )  
// -----------------------------------------------------------------
//
#define NB_GPIO1    7         // A2
#define NB_GPIO2    2         // A3
#define NB_GPIO3    A2        // D3
#define NB_GPIO4    A0        // D4
#define NB_GPIO5    9         // D7
#define NB_GPIO6    9         // D9
#define NB_GPIO7    9         // A6
#define NB_GPIO8    9         // A7
//
#define _P1  NB_GPIO1   // Port as Pin
#define _P2  NB_GPIO2
#define _P3  NB_GPIO3
#define _P4  NB_GPIO4
#define _P5  NB_GPIO5
#define _P6  NB_GPIO6
#define _P7  NB_GPIO7
#define _P8  NB_GPIO8
//
//  Hardware
//  Input Pin: close circuit to GND read 1, float pin/open circuit read 0    (Internal pull-up )
//  output Pin 
//
class ModuleNBV30: public Module_ {        // IO Module 16bit, based on Microchip MCP23S17 SPI I/O Expander

  private:
    uint16_t lpinval=0;     // module's last hardware state
    
  public:                                    
    ModuleNBV30(idx_t nidx):Module_(nidx,0) {};   // Costructor        
    ~ModuleNBV30(){};
    uint8_t Initialize();
    port_t  mapPort(flag_t io, saddr_t subaddr, flag_t ptype=0 );
    uint8_t readPorts(port_t* _pi, idx_t _n);  
    uint8_t writePorts(port_t* _po, idx_t _n);

};




// -----------------------------------------------------------------
// ModuleSEM16 - Microchip MCP23S17 SPI I/O Expander  
// -----------------------------------------------------------------
//
//  SPI Expansion Bus Connector
//  1---+---+---+---+---+---+
//  |GND|SO |SI |CK |SS |VCC|
//  +---+---+---+---+---+---+
//
//
//
#define _B1 7   // mcp23x17 Bank B
#define _B2 6
#define _B3 5
#define _B4 4
#define _B5 3
#define _B6 2
#define _B7 1
#define _B8 0
//
#define _A1 8    // mcp23x17 Bank A
#define _A2 9     
#define _A3 10     
#define _A4 11     
#define _A5 12     
#define _A6 13     
#define _A7 14     
#define _A8 15     
//
typedef uint16_t reg_t;    // MCP23S17 register  
//
class ModuleSEM16: public Module_ {        // IO Module 16bit, based on Microchip MCP23S17 SPI I/O Expander

  private:
    MCP23x17* sem16;
    reg_t imsk = 0;     // module's pin map ( 2 banks x 8 pin , 0=Out 1=In )
    reg_t lval = 0;     // module's last hardware state
    
      
  public:
    ModuleSEM16(idx_t nidx, maddr_t maddr):Module_(nidx, maddr)   // Costructor,  1..7 masked to bbaaaxxx  ( bb=00 SIP )
    {
      sem16 = new MCP23x17(Module_::addr, NB_CSPIN);     // Init SPI module 
    };   
    ~ModuleSEM16(){};

    uint8_t Initialize();
    port_t mapPort(flag_t io, saddr_t subaddr, flag_t ptype=0 );
    uint8_t readPorts(port_t* _pi, idx_t _n);  
    uint8_t writePorts(port_t* _po, idx_t _n);
};





// -----------------------------------------------------------------
// ModuleLCNIF Module (Loconet Interface)  
// -----------------------------------------------------------------
//
//
//#define LN_TX_PIN   NB_LCNTX    // LN_RX_PIN Hardcoded in library for UNO and MEGA
//#define LCN_MODE_CONTROLLER   1      //  Send  OPC_SW_REQ
//#define LCN_ADDR_OFFSET       000U   //  offset to +/- to loconet addresses
//
//
// Decoder Mode:
// decode OPC_SW_REQ, OPC_SW_REP for In ports, send OPC_INPUT_REP for Out port
//     
// Controller Mode:
// decode OPC_INPUT_REP, OPC_SW_REP for In ports, send OPC_SW_REQ and OPC_INPUT_REP for Out port
//
// loconet opcodes:
//
#define OPC_SW_REQ        0xb0  // REQ SWITCH function
#define OPC_SW_REP        0xb1  // Turnout SENSOR state REPORT
#define OPC_INPUT_REP     0xb2  // General SENSOR Input codes
#define OPC_SW_STATE      0xbc  // 
#define OPC_SW_ACK        0xbd  //
//
#define OPC_UNKNOWN       0xb3
#define OPC_LONG_ACK      0xb4
#define OPC_BUSY          0x81    
#define OPC_GPOFF         0x82
#define OPC_GPON          0x83
#define OPC_IDLE          0x85
//
#define OPC_RQ_SL_DATA    0xbb
//
#define OPC_MULTI_SENSE   0xd0
#define OPC_PEER_XFER     0xe5
#define OPC_SL_RD_DATA    0xe7
#define OPC_IMM_PACKET    0xed
#define OPC_IMM_PACKET_2  0xee
#define OPC_WR_SL_DATA    0xef
//
//#define OPC_MASK          0x7F  // mask for acknowledge opcodes 
//


class ModuleLCN: public Module_ {        // Module IO16 implementation
private:
public: 
   ModuleLCN(idx_t nidx, maddr_t maddr):Module_(nidx, maddr)  {};     // Class constructor
   ~ModuleLCN() {};
   
  uint8_t Initialize();
  port_t mapPort(flag_t io, saddr_t subaddr, flag_t ptype=0);
  
  uint8_t readPorts(port_t* _pi, idx_t _n);  
  uint8_t writePorts(port_t* _po, idx_t _n); 
};




#endif // MODULES_H













  
