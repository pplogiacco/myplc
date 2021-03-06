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

#ifndef CORE_H
#define CORE_H

#define DEBUG_FORCE_DEFAULT 

#include "myplc.h"

extern void Beep();
// Error codes
//
#define E00 0x00    // No error
#define Eok E00
//
#define E01   0x01    // Generic fault !
#define E02   0x02   // Memory fault !
#define E03   0x03  // Hardware fault !

#define EEPROM_OFFSET  0x00

// Bitwise utils
//
#define _LSB(w)  ((uint8_t) ((w) & 0xff))
#define _MSB(w)  ((uint8_t) ((w) >> 8U))
//#define _BV(n)   (1 << (n)) 
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define pinToggle(pin) PINB ^= 1UL _BV();



// ---------------------------------------------------------------------------------------------
// RMS Project                                                         N A N O B O A R D   v 3.0
// ---------------------------------------------------------------------------------------------
//
//  SPI Bus  ( SEM Modules )
//  1---+---+---+---+---+---+
//  |GND|SO |SI |CK |SS |VCC|
//  +---+---+---+---+---+---+
//       D12 D11 D13 D10 
//
//  Serial
//  1---+---+---+
//  |TX |GND|RX |
//  +---+---+---+
//   D1      D0  
//
//  I2C LCD 
//  1---+---+---+---+
//  |GND|CA |CL |VCC|
//  +---+---+---+---+
//       A4  A5  
//
//  SPI Expansion Bus
//  1---+---+---+---+---+---+
//  |GND|SO |SI |CK |SS |VCC|
//  +---+---+---+---+---+---+
//       D12 D11 D13 D10 
//
//
//  User Controls
//  1---+---+---+---+
//  |GND|LOK|RUN|GND|
//  +---+---+---+---+
//        
//
#define NB_CSPIN    10        // SS pin SPI bus 
#define NB_LCNTX    6         // Loconet
#define NB_DCCIN    2         // DCC In
#define NB_BUZZER   5         // Buzzer
#define NB_SW1      4         // A0 ( Keylock  Analog )
#define NB_SW2      3         // A1 ( Execute  Analog )


// ---------------------------------------------------------------------------------------------
// RMS Project                                                       M O D U L E S  &  P O R T S
// ---------------------------------------------------------------------------------------------
//
//
#define On   0x01         // Port value On ( +5V / Circuit closed ), Off ( 0V / Circuit open )
#define Off  0x00
//
#define Yes   On         // Port changed Yes/Not
#define Not   Off         
//
#define In    On          // I/O port data direction In/Out
#define Out   Off 
//
#define Act   On          // Controlled device Sns  Act ( 1 actuator), (0 sensor)
#define Sns   Off 
//
#define InitOK  E00       // Module Init ok
#define InitERR E01       // Module initialization fail
//
//
typedef uint16_t port_t;  // 16bit Port
//
//  16bit Port addressing 2^10 logic/phisical gates x 2^3 modules = 8192 I/O
//  
//                       MSB                      LSB
//              +------------------+-----+---+-------+---+---+
//              | a a a a a a a a  | a a | t | m m m | v | c |
//              +-----^------------+-^---+-^-+---^---|-^-|-^-+ 
//                    |              |     |     |     |   |
//                    lsb           msb    |     |     |  changed      
//                    8bit          2bit   type  |    value   
//                    address       addr        module-idx      
//                                             
//
// Generic Port Managing Utils
//
#define _PNew(m, a, t)  (((port_t)(a))<<8)|(((port_t)(t))<<5)|(((port_t)(m))<<2)|0x01  
#define _PFNew(m, a,t)  (((a&0x300)>>2)|(a<<8))|(((port_t)(t))<<5)|(((port_t)(m))<<2)|0x01  
//
#define _PChg(p)     (port_t)(p & 0x01)                   // Port value changed
#define _PsChg(p,s)  ((s)?p|0x0001:p&0xFFFE)              // Set changed to 0/1
//
#define _PVal(p)     (port_t)( (p & 0x02)>>1 )            // Port value
#define _PsVal(p,v)  ((v)?p|0x0002:p&0xFFFD)              // Set value ( On / Off ) AND Changed
#define _PscVal(p,v) ((v)?p|0x0002:p&0xFFFD)|0x0001       // Set value ( On / Off ) AND Changed
//
#define _PMod(p)     (port_t)( (p & 0x1C)>>2 )            // Module  0..15 
#define _PsMod(p,m)  (p&0xFFE3)|((((port_t)(m))<<2))      // Set module  (3 bit 8 module) 
//
#define _PTyp(p)     (port_t)( (p & 0x20)>>5 )            // Port Type
#define _PsTyp(p,t)  ((v)?p|0x0020:p&0xFFDF)              // Set Port Type 1 bit (0 sns, 1 act )
//
#define _PAdr(p)     (port_t)(p>>8)                       // Address ( short only 8bit )
#define _PsAdr(p,a)  (p&0x00FF)|(((port_t)a)<<8)          // Set short s-address (8bit)  
//
#define _PFAdr(p)     (port_t)((p&0x00C0)<<2)|(p>>8)      // Full 10 Bit Address  
#define _PsFAdr(p,a)  (p&0x003F)|(((a&0x0300)>>2)|(a<<8)) // Set 10 bit Address 
//
//
typedef uint16_t  maddr_t;   // Module phisical address ( SPI, i2c, Loconet... )
typedef uint16_t  saddr_t;   // Port sub-address ( 10bit max, expander gate, pin, lcn-addr, etc.  )
typedef uint8_t   flag_t;    // smaller data only one bit !
typedef uint8_t   idx_t;     // Port/Module index in array
//
#define _MOpt1(o) ( o    & 0xF )
#define _MOpt2(o) ( o>>4 & 0xF )
//
//
typedef enum : uint8_t {          // Bus & Module:
                  _NBV30 = 0x01,     // Nanoboard 3.0 
                  _SEM16 = 0x02,     // SPI I/O Port Expander
                  _LCNIF = 0x03      // Loconet Interface
                       } module_t;
//
//
class Module_ {         // Abstract class 
  protected:
         idx_t idx  = 0;      // (Module ID) position in _MO array
       maddr_t addr = 0;      // phisical addres
        flag_t opts = 0;      // Options 4bits lsn/msn
     
  public:
    Module_(idx_t nidx, maddr_t maddr, flag_t mopts):idx(nidx),addr(maddr),opts(mopts) {};
    ~Module_() {}; 
    
    virtual uint8_t Initialize() {};    // Return E00 = InitOK / E01 = InitERR
    virtual port_t  mapPort(flag_t io, saddr_t subaddr, flag_t ptype=0) {}; // 
    virtual uint8_t readPorts(port_t* _pi, idx_t _n) {}; 
    virtual uint8_t writePorts(port_t* _po, idx_t _n) {};
};






// ---------------------------------------------------------------------------------------------
// RMS Project                                                                             RULES
// ---------------------------------------------------------------------------------------------
// V2
//
typedef struct {
    uint8_t   rl;         // rule condition coding
    idx_t     pin;        // index in cIN table 
    idx_t     pout;       // index in cOUT table 
} rule_t;
// 
//   Rule (1 byte + 2 x idx_t ) 
//
//   |Condition| Values  | idx InPort |idx OutPort |    
//   +---------+---------+------------+------------+
//   | x x c c | v v s s | .......... | .......... |
//   +---------+---------+------------+------------+  
//
//
//   |Condition| Values  |
//
//   +---------+---------+
//   | x x c c | v v s s |
//   +---------+---------+ 
//
//    xx - nd ( da utilizzare per regole con op logici and/or ) 
//    cc - condition: 00 - set Out=In
//                    01 - if pIn equ vv -> pOut=ss
//                    10 - if pIn neq vv -> pOut=ss
//                    11 - nd
//
//    vv - value to compare pIn
//    ss - value to set to pOut
//
//
#define _CCMask 0x30
#define _Cxx    0x00   // Imposta uscita allo stesso valore di ingresso 
#define _Cnx    0x01   // Imposta uscita allo stesso valore di ingresso 
//
#define _Ceq    0x10   // Imposta uscita a valore ss ingresso corrisponde a vv
#define _Cne    0x20   // Imposta uscita a valore se ingresso non corrisponde a ss 
//
//
#define _Rule(c,v,s) (byte)( (c<<4)|((v & 0x03)<<2)|(s & 0x03) )
#define _RuleEQ(v)   (byte)( ((v & 0x01)<<4)|((v & 0x03)<<2)|(v & 0x03) ) 
//
#define _RuleCC(r)     ((r>>4) & 0x03)    // Condition bits 
#define _RuleSCC(r,v)  ((v<<4) | ( r & 0xCF) )    // Condition bits 
#define _RuleVV(r)     ((r>>2) & 0x03)    // Value to check
#define _RuleSS(r)     (r & 0x03)         // Value to set 
//






// ---------------------------------------------------------------------------------------------
// RMS Project                                                                             P L C
// ---------------------------------------------------------------------------------------------
// V3
//
typedef struct { 
                Module_* mod;
                 maddr_t maddr;      // phisical addres DUPLICATED IN MODULE CLASS
                module_t mtype;
                  flag_t mopts;   
               } mnode_t;
//
//
class MyPlc {
  
 private:
  mnode_t*  _MO;  // Modules  
  port_t*   _PI;  // Input ports
  port_t*   _PO;  // Output ports
  rule_t*   _RL;  // Rules
  idx_t     nMO,  // number of modules
            nPI,  // number of inputs
            nPO,  // number of outputs
            nRL;  // number of rules


  uint8_t  rtime=0;   // 0=no changes, 1=in chaged, 2= evaluation chage out 
  
  bool checkEprom();
  void loadEprom();
  void loadDefault();
  void clearMemory();
  void initMemory(idx_t maxmod,idx_t maxin,idx_t maxout,idx_t maxrls);
   
 public:
 
  MyPlc();
  ~MyPlc();

  // Configution 
  idx_t   addWiringModule(module_t mtype, maddr_t maddr = 0, flag_t mopts = 0);        // Map hardware module
  idx_t   addPort(flag_t io, idx_t imodule, saddr_t subaddr, flag_t ptype=0 , flag_t dval=0);     
  uint8_t addRule(uint8_t cc, idx_t pin, uint8_t vin, idx_t pout, uint8_t vout);
  uint8_t addRule(uint8_t cc, idx_t pin, idx_t pout );
  uint8_t addRule(idx_t pin, idx_t pout )  { return addRule(_Cxx,pin,pout );  }; 

  // Serial and memory functions
  void checkSerial();
  void sendINFO();  
  void sendWDatas();
  void sendCOMPLETE();
  bool saveToEprom();
  
  void updateCycle(long period);

  
  
  // PLC's main functions
  uint8_t Init();        // Initialize all modules hardware & Bus, ret 0 se ok
  uint8_t Read();        // Update in-ports from local Inputs & check loconet incoming packets
  uint8_t Evaluate();    // Evaluate logic rules and update out-ports
  uint8_t Write();       // Update hardware pins and send loconet packets
          
}; // MyPlc 


// ---------------------------------------------------------------------------------------------
//                                                                 S E R I A L   P R O T O C O L
// ---------------------------------------------------------------------------------------------
//
//

typedef uint16_t word_t;         // short unsigned int

typedef enum : word_t            // short unsigned int
        {          
        _INFO     = 0x00,         // Clear uC memory
        _RETRIEVE = 0x01,         // uC -> PC, uC send back all configuration datas
        _REWRITE  = 0x02,         // uC Send infos
        _MODULE   = 0x03,         // module datas ( used by _RETRIEVE / _UPDATE )    
        _PORT     = 0x04,         // port datas  
        _RULE     = 0x05,         // rule datas
        _COMPLETE = 0x06          // Data exchange finished  
        } _vs_cmd_t;

typedef struct {       // 20 Bytes
    word_t hardware;   // Board type
    word_t firmware;   // Firmware version
    word_t serial;     // Serial
    word_t memory;     // Available RAM memory (Kb)
    word_t eprom;      // Available eprom (Kb)  
    word_t cycle;      // Cycle time ms                   
    word_t modules;    // Modules
    word_t p_in;       // n Ports
    word_t p_out;      // n Ports    
    word_t rules;      // Rules       
        } vs_info_t;    

typedef struct {          // MODULE
    word_t idx;           // index
    word_t mtype;     
    word_t maddr;
    word_t mopts;          // !!!! Splitted from mtype
    // word_t fpi;         // !!!! First in PI[]
    // word_t npi;         // !!!! N port in PI[]
    // word_t fpo;         // !!!! First in PO[] 
        } vs_module_t;

typedef struct {          // PORT ( 7Bytes)
      word_t idx;         // index
      word_t imodule;    
      word_t subaddr;      
      word_t io;
      word_t ptype;
      word_t defval;      
        } vs_port_t;

typedef struct {        // RULE
     word_t idx;        // index
     word_t rl;         // rule condition coding 
     word_t ipev;        // index innput port 
     word_t ipac;       // index output port
                          // !!! 2 bit to set i/o for each one 
     //word_t pin;        // !!! index innput port 
     //word_t pout;       // !!! index output port 
               } vs_rule_t;


typedef struct {        ///  !!!!!  USE INFO_____________________
     word_t mode;    
     word_t nmo;       // rule condition coding
     word_t npi;       // index innput port 
     word_t npo;       // index output port 
     word_t nrl;
               } vs_rewrite_t;
#define _VS_REWRITE_BEGIN   1
#define _VS_REWRITE_END     0

               
typedef struct {                // PACKET ( MAX 22 Bytes ) 
  _vs_cmd_t cmd;                // 
    union {
          vs_module_t module;   // 
            vs_port_t port;     // 
            vs_rule_t rule;     // 
            vs_info_t info;     // 20 bytes
         vs_rewrite_t rewrite;  // 
          };
               } vs_pkt_t;
        
#define _VS_PKT_SIZE        22   // 20+2


#endif // CORE_H




  
