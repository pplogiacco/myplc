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

#ifndef MYPLC_H
#define MYPLC_H

#define _MYPLC_V         3   // Version
#define _MYPLC_R        12  // Release
//
#define _VerByte(v,r)   (uint8_t)( v<<5|(r & 0x1F) )
#define _ByteVer(b)     (uint8_t)( b>>5|)
#define _ByteRel(b)     (uint8_t)( b & 0x1F )
//
#define _MYPLC_HW_NBV3  0x01 // Nanoboard V3 ( RMS Shield + Arduino Nano )
#define _MYPLC_HW_ESP1  0x02 // Esp 8266 MyPlc Shield
//
#define _MYPLC_HW       _MYPLC_HW_NBV3
#define _MYPLC_FW       _VerByte(_MYPLC_V,_MYPLC_R)

// Config
//
#undef  _SEROUT_
#undef  _LCDOUT_
#define _LCD_ADDR   0x3F          //  I2C Address:
                                  //  PCF8574P   0100AAA0   000: 0x27 
                                  //  PCF8574AP  0111AAA0   000: 0x3F  
#define LOCONET_CONTROLLER  0     //  On Send OPC_SW_REQ otherwise Receive commands
// 


#endif // MYPLC_H












  
