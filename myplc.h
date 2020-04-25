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
//
//
//  DEVICE TYPES  
//
#define HW_NBV30      0x01 // Nanoboard V3 ( RMS Shield + Arduino Nano )
#define HW_ESP01      0x02 // Esp 8266 MyPlc Shield  ( !!! To bee )
#define HW_PC241      0x03 // Nanoboard PIC24xxxx  ( !!! To bee )
//
//
#define _DEV_HARDWARE  HW_NBV30     // Hardware type
#define _DEV_FW_VER           1     // Firmware version
#define _DEV_FW_REL          13     // Release 
#define _DEV_SERIAL          31     // Device serial
#define _DEV_MEMORY           2     // 2 KBytes SRAM (ATMega 328P) 
#define _DEV_EEPROM           1     // 1 KBytes EEPROM (ATMega 328P)
//    
//
//
// Config
//
#undef  _SEROUT_
#undef  _LCDOUT_
#define _LCD_ADDR   0x3F          //  I2C Address:
                                  //  PCF8574P   0100AAA0   000: 0x27 
                                  //  PCF8574AP  0111AAA0   000: 0x3F  
                                  
// 
#define _VerByte(v,r)   (uint8_t)((v<<5)|(r&0x1F))
#define _ByteVer(b)     (uint8_t)(b>>5)
#define _ByteRel(b)     (uint8_t)(b&0x1F)

#endif // MYPLC_H












  
