/* +- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -+  
** |                                                                     |
** |                         Firmware MyPLC                              | 
** |                                                                     |
** +- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -+ 
** +---------------------------------------------------------------------+
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












  
