/* +- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -+  
** |                                                                     |
** |                         Firmware MyPLC                              | 
** |                                                                     |
** +- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -+ 
** |  Railways Modelling System                https://www.rail2rail.eu  |
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
//
// Ver.Rel changes 
// ----------------------------------------------------------------------
//  1.13 - 11/04/20 - Added module's options 1/2 
//         07/03/20 - Serial Rewrite
//         06/03/20 - Eprom save/load
//         22/02/20 - Serial protocol & refactory
//         10/02/20 - Serial port handshake 
//         09/02/20 - Lcd addressing and test
//         31/01/20 - Refactoring to unified port mask  
// 
//
#include "myplc.h"
#include "core.h"
//
//
#undef _DEBUG_



//  Beep
//
void Beep() {
 digitalWrite(NB_BUZZER,1);
 delay(40);
 digitalWrite(NB_BUZZER,0);
}


// Display  
// 
#ifdef _LCDOUT_

#define _L1  1;             // Update Line 1
#define _L2  2;
#include <LCD.h>
#include <LiquidCrystal_I2C.h> 
/* #define BACKLIGHT_PIN   3  */
LiquidCrystal_I2C lcd(LCD_ADDR,2,1,0,4,5,6,7);  // Pins: En Rw Rs D4 D5 D6 D7

void Show(const char* txt, uint8_t l=0) { // l=0 clear before print, 1..4 print on line...
  if(l) {   
    //lcd.gotoxy 
    //padline   
  } else { // clear
    lcd.clear();
    lcd.home();   
  }
 lcd.print(txt); 
}
#endif


MyPlc myplc;

bool    _visio = false;
uint8_t _cycle = 5;
long    _ltime;


void setup() {

    pinMode(NB_BUZZER, OUTPUT);
    pinMode(NB_LCNTX,  OUTPUT);

    #ifdef _LCDOUT_               // LCD 
    lcd.begin (20,4);        
    lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
    lcd.setBacklight(LED_ON);
    #endif


    Serial.begin(57600,SERIAL_8N1);
    while ((!Serial.available()) && (--_cycle)) { delay(100); };  // wait for incoming byte:
     if (Serial.available() ) {
          char pktbuf = Serial.read();
          _visio = (pktbuf=='A')?On:Off;
          _cycle = 2;  // n-times to compute cycle period 
     }
     
    if( ! _visio ) {
      myplc.Init();     // ret 0 se ok
    };
      
    #ifdef _LCDOUT_  
    Show("MyPlc");
    #endif
    
    Beep();
}




void loop() {
                  
    if ( _visio ) {                // _____Serial link

      if ( _cycle ) {                // Compute Cycle Time
       if (_cycle==1) {
         myplc.updateCycle(millis()-_ltime);
         myplc.sendINFO();          
       }
        _ltime = millis();
       _cycle--;  
                 
      } 
      myplc.checkSerial(); 
                
    } else {                       // _____Run 
      
      if (digitalRead(NB_SW1)) {   // SW2 Pin                                    
        if ( myplc.Read() )         // read in-ports / loconet bus
        if ( myplc.Evaluate() ){    // evaluate rules 
              myplc.Write();        // write out-pins and send loconet messages
        }   
      }
    }  
 
} // loop
