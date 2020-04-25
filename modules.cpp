#include <Arduino.h>
#include <avr/io.h>
#include "core.h"
#include "modules.h"

#undef _SDEBUG_
#undef _LDEBUG_
#undef _NDEBUG_


// ---------------------------------------------------------------------------------------------
// RMS Project                                                         N A N O B O A R D   v 3.0   
// ---------------------------------------------------------------------------------------------
// 
//
/*Arduino ha 3 porte :

B uscite digitali da 8 a 13
C ingressi analogici
D uscite digitali da 0 a 7
Iniziamo a parlare alla CPU a suoni di 1 e 0…..parliamo semplicemnte….1 e’ abilitato…0 disabilitato.
settare i pin da 1 a 7 come output DDRD = B11111110;
settare i pin 7,5 e 3 come HIGH e tutti gli altri LOW PORTD = B10101000; ( 1 in corrispondenza dei pin da settare…) */

uint8_t ModuleNBV30::Initialize() // ret 0 se ok
  { 
    // nothing to do 
    return (InitOK); 
  }


port_t ModuleNBV30::mapPort(flag_t io, saddr_t subaddr, flag_t ptype=0 ) 
  { 
  if(io) { pinMode(subaddr,  INPUT); // Set IN pin
           digitalWrite(subaddr,  HIGH); // pull-up on pin ( connect to GND to 
         } 
   else  { 
           pinMode(subaddr,  OUTPUT);
           digitalWrite(subaddr, LOW); // Open drain
         };      
  port_t p = _PNew(Module_::idx, subaddr, ptype);
  return (p);
  }


uint8_t ModuleNBV30::readPorts(port_t* _pi, idx_t _n)    // read pin's register, return n-chages 
  {    
   uint8_t pval;
   uint8_t c=0;   
   for(uint8_t i=0;i<_n;i++) {      // Per tutte le porte
    
    if ( _PMod(_pi[i]) == Module_::idx) {  // ...del modulo

      pval = digitalRead( _PAdr( _pi[i]) );
         #ifdef _NDEBUG_
          Serial.print("Read D8=");Serial.println(pval);
          Serial.print("lpinval=");Serial.println( lpinval ,BIN );
          Serial.print("npin=");Serial.println( _PAdr(_pi[i]) );   
          Serial.print("bit val=");Serial.println( bitRead(lpinval,_PAdr(_pi[i]) )  );   
         #endif
      if ( pval != bitRead(lpinval,_PAdr(_pi[i])) ) {        // Port changed 
        
        _pi[i] =  _PscVal( _pi[i] , pval );   // Set Val & Changed             
   
        bitWrite(lpinval, _PAdr(_pi[i]), pval); // Update port's values register
        c++;
        #ifdef _SEROUT_
        Serial.print("<D");Serial.println(_PAdr(_pi[i])); 
        #endif 
        
        #ifdef _NDEBUG_
        Serial.print(" _pi[i]=");Serial.println( _pi[i],BIN );  
        Serial.print("lpinval=");Serial.println( lpinval ,BIN );   
        Serial.print("Pin chaged=");Serial.println(_PChg(_pi[i])); 
        #endif          
      }; // hw changed 
    };  // module's port
   }; // All ports  
  
  return (c);
  };



uint8_t ModuleNBV30::writePorts(port_t* _po, idx_t _n)   // write changed outputs, return n-changes
  {
    #ifdef _NDEBUG_
    Serial.println("@ModuleSEM16::writePorts()");
    #endif
    
    uint8_t  c=0;
    uint16_t vout=0;
    uint8_t  npin=0;
    
     for(uint8_t i=0;i<_n;i++) {      // all ports
      
       if ( ( _PMod(_po[i]) == Module_::idx ) && _PChg(_po[i]) ) { //  port changed...

          digitalWrite( _PAdr(_po[i]), _PVal(_po[i])  ); 
          #ifdef _SEROUT_
          Serial.print(">D");Serial.println(_PAdr(_po[i])); 
          #endif 

          
          #ifdef _NDEBUG_
          Serial.print("digitalwrite(");Serial.print(_PAdr(_po[i]));Serial.print(",");Serial.print( _PVal(_po[i]));Serial.println(")");
          #endif  
          _po[i] =  _PsChg(_po[i],Not); // set not changed, will be update from Evaluate 
         c++;
       }
     } // ports
   
    return (c); 
  };





// ---------------------------------------------------------------------------------------------
// RMS Project                                     S P I   1 6 B I T   P O R T   E X P A N D E R  
// ---------------------------------------------------------------------------------------------
// 
//
uint8_t ModuleSEM16::Initialize() // ret 0 se ok
  {      
     #ifdef _SDEBUG_
     Serial.print("@ModuleSEM16::Initialize()....");
     Serial.print("imsk= "); Serial.println(imsk,BIN);
     #endif
    
    if ( sem16->init() ) {  // ret >0 se ok
     sem16->setPinmap(imsk);    // Set pin direction bank A,B 1=In 0=Out
     sem16->setPullup(imsk);    // Selects internal 100k input pull-up of all I/O pins at once 
     sem16->setInvert(imsk);    // Selects input state inversion of all I/O pins at once (writing a 1 turns on inversion)
     return (InitOK);  // ret 0 se ok 
    } else return (InitERR); 
  }


port_t ModuleSEM16::mapPort(flag_t io, saddr_t subaddr, flag_t ptype=0 ) 
  {
  bitWrite(imsk,subaddr,io);      // Update In/Out mask. Write a 0 to make a pin an output, a 1 to make it an input
  port_t p = _PNew(Module_::idx, subaddr, ptype);
    
  #ifdef _SDEBUG_
  Serial.println("@ModuleSEM16::setPort()....");
  Serial.print("io= "); Serial.println(io,BIN);
  Serial.print("pin= "); Serial.println(_PAdr(p)); 
  Serial.print("port= "); Serial.println(p,BIN); 
  Serial.print("imsk= "); Serial.println(imsk,BIN);  
  #endif
  
  return (p);
  }



uint8_t ModuleSEM16::readPorts(port_t* _pi, idx_t _n)    // read pin's register, return n-chages 
  {
  uint16_t rtmp = lval;        // save last read 
  lval = sem16->pinRead() & imsk; // read in ports

  uint8_t nbit;
  uint8_t c=0;   
  if (lval != rtmp ) {  // hw changed
   for(uint8_t i=0;i<_n;i++) {      // Per tutte le porte
    if ( _PMod(_pi[i]) == Module_::idx) { // ...del modulo

      nbit =_PAdr(_pi[i]);
      if (bitRead(rtmp,nbit) != bitRead(lval,nbit)) {    // Port changed 
        _pi[i] =  _PscVal( _pi[i] , bitRead(lval, nbit ) );   // Update value and mark as changed
        c++;
        #ifdef _SEROUT_ 
         Serial.print("<");Serial.print((nbit>7)?"A":"B");Serial.println((nbit>7)?nbit-7:8-nbit);
        #endif   
      };  
    }
   } // Ports
  }; // hw chg 
  return (c);
  };


uint8_t ModuleSEM16::writePorts(port_t* _po, idx_t _n)   // write changed outputs, return n-changes
  {
    uint8_t  c=0;
    uint16_t vout=0;
    uint8_t  nbit=0;
     for(uint8_t i=0;i<_n;i++) {      // all ports
      
       if ( ( _PMod(_po[i]) == Module_::idx ) && _PChg(_po[i]) ) { // port has new value
         nbit =_PAdr(_po[i]);
         bitWrite(vout, nbit, _PVal(_po[i]));
         _po[i] =  _PsChg(_po[i],Not); // set no changed for next evaluation cycle !
         c++;
         #ifdef _SEROUT_
         Serial.print(">");Serial.print((nbit>7)?"A":"B");Serial.println((nbit>7)?nbit-7:8-nbit); 
         #endif
       }
     } // ports
    if (c) { sem16->pinWrite(vout);  }
    return (c);
  };




// ---------------------------------------------------------------------------------------------
// RMS Project                                                                     L O C O N E T 
// ---------------------------------------------------------------------------------------------
// 
//
#include <LocoNet.h>
//
//
#define LCN_SW 1  // Loconet switch request
#define LCN_SR 2  // Loconet switch state
#define LCN_IR 3  // Loconet sensor report
//

uint8_t ModuleLCN::Initialize() 
  { 
    LocoNet.init(NB_LCNTX);  // Loconet on Nanoboard pins 8 (RX) and 6 (TX)
    #ifdef _LDEBUG_
    Serial.println("@ModuleLCN::Initialize()");
    #endif
    return (InitOK);
  }


port_t ModuleLCN::mapPort(flag_t io, saddr_t subaddr, flag_t ptype=0 ) 
  {
        #ifdef _LDEBUG_
        Serial.println("@ModuleLCN::mapPort()..");
        Serial.print("io ="); Serial.println(io);
        Serial.print("adr="); Serial.println(subaddr);
        Serial.print("typ="); Serial.println(ptype);
        #endif
  
        port_t p = _PFNew(Module_::idx,subaddr,ptype);

        #ifdef _LDEBUG_
        Serial.print("PFa="); Serial.println(_PFAdr(p));        
        Serial.print("prt="); Serial.println(p,BIN); 
        #endif
  
  return (p);
  }


uint8_t ModuleLCN::readPorts(port_t* _pi, idx_t _n)    // read pin's register and check if input ports are changed 
  {
    
   uint8_t  c=0;      
   lnMsg*   LnPacket = LocoNet.receive();     // pointer to a received LNet packet
        
        if ( LnPacket ) {

              uint16_t padr =0;
              uint8_t  pval =0;
              uint8_t  ptyp =0;
              uint8_t  i; 
              
              #ifdef _LDEBUG_
              uint8_t msgLen = getLnMsgSize(LnPacket);
              Serial.print("LcnMsg[x]:");  
              for (uint8_t x = 0; x < msgLen; x++) {
              uint8_t vl = LnPacket->data[x];
              if (vl < 16)    // Print a leading 0 if less than 16 to make 2 HEX digits
              Serial.print('0'); Serial.print(vl, HEX); Serial.print(" ");
              };  // print out the packet in HEX
              Serial.println(" ");
              #endif


           // Controller Mode decode: OPC_INPUT_REP, OPC_SW_REP for In ports
           // Decoder Mode decode OPC_SW_REQ, OPC_SW_REP for In ports

           switch ( LnPacket->data[0]  ) {

            case OPC_SW_REQ:  // 0xB0 - Request Switch function ( ln_opc.h: swreq_t /srq ) 
                              //
                              // OPC_SW_REQ 0xB0 ;REQ SWITCH function NO-REPLY (<0xB0>,<SW1>,<SW2>,<CHK>)
                              // <SW1> =<0,A6,A5,A4- A3,A2,A1,A0>, 7 ls adr bits. A1,A0 select 1 of 4 input pairs in a DS54
                              // <SW2> =<0,0,DIR,ON- A10,A9,A8,A7> Control bits and 4 MS adr bits.
                              //  DIR: 1 for Closed, 0 for Thrown/RED; ON: 1 for Output ON, 0 FOR output OFF
                              //
                              //  Note: Immediate response of <0xB4><30><00> if command failed, otherwise no response
                              //          
                              ptyp = LCN_SW;                              // LCN_SWREQ
                                                                          // <SW1> =<0,A6,A5,A4,A3,A2,A1,A0>
                                                                          // <SW2> =<0,0,DIR,ON,A10,A9,A8,A7>
                                                                          // DIR: 1 Closed, 0 Thrown; ON: 1 Output ON, 0 Output OFF
                              padr = LnPacket->srq.sw2 & 0x0F ;           // MSB 4 bit    
                              padr = padr<<7;                                    
                              padr |= LnPacket->srq.sw1;                  // LSB 7 bit
                              padr++;                                     // Full 11 bit received loconet address  
                              pval = ( (LnPacket->srq.sw2>>5) & 1U);      // Ignore ON bit use only DIR
                            
            break; // OPC_SW_REQ


            case OPC_SW_REP:  // 0xB1 ;Turnout SENSOR state REPORT NO-REPLY  ( ln_opc.h: swrep_t / srp )
                              //
                              // OPC_SW_REP 0xB1 ;Turnout SENSOR state REPORT NO-REPLY (<0xB1>,<SN1>,<SN2>,<CHK>) 
                              //
                              // <SN1> =<0,A6,A5,A4- A3,A2,A1,A0>, 7 ls adr bits. A1,A0 select 1 of 4 input pairs in a DS54
                              //
                              // <SN2> =<0,1,I,L- A10,A9,A8,A7> Report/status bits and 4 MS adr bits.
                              // Note: this <B1> opcode encodes input levels for turnout feedback
                              //  I =0 for "aux" inputs (normally not feedback), 1 for "switch" input used for turnout feedback for DS54 ouput/turnout # encoded by A0-A10
                              //  L =0 for this input 0V (LO), 1= this input > +6V (HI) alternately
                              //
                              // <SN2> =<0,0,C,T- A10,A9,A8,A7> Report/status bits and 4 MS adr bits.
                              // Note: this <B1> opcode encodes current OUTPUT levels
                              //  C =0 if "Closed" ouput line is OFF, 1="closed" output line is ON (sink current)
                              //  T =0 if "Thrown" output line is OFF, 1="thrown" output line is ON (sink I)       
                              
                              ptyp = LCN_SR;                              // LCN_SWREP
                                                                          // <SN1> =<0,A6,A5,A4,A3,A2,A1,A0>
                                                                          // <SN2> =<0,1,I,L,A10,A9,A8,A7>
                                                                          // I: 0 Aux, 1 Switch;  L: 0 OFF, 1 ON  
                              padr = LnPacket->srp.sn2 & 0x0F ;           // MSB 4 bit    
                              padr = padr<<7;                                    
                              padr |= LnPacket->srp.sn1;                  // LSB 7 bit
                              padr++;                                     // Full 11 bit received loconet address  
                              pval = ( (LnPacket->srp.sn2>>4) & 1U);      // Ignore "I" bit use only "L"             

            break;  // OPC_SW_REP

            
            case OPC_INPUT_REP:   // 0xB2 ; General SENSOR Input codes ( ln_opc.h: inputRepMsg / ir )
                                  //
                                  // OPC_INPUT_REP 0xB2 ; General SENSOR Input codes NO-REPLY (<0xB2>, <IN1>, <IN2>, <CHK>)
                                  // <IN1> =<0,A6,A5,A4,A3,A2,A1,A0>, 7 ls adr bits. A1,A0 select 1 of 4 inputs pairs in a DS54
                                  // <IN2> =<0,X,I,L- A10,A9,A8,A7> Report/status bits and 4 MS adr bits.
                                  //  I=0 for DS54 "aux" inputs and 1 for "switch" inputs mapped to 4K SENSOR space.
                                  //  L=0 for input SENSOR low 0V (LO), 1 for Input sensor >=+6V (HI)
                                  //  X=1, control bit , 0 is RESERVED for future!
                                  //
                                  ptyp = LCN_IR;                          // LCN_INREP 
                                                                          // <IN1> =<0,A6,A5,A4,A3,A2,A1,A0>
                                                                          // <IN2> =<0,X,I,L,A10,A9,A8,A7>
                                                                          // X: =1 control; I: 0 Aux, 1 Switch; L: 0 OFF, 1 ON  
                                  padr = LnPacket->ir.in2 & 0x0F ;        // MSB 4 bit    
                                  padr = padr<<7;                                
                                  padr |= LnPacket->ir.in1;               // LSB 7 bit
                                  padr++;                                 // Full 11 bit received loconet address  
                                  pval = ( (LnPacket->ir.in2>>4) & 1U);   // Ignore I bit use only L
                                  

            break;           
              
           }; // switch

          if (ptyp) {   //  valid message type
            
              #ifdef _LDEBUG_
              Serial.println("Lcn msg:"); 
              Serial.print("ptyp="); Serial.println(ptyp);  
              Serial.print("padr="); Serial.println(padr); 
              Serial.print("pval="); Serial.println(pval,BIN);
              #endif 
              
              // Update module's port
              i=0;
              while (i<_n &&  !( ( _PMod(_pi[i]) == Module_::idx ) && (_PFAdr(_pi[i])== padr) )  ) {  // find port
              #ifdef _LDEBUG_                
               Serial.print("PFA(i)="); Serial.print(i);
               Serial.print(","); Serial.println(_PFAdr(_pi[i]));
              #endif
                              
               i++;  
              } 

              
                if ( i<_n ) {  
                    if (_PVal(_pi[i]) != pval) {              //  check if changed  
                      _pi[i] =  _PscVal( _pi[i], pval );      // get value bit (6)
                      //_pi[i] =  _PSChg(_pi[i],On);          // set changed !!!!
                      c++;
                      #ifdef _SEROUT_
                      Serial.print("<L");Serial.println(_PFAdr(_pi[i])); 
                      #endif
                      
              #ifdef _LDEBUG_
              Serial.print("p[i]="); Serial.println( _pi[i],BIN);
              #endif
                      
                    }   
                }
          }
               
        }  // LnPacket

     return (c);  
  };



uint8_t ModuleLCN::writePorts(port_t* _po, idx_t _n)   // update changed output
  {
    lnMsg SendPacket ; // Construct a Loconet packet that requests a turnout to set/change its state
    uint8_t c = 0;

    
      #ifdef _LDEBUG_
      Serial.println("@ModuleLCN::writePorts()"); 
      #endif
      
     for(uint8_t i=0;i<_n;i++) {      // all ports
        
       if ( ( _PMod(_po[i]) == Module_::idx ) && _PChg(_po[i]) ) {   // port has new value

        if ( _MOpt1(Module_::opts) )  /// LCN_MODE_CONTROLLER
           {
              // Send OPC_SW_REQ ( 0xB0 ) - Request Switch function ( ln_opc.h: swreq_t ) 
            
              SendPacket.data[ 0 ] = OPC_SW_REQ ;                       // Set OPC
              SendPacket.data[ 1 ] = _PAdr(_po[i]);                     // Set Address ( low 7 bits )
              SendPacket.data[ 1 ]--;                                   // Full loconet address                            
              SendPacket.data[ 2 ] = (_PVal( _po[i]))?B00100000:0U;     // Set Direction t/c
              SendPacket.data[ 2 ] |= (uint8_t)_PFAdr(_po[i])>>7;       // Set  Address ( high 4 bit )
              
              #ifdef _LDEBUG_
              Serial.print("Packet:"); Serial.println((uint8_t)SendPacket.data[0],HEX); 
              Serial.print("Addres:"); Serial.println((uint8_t)SendPacket.data[1],HEX);
              Serial.print(" State:"); Serial.println((uint8_t)SendPacket.data[2],HEX);
              Serial.print("  padr:"); Serial.println((uint8_t)SendPacket.data[2],BIN);              
              #endif
              
              SendPacket.data[ 2 ] |= B00010000;   // On
              LocoNet.send( &SendPacket ); 
              
              SendPacket.data[ 2 ] &= B11101111;   // Off
              LocoNet.send( &SendPacket );
              
           } else {

              // Send OPC_INPUT_REP ( 0xB2 ) - General SENSOR Input codes ( ln_opc.h: inputRepMsg / ir )
            
              SendPacket.data[ 0 ] = OPC_INPUT_REP ;                        // Set OPC    
              SendPacket.data[ 1 ] = _PAdr(_po[i]);                        // Set Address ( low 7 bits )
              SendPacket.data[ 1 ]--;                                       // Full loconet address                            
              SendPacket.data[ 2 ] = (_PVal( _po[i]))?B00010000:0U;        // Set State On/Off
              SendPacket.data[ 2 ] |= B01100000;                            // Set report for sensor
              SendPacket.data[ 2 ] |= (uint8_t)_PFAdr(_po[i])>>7;       // Set  Address ( high 4 bit )
              
              LocoNet.send( &SendPacket ); 

              // Send OPC_SW_REP ( 0xB1 ) - Turnout SENSOR state REPORT NO-REPLY  ( ln_opc.h: swrep_t / srp )
              SendPacket.data[ 0 ] = OPC_INPUT_REP ;                        // Set OPC    
              SendPacket.data[ 1 ] = _PAdr(_po[i]);                        // Set Address ( low 7 bits )
              SendPacket.data[ 1 ]--;                                       // Full loconet address                       
              SendPacket.data[ 2 ] = (_PVal( _po[i]))?B00010000:0U;        // Set State On/Off
              SendPacket.data[ 2 ] |= B01100000;                            // Set report for "Switch"
              SendPacket.data[ 2 ] |= (uint8_t)_PFAdr(_po[i])>>7;       // Set  Address ( high 4 bit )

              LocoNet.send( &SendPacket ); 
        
           } // end CTRL

        _po[i] =  _PsChg(_po[i],Not); // set no changed !!!!
        c++;
        #ifdef _SEROUT_
        Serial.print(">");Serial.println(_PFAdr(_po[i])); 
        #endif
       } // changed
     } // all ports
     
    return (c);  
  };
