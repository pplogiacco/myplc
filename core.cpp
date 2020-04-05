
#undef _CDEBUG_ 

#include <Arduino.h>
#include <avr/io.h>

#include "core.h"
#include "modules.h"
#include <EEPROM.h>


idx_t MyPlc::addWiringModule(module_t mtype, maddr_t address=0)      // Map hardware module
  {
   #ifdef _SEROUT_    
   Serial.print(nMO);
   #endif       

   switch (mtype) {
      case _NBV30:
           _MO[nMO].mod = new ModuleNBV30(nMO);      // Differetn types different implementations 
              #ifdef _SEROUT_
              Serial.println(" Nanoboard");
              #endif   
          break;   
          
      case _SEM16: 
           _MO[nMO].mod = new ModuleSEM16(nMO, address);    // Different type different implementations          
           _MO[nMO].mtype = _SEM16;
           _MO[nMO].addr = address;
           
              #ifdef _SEROUT_
              Serial.print(" Sem16(");Serial.print(address);Serial.println(") ");
              #endif
          break;
          
      case _LCNIF:
           _MO[nMO].mod = new ModuleLCN(nMO, address);      // Differetn types different implementations   
           _MO[nMO].mtype = _LCNIF;
           _MO[nMO].addr = address;                  
              #ifdef _SEROUT_
              Serial.println(" Loconet ");
              #endif       
          break;           
   };  
   _MO[nMO].addr;
   _MO[nMO].mtype = mtype;
   nMO++;
   return (nMO-1);
  }; // addWiringModule...



idx_t MyPlc::addPort(flag_t io, idx_t imod, saddr_t subaddr, flag_t ptype=0, flag_t dval=0 )
  { 
   idx_t iprt;
   if (io) {   
     _PI[nPI] = _MO[imod].mod->mapPort(In , subaddr, ptype);  // In (1)
     #ifdef _SEROUT_
     ;Serial.print(imod);Serial.print("<");Serial.println(subaddr);
     #endif       
     iprt = nPI;
     nPI++;
   } else {  
     _PO[nPO] = _MO[imod].mod->mapPort(Out, subaddr, ptype); // Out (0)
     #ifdef _SEROUT_
     Serial.print(imod);Serial.print(">");Serial.println(subaddr);
     #endif  
     iprt = nPO;
     nPO++;
   }
   return (iprt);
  };


uint8_t MyPlc::addRule(flag_t cc, idx_t pin, flag_t vin, idx_t pout, flag_t vout) 
  {

  };
  

uint8_t MyPlc::addRule(flag_t cc, idx_t pin, idx_t pout )
  {
   _RL[nRL].rl = 0U;    // initialize
   _RL[nRL].rl = _RuleSCC(_RL[nRL].rl,cc); // set condition bit
   _RL[nRL].pin = pin;  // set input port
   _RL[nRL].pout = pout; // set out port
   nRL++;     
  };



uint8_t MyPlc::Init()   // Initialize...
  {
     uint8_t err=0;                                                 // USE WHILE and stop on Error E01,E02,E03 <<<<----------!!!!!!!!!                                                       
      for(idx_t i=0; i<nMO; i++ ) {  // initialize all modules        
        err += _MO[i].mod->Initialize();         
      };
    #ifdef _SEROUT_
    Serial.println((err)?"Error !":"Ready!"); 
    #endif    
   return (err); 
  }



uint8_t MyPlc::Read()     // Update input ports 
  {
      uint8_t nchg=0;
       if (digitalRead(NB_SW1)) {     // Use Plc State     _STOP, _RUN
         for(idx_t i=0; i<nMO; i++ ) {  // for all modules       
          nchg += _MO[i].mod->readPorts(_PI,nPI); // ret n ports changed
         }; 
         if (nchg) { MyPlc::rtime = 1; }        // In ports changed
       };
      return nchg;       
  };



uint8_t MyPlc::Evaluate()             // Evaluate logic rules and update out-ports
  { 
   #ifdef _CDEBUG_
   Serial.print("@Plc::Evaluate(), state = ");Serial.println(state);  
   #endif
                                                                                            // !!!!!!!!! Implement PLC_WORKMODE to use interactive cycle
   uint8_t n=0;
   if ( MyPlc::rtime == 1 ) {
    
    for (idx_t i = 0; i<nRL; i++) {   // for all rules 
        
        #ifdef _CDEBUG_
        Serial.print("__nRL=");Serial.println(i);     
        Serial.print("Rule =");Serial.println(_RL[i].rl,BIN); 
        Serial.print("In prt=");Serial.println(_RL[i].pin,BIN);   
        //Serial.print("In val=");Serial.println(_PVal( _PI[ _RL[i].pin ]  ) );
        //Serial.print("In chg=");Serial.println(_PChg( _PI[ _RL[i].pin] ));      
        #endif
        
      if( _PChg( _PI[ _RL[i].pin ] ) ) {   // port changed              

        // Serial.print("In chg=");Serial.println(_PChg( _PI[ _RL[i].pin] ));    
                 
        switch( (_RL[i].rl & _CCMask) ) 
        {
           case _Cxx:       // Imposta uscita allo stesso valore di ingresso 
                  
                    //_PO[_RL[i].pout] = _PSChg(_PO[_RL[i].pout],On);                           // Set Out port changed 
                    _PO[_RL[i].pout] = _PscVal(  _PO[_RL[i].pout] , _PVal(_PI[_RL[i].pin] ) );  // set pOUT = pIN
                    #ifdef _CDEBUG_

                    Serial.print("In val=");Serial.println(_PVal( _PI[ _RL[i].pin ]  ) );
                    Serial.print("Out vl=");Serial.println(_PVal(  _PO[_RL[i].pout] )  );  
                    Serial.print("Out ch=");Serial.println(_PChg(  _PO[_RL[i].pout] )  );         
                    #endif                               
                    break; 

           case _Cnx:       // Imposta uscita al valore negato dell'ingresso 
                    //_PO[_RL[i].pout] = _PSChg(_PO[_RL[i].pout],On);                           // Set Out port changed 
                    _PO[_RL[i].pout] = _PscVal(_PO[_RL[i].pout], ! _PVal(_PI[_RL[i].pin] ) );  // set pOUT = ! pIN
                    break; 
                    
           case _Ceq:       // Imposta uscita a valore ss ingresso corrisponde a vv
           
                    break;
           case _Cne: {  }  // Imposta uscita a valore se ingresso non corrisponde a ss 
           
                    break;          
           default:   {  }
        };      
      n++;   
        
      #ifdef _CDEBUG_
      Serial.print("Out pt=");Serial.println(_RL[i].pout,BIN); 
      #endif    
      }; // prt chged
    }; 

    for (int i = 0; i<nRL; i++) { // for all rules
      _PI[_RL[i].pin] = _PsChg(_PI[_RL[i].pin],Not);     
    }

   }; // state 
   rtime = 2;   // Eval complete
   return n;   
  };  // check


uint8_t MyPlc::Write()     // Update out-ports 
  {         
     uint8_t rv=0;
     for(idx_t i=0; i<nMO; i++ ) {  // for all modules       
      rv += _MO[i].mod->writePorts(_PO,nPO);
     }; 
     return rv; 
  };



// ---------------------------------------------------------------------------------------------
//                                                                                   C O N F I G
// ---------------------------------------------------------------------------------------------


void MyPlc::clearMemory() 
{
     delete[] _MO;
     delete[] _PI;
     delete[] _PO;
     delete[] _RL;   
}

  
void MyPlc::initMemory(idx_t maxmod,idx_t maxin,idx_t maxout,idx_t maxrls)
{ 
       _MO = new mnode_t[maxmod];
       _PI = new port_t[maxin];
       _PO = new port_t[maxout];
       _RL = new rule_t[maxrls];
       nMO=0;
       nPI=0;
       nPO=0;
       nRL=0; 
} 



bool MyPlc::checkEprom(){  // Update PLC indexes to max
  // Check first 2 bytes marker  
  uint16_t ep = EEPROM_OFFSET;
  uint8_t chkA = EEPROM.read(ep++);  //  x 3 Bytes
  uint8_t chkB = EEPROM.read(ep++);  //  x 2 Bytes 
  if ( ((chkA>>4)&&chkB) && ((chkB<<4)&&chkA  )) {  // Assume valid config in eeprom
   return (true); // force update 
  };
 return (false);
};



bool MyPlc::saveToEprom(){  

 uint16_t ep = EEPROM_OFFSET;
 uint8_t i;
 uint16_t tmp;

   // EEPROM.begin(EEPROM_SPACE);
   // EEPROM.length()
 if (true ) {       // verify memory size
   
   EEPROM.write(ep++, 0B01011010);  // ChkA
   EEPROM.write(ep++, 0B10100101);  // ChkB

   EEPROM.write(ep++, nMO);  // x 3 Bytes
   EEPROM.write(ep++, nPI);  // x 2 Bytes 
   EEPROM.write(ep++, nPO);  // x 2 Bytes
   EEPROM.write(ep++, nRL);  // x 3 Bytes
   
   for ( i = 0; i < nMO; i++ ) {         // Save modules defs
      EEPROM.write(ep++, _MO[i].mtype );
      tmp =  _MO[i].addr;    //_MO[i].mod->getAddr();
      EEPROM.write(ep++, lowByte(tmp) );
      EEPROM.write(ep++, highByte(tmp) );  
      //EEPROM.write(ep++, mem.xMO[i].mPI );
      //EEPROM.write(ep++, mem.xMO[i].mPO );
   }
   for ( i = 0; i < nPI; i++ ) {         // Save In Port defs
      EEPROM.write(ep++, lowByte(_PI[i]) );  
      EEPROM.write(ep++, highByte(_PI[i]) );
   }

   for ( i = 0; i < nPO; i++ ) {         // Save Out Port defs
      EEPROM.write(ep++, lowByte(_PO[i]) );  
      EEPROM.write(ep++, highByte(_PO[i]) );
   }

   for ( i = 0; i < nRL; i++ ) {         // Save Rules defs
      EEPROM.write(ep++, _RL[i].rl );  
      EEPROM.write(ep++, _RL[i].pin );
      EEPROM.write(ep++, _RL[i].pout );
   }
                    
   EEPROM.end();   
   Beep();
   Beep();
  return (true);
 };
 return (false);
};


void MyPlc::loadEprom(){  // Update PLC indexes to 0

   uint16_t ep = EEPROM_OFFSET+2;
   idx_t i;
   uint16_t tmp;
   uint8_t tmpb;
  
   uint8_t _nMO = EEPROM.read(ep++);  //  x 3 Bytes
   uint8_t _nPI = EEPROM.read(ep++);  //  x 2 Bytes 
   uint8_t _nPO = EEPROM.read(ep++);  //  x 2 Bytes
   uint8_t _nRL = EEPROM.read(ep++);  //  x 3 Bytes

   _MO = new mnode_t[_nMO];
   _PI = new port_t[_nPI];
   _PO = new port_t[_nPO];
   _RL = new rule_t[_nRL];
   
       
   for ( i = 0; i < _nMO; i++ ) {         // Map hardware module
      tmpb = EEPROM.read(ep++);   // mtype
       tmp = EEPROM.read(ep++);       // low address byte
       tmp &= EEPROM.read(ep++) << 8; // high
       
        //_MO[i].mPI = EEPROM.read(ep++);   // Save first nodule's port idx TO OPTIMIZE !!!
        //_MO[i].mPO = EEPROM.read(ep++);
      addWiringModule((module_t)tmpb, tmp);        
   }
   
   for ( i = 0; i < nPI; i++ ) {         // Map ports In
      tmp = EEPROM.read(ep++);  
      tmp &= EEPROM.read(ep++ ) << 8;   
      addPort(In, _PMod(tmp) , _PFAdr(tmp) , _PTyp(tmp) );
   }

   for ( i = 0; i < nPO; i++ ) {         // Map ports Out
      tmp = EEPROM.read(ep++);                  
      tmp &= EEPROM.read(ep++ ) << 8;       
      addPort(Out, _PMod(tmp) , _PFAdr(tmp) , _PTyp(tmp) );
   }

   for ( i = 0; i < nRL; i++ ) {         // Map rules
      tmp = EEPROM.read(ep++);  
      tmpb = EEPROM.read(ep++);
      addRule(tmp,tmpb,EEPROM.read(ep++) );
   }
  
};


void MyPlc::loadDefault()  {

       _MO = new mnode_t[1];
       _PI = new port_t[2];
       _PO = new port_t[2];
       _RL = new rule_t[2];
 
      idx_t m0 =  addWiringModule(_NBV30);      // Nanoboard v 3.0
      //
      idx_t p1  = addPort(In  ,m0, _P1);       // Nanoboard pin D8
      idx_t p2  = addPort(In  ,m0, _P2);       // ...
      idx_t p3  = addPort(Out ,m0, _P3); 
      idx_t p4  = addPort(Out ,m0, _P4); 
      addRule(p1,p3);            
      addRule(p2,p4);            
}


void MyPlc::updateCycle(long period) {
  rtime = (uint8_t) period;
};



// ---------------------------------------------------------------------------------------------
//                                                                                  S E R I A L
// ---------------------------------------------------------------------------------------------



void MyPlc::sendINFO() 
{
  char   pktbuf[_VS_PKT_SIZE];       // char token buffer
  vs_pkt_t *pbuf = (vs_pkt_t*) pktbuf;
     pbuf->cmd = _INFO;         
     pbuf->info.hardware = _MYPLC_HW;        // Board type
     pbuf->info.firmware = _MYPLC_FW;        // Firmware version
     
     pbuf->info.serial = 0;
     
     pbuf->info.memory = 1024;                // Micro available memoryDevice 
     pbuf->info.eprom = 0;
     
     pbuf->info.cycle  = rtime;               // Cycle time ms
   
     pbuf->info.modules= nMO;                 // Modules
     pbuf->info.p_in   = nPI;                 // n Ports
     pbuf->info.p_out  = nPO;                 // n Ports
     pbuf->info.rules  = nRL;                 // Rules  
     Serial.write(pktbuf, _VS_PKT_SIZE);
     Serial.flush();
}


void MyPlc::sendWDatas() 
{
  idx_t i;
  
  char   pktbuf[_VS_PKT_SIZE];       // char token buffer
  vs_pkt_t *pbuf = (vs_pkt_t*) pktbuf;
   
   pbuf->cmd = _MODULE; 
   for (i = 0; i < nMO; i++ ) { // Send all modules datas... 
     pbuf->module.idx =  i;
     pbuf->module.mtype = _MO[i].mtype;
     pbuf->module.addr =  _MO[i].addr;
     
     Serial.write(pktbuf, _VS_PKT_SIZE);
     Serial.flush();
   }                 

   pbuf->cmd = _PORT; 
   for ( i = 0; i < nPI; i++ ) { // Send all In Ports datas... 
     pbuf->port.idx =  i;
     pbuf->port.imodule = _PMod(_PI[i]);
     pbuf->port.io = In;
     pbuf->port.subaddr = _PFAdr(_PI[i]);
     pbuf->port.ptype = _PTyp(_PI[i]);
     pbuf->port.defval = Off;     
     Serial.write(pktbuf, _VS_PKT_SIZE);
     Serial.flush();
   } 

   pbuf->cmd = _PORT; 
   for ( i = 0; i < nPO; i++ ) { // Send all In Ports datas... 
     pbuf->port.idx =  i;
     pbuf->port.imodule = _PMod(_PO[i]);
     pbuf->port.io = Out;
     pbuf->port.subaddr = _PFAdr(_PO[i]);
     pbuf->port.ptype = _PTyp(_PO[i]);
     pbuf->port.defval = Off;     
     Serial.write(pktbuf, _VS_PKT_SIZE);
     Serial.flush();
   }

   pbuf->cmd = _RULE; 
   for ( i = 0; i < nRL; i++ ) { // Send all In Ports datas... 
     pbuf->rule.idx  =  i;   
     pbuf->rule.rl   = _RL[i].rl;
     pbuf->rule.pin  = _RL[i].pin;
     pbuf->rule.pout = _RL[i].pout;
     Serial.write(pktbuf, _VS_PKT_SIZE);
     Serial.flush();
   }
}




void MyPlc::checkSerial()
{
  char   pktbuf[_VS_PKT_SIZE];       // char token buffer
  vs_pkt_t *pbuf = (vs_pkt_t*) pktbuf;   
  
   if (Serial.readBytes(pktbuf, _VS_PKT_SIZE)==_VS_PKT_SIZE) {   // Read packet

          switch ( pbuf->cmd  ) {
                                              
             case _REWRITE:           // Clear uC memory
                  if ( pbuf->rewrite.mode == _VS_REWRITE_BEGIN ) 
                  {
                    clearMemory();
                    initMemory(pbuf->rewrite.nmo,pbuf->rewrite.npi,pbuf->rewrite.npo,pbuf->rewrite.nrl);
                    Beep();Beep();Beep();
                    
                  } else {    
                    saveToEprom();
                    sendINFO();                    
                  }
                break;    
        
             case _RETRIEVE:        // uC -> PC
                  sendWDatas();   
                break;               
                
             case _INFO:            // Get uC details
                  sendINFO(); 
                break;              
                  
             case _MODULE:          // Trasfer module
                  addWiringModule((module_t)pbuf->module.mtype, pbuf->module.addr);   
                break;                     
                
             case _PORT:            // Trasfer port     
                  addPort(pbuf->port.io, pbuf->port.imodule, pbuf->port.subaddr , pbuf->port.ptype, pbuf->port.defval );
                break;   
                
             case _RULE:           // Trasfer rule     
                  addRule(pbuf->rule.rl,pbuf->rule.pin,pbuf->rule.pout );                
                break;           
           }; // switch 
   
   } else { // Pkt error
         
   };
}






  
