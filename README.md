# myplc

MyPlc project is part of Railways Modelling System ( https://www.rail2rail.eu ) 

The sketch implements a Very Simple Programmable Controller developed on Arduino Nano and some custom hardware modules.
The project also link external Loconet library (https://github.com/mrrwa/LocoNet).

Goal of the project is to define a methodology to wire the railway's models ensuring riability and compatibility with most commercial hardware based on Loconet protocol.

Cheap hardware and free software offers the opportunity to experiment innovative approch to railway's reproduction design and develop.

Hardware system consist basically of three type of modules:

# Nanoboard
It's a shield-board for Arduino Nano that implement power circuit and define wiring model for all RMS (Railways Modelling System) implementations. 
The board offers 8 basic IO ports (screw wiring terminal), a couple of connectors defined expansion bus, and some other connector for LCD display and basic controls as reset and keylock switch.

# Expansion Module (SEM16)
This module is designed to expand the system by 16 basic IO ports (always wireable by screw terminal). Each Nanoboard support  8 expansinon module over 128 physical IO ports that can be managed by logical controller implemented in MyPlc firmware.

# Loconet-IF
It's the interface to Loconet's bus used by Nanoboard. It's very easy and cheaper becouse it only does adaption of electrical signals. 

All hardware's designs are free and I've published Eagle files too. You can make devices by yourself or buy it online.

Over this I've released a visual tools too called  Vipex (Visual Programming Enviroment ) executable on Windows and Linux to simplify all programming task as upload firmware and set the logic rules.


Enjoy yourself

plogiacco@smartlab.it
