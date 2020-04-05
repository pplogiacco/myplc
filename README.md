# myplc

MyPlc project is part of Railways Modelling System ( https://www.rail2rail.eu ) 

The sketch implements a Very Simple Programmable Controller developed on Arduino Nano and some custom hardware modules.
The project also link external Loconet library (https://github.com/mrrwa/LocoNet).

Goal of the project is to standardize a methodology to wire the railway's models ensuring riability and compatibility with most commercial hardware based on Loconet protocol.

Cheap hardware and free software offers the opportunity to experiment innovative approch to railway's reproduction design and develop.

Hardware system consist basically of three type of modules:

Nanoboard
Is a shield-board for Arduino Nano that implement power circuit and define wiring methodology for all implementations RMS (Railways Modelling System). 
The board offers 8 basic IO ports (screw wiring terminal), a couple of connectors defined expansion bus, and some other connector for LCD display and basic controls as reset, keylock switch.

Expansion Module (SEM16)
This module is designed to expand the system by 16 basic IO ports (always wireable by screw terminal). Each Nanoboard support  8 expansinon module over 128 physical IO ports that can be managed by logical controller implemented by MyPlc's firmware.

Loconet-IF
Is the interface to Loconet's bus used by Nanoboard. It's very easy and cheaper becouse it only does adaption of electrical signals. 

All hardware's design is free and I've published Eagle's files too. It can be make by yourself or if you prefer can buy it online.

Over this I've released a visual tools too called Vipex (Visual Programming Enviroment ) executable on Windows, iOS and Linux to simplify all programming task as upload firmware and functions to set the essential logic rules that performs ports states evaluations and sets.


Enjoy

plogiacco@smartlab.it
