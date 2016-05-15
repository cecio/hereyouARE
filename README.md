# hereyouARE
Agile Rescue Engine: open source dog tracker based on Arduino compatible board

It needs the Xadow libraries to compile:
  https://github.com/Seeed-Studio/Xadow_MainBoard
 
 Functions:
 
 send SMS with the following format to trigger functions
 
  TRACK#ON#<sec>     Sends SMS with GPS position every <sec> to Primary Number
  
  TRACK#OFF          Stops track functions
  
  UBIDOTS#ON         Sets the Ubidots tracking ON or OFF
  
  GETPOS#            Get the position via SMS instead of via call (useful when out of GSM coverage)
  
  GETPOSGSM#         Get the position vis GSM cells (useful when out of GPS coverage)
  
  INFO#              Sends debug infos to calling number
 
 
Programming:
===========
 
 
 send SMS with the following format to program the device
 
 PN#<num>         Sets the accepted number for receiving call and programming
  Once set, other numbers will be ignored
 A1#<num>         Sets additional numbers for programming/call acceptance
 
 A2#<num>         Sets additional numbers for programming/call acceptance
 
 A3#<num>         Sets additional numbers for programming/call acceptance
 
 A4#<num>         Sets additional numbers for programming/call acceptance
 
 SE#<string>      Sets the search engine for coordinates sents back with SMS
 
 MU#METRIC        Sets the measure unit: METRIC or IMPERIAL
 
 SA#x1;y1#x2;y2   Defines coordinates for Safe Area. Decimal format. Example:
  45.8834;9.8172#45.7456;10.2401
 
 SF#OFF              Sets the Safe Area check: ON or OFF
 
 SM#OFF              Sets the sleep Mode: ON or OFF
 
 UT#<string>         Ubidots Auth Token (short token)
 
 UI#<string>          Ubidots Variable ID
 
 UA#<string>         APN for Ubidots connection


RESET
=====

Press WAKE button for 15 seconds to reset the Primary Number
