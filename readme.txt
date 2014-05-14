Mesh Commander Library

Updated version of the Commander Library that allows you to broadcast commands to multiple robots individually from one controller

 Packet Info
  Packet #         Data
  0                Header (Contanst 0xFF)
  1                Analog 0 Value mapped from 0-1023 -> -127 to 127
  2                Analog 1 Value mapped from 0-1023 -> -127 to 127
  3                Analog 2 Value mapped from 0-1023 -> -127 to 127
  4                Analog 3 Value mapped from 0-1023 -> -127 to 127 UNUSED
  5                Button Bute
  6                Extende Byte
  7                ID
  8                Checksum (255 - (parameters %255));
  
  
  
 Installation - place 'CommanderMesh' into your library folder
 
 cq - examples files