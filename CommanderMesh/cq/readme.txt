cq_control - motor control using the pololu dual motor controller (geekduino)
cq_control_scorp - motor control using the scorpionXL controller (or other PWM controller)(geekduino)
cq_remote - remote contorl box (Fio)

change 
	#define ID 1
to set the ID for the remote and motor controller. ID can be anywhere from 1 - 254


change
	#define PWM_MIN 50 //reduce for max speed
	#define PWM_MAX 150//increase for max speed
To try to get the best band out of the PWM motor controller (scorpion)
NOTE: I will be changing this from degrees (0-180) to micorsecond pulses (1000-2000)


NOTE: Do some testing with the relay to check for the auto-firing bug. Please let me know the exect behavior (does the relay ever turn off, does toggling the remote affect it, etc)




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