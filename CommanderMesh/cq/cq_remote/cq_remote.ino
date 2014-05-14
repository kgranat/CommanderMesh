/*************************
* Crazy Quail Remote Transmitter Control Box
*
*
*
* A0 - Rotate Joystick
* A1 - Vertical Joystick
* A2 - Horizontal Joystick
*
 * DIO 2 - Pushbutton (Input Pullup High)
*
*
* Packet Info'
* COMMANDER PACKET FIX DOCUMENTATION
*
*************************/


#define ARDUINO_FIO

#ifdef ARDUINO
#define SERIAL_PORT Serial
#endif

#ifdef ARDUINO_FIO
#define SERIAL_PORT Serial1
#endif


#define ID 1

 
#define FIRE_BUTTON_PIN 2
#define NUDGE_BUTTON_PIN 3
 
// analog  in full 0-1023 range
int RotateVal = 0;
int LeftRightVal = 0;
int UpDownVal = 0;
// mapped analog  in  0-255 range
unsigned char  RotateValMapped = 0;
unsigned char  LeftRightValMapped = 0;
unsigned char  UpDownValMapped = 0;
//   avoid using pins with LEDs attached
 
void setup()
{
  SERIAL_PORT.begin(9600);
  
  pinMode(FIRE_BUTTON_PIN, INPUT_PULLUP); //set pin # FIRE_BUTTON_PIN to be a input with internal pullups enabled
  pinMode(NUDGE_BUTTON_PIN, INPUT_PULLUP); //set pin # FIRE_BUTTON_PIN to be a input with internal pullups enabled
}
 
void loop()
{
  
   RotateValMapped = map(analogRead(0), 0, 1023, 1, 254);
   LeftRightValMapped = map(analogRead(1), 1023, 0, 1, 254);
   UpDownValMapped = map(analogRead(2), 0, 1023, 1, 254);
 
  
  
  
  unsigned char buttons = 0;
  //check if FIRE_BUTTON_PIN is LOW(active because of internal pullup), then set bit appropriatley.
  if(digitalRead(FIRE_BUTTON_PIN) == LOW)
  {
    buttons = buttons + 1; //set the low bit on the button byte high 
  }
  if(digitalRead(NUDGE_BUTTON_PIN) == LOW)
  {
    buttons = buttons + 2; //set the low bit on the button byte high 
  }
 
 
  
  SERIAL_PORT.write((byte)0xff);  //header
  SERIAL_PORT.write((byte)RotateValMapped);  //analog 0
  SERIAL_PORT.write((byte)LeftRightValMapped);  //analog 1
  SERIAL_PORT.write((byte)UpDownValMapped);  //analog 2
  SERIAL_PORT.write((byte)0x00);
  SERIAL_PORT.write((byte)buttons);//button byte
  SERIAL_PORT.write((byte)0x00);
  SERIAL_PORT.write((byte)ID);
  SERIAL_PORT.write((unsigned char)(255-(RotateValMapped + LeftRightValMapped + UpDownValMapped + buttons + ID))%256); //checksum - add all values (except header) then take off the lower byte, and invert it
   
  delay(33); //delay 33 ms, run at ~ 30hz 
 }
