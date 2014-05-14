/*************************
* Crazy Quail Motor Control Box
*
*
*
*
 *
*
* Packet Info
*  Packet #         Data
*  0                Header (Contanst 0xFF)
*  1                Analog 0 Value mapped from 0-1023 -> -127 to 127
*  2                Analog 1 Value mapped from 0-1023 -> -127 to 127
*  3                Analog 2 Value mapped from 0-1023 -> -127 to 127
*  4                Analog 3 Value mapped from 0-1023 -> -127 to 127 UNUSED
*  5                Button Bute
*  6                Extende Byte
*  7                ID
*  8                Checksum (255 - (parameters %255));
*  Recieve code taken from ArbotiX Commander Library
*************************/
#define FIRE_ENABLE 12
#define FIRE_PWM 11
#define FIRE_SWITCH 5
 
#define PAN_DIRECTION_A 2
#define PAN_DIRECTION_B 4
#define PAN_PWM         9
 
#define TILT_DIRECTION_A 7
#define TILT_DIRECTION_B 8
#define TILT_PWM         10
 
#define ID 1
#define SYNC_ID 254//shared ID for synchronus commands


#define ARDUINO

#ifdef ARDUINO
#define SERIAL_PORT Serial
#endif

#ifdef ARDUINO_FIO
#define SERIAL_PORT Serial1
#endif


#define DEADBAND 20 
 
 #define TIMEOUT  3000 
 //include the I2C Wire library - needed for communication with the I2C chip attached to the LCD manual
//#include <Wire.h>
// include the RobotGeekLCD library
//#include <RobotGeekLCD.h>
 
 
#include <CommanderMesh.h>
 
// create a robotgeekLCD object named 'lcd'
//RobotGeekLCD lcd;
 
Commander command = Commander();
// analog  in  0-255 range
int RotateVal = 0;
int LeftRightVal = 0;
int UpDownVal = 0;
int buttons;
int index = -1; //packet index, byte of the packet currently being serached for -1 = new packet
unsigned char vals[4];  // temporary values, moved after we confirm checksum
int checksum = 0;
int resetFlag = 0; //flag to tell if we were in absolute speed mode
int tempSpeed = 0; //temp pwm speed out for incremental mode
int fireFlag = 0;
int fireFlag2 = 0;
unsigned long g_ulLastMsgTime;
 
// Variables will change:
int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
int buttonState2;             // the current reading from the input pin
int lastButtonState2 = LOW;   // the previous reading from the input pin
 
// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers
long lastDebounceTime2 = 0;  // the last time the output pin was toggled
long debounceDelay2 = 50;    // the debounce time; increase if the output flickers
 
long lastStopTime = 0;
 
void setup()
{
  SERIAL_PORT.begin(9600);
  pinMode(2,OUTPUT);
  pinMode(4,OUTPUT);
 
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
 
  
  pinMode(12,OUTPUT);
 
  
  
  pinMode(FIRE_SWITCH, INPUT_PULLUP);
 
  
  // initlaize the lcd object - this sets up all the variables and IIC setup for the LCD object to work
  //lcd.init();
  // Print a message to the LCD.
  //lcd.print("Crazy Quail!");
 //delay(1000);
}
 
void loop()
{
 
   if(command.ReadMsgs() > 0)
   {
    if(command.id == ID || command.id ==SYNC_ID)
    {
        if(command.lookV < (-1 * DEADBAND))
        {
          digitalWrite(2,LOW);
          digitalWrite(4,HIGH);
          analogWrite(9,map(command.lookV, 0, -127, 0, 255));
          tempSpeed = -1*map(command.lookV, 0, -127, 0, 255);
          resetFlag = 1;
        }
       
        else if (command.lookV > DEADBAND)
        {
          digitalWrite(4,LOW);
          digitalWrite(2,HIGH);
          analogWrite(9,map(command.lookV, 0, 127, 0, 255));
          tempSpeed = 1*map(command.lookV, 0, 127, 0, 255);
          resetFlag = 1;
         
        }
       
        else
        {
          if(resetFlag ==1)
          {
            digitalWrite(4,LOW);
            digitalWrite(2,LOW);
            analogWrite(9,0);
            resetFlag = 0;
          }
        }
       
        
        
        //pan incremental
        if(command.lookH < (-1*DEADBAND) || command.lookH > DEADBAND)
        {
          //lcd.print("RARAWW");
          tempSpeed = tempSpeed + command.lookH/20;
          tempSpeed = min(tempSpeed, 255);
          tempSpeed = max(tempSpeed, -255);
          if (tempSpeed > 0)
          {
           
            digitalWrite(4,LOW);
            digitalWrite(2,HIGH);
          }
          else
          {
           
            
            digitalWrite(2,LOW);
            digitalWrite(4,HIGH);
          }
         
          analogWrite(9,abs(tempSpeed));
         
        }

        
        
        
        
        //tilt
        if(command.walkV < (-1*DEADBAND))
        {
          digitalWrite(7,LOW);
          digitalWrite(8,HIGH);
          analogWrite(10,map(command.walkV, 0, -127, 0, 255));
        }
       
        else if (command.walkV > DEADBAND)
        {
          digitalWrite(8,LOW);
          digitalWrite(7,HIGH);
          analogWrite(10,map(command.walkV, 0, 127, 0, 255));
         
        }
       
        else
        {
          digitalWrite(7,LOW);
          digitalWrite(8,LOW);
          analogWrite(10,0);
         
        }
       
        
   
   
        int reading = command.buttons&BUT_R1;
   
    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH),  and you've waited
    // long enough since the last press to ignore any noise:
   
    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) 
    {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
   
    if ((millis() - lastDebounceTime) > debounceDelay) 
    {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:
   
      // if the button state has changed:
      if (reading != buttonState) 
      {
        buttonState = reading;
   
        // only toggle the LED if the new button state is HIGH
        if (buttonState == HIGH) 
        {
         
          
          digitalWrite(12, HIGH);
          fireFlag = 1;
           

    
    
    
          //ledState = !ledState;
          //delay(500);
          //digitalWrite(12, LOW);
         
        }
      }
    }
   
   
    int reading2 = command.buttons&BUT_R2;
    if(fireFlag ==0)
    {
          if (reading2 == LOW) {
           
            
            digitalWrite(12, LOW);
            fireFlag2 = 0;
     
     
          }
          else
          {
          
            digitalWrite(12, HIGH);
            fireFlag2 = 1;
          }
    }
    // save the reading.  Next time through the loop,
    // it'll be the lastButtonState:
    lastButtonState = reading;
    lastButtonState2 = reading2;
   
   
    
    
    }
    g_ulLastMsgTime = millis();
    
  }//end commander check
  
  else
  {
      if ((millis() - g_ulLastMsgTime) > TIMEOUT)
      {
       
        //motor 1 off
        digitalWrite(7,LOW);
        digitalWrite(8,LOW);
        analogWrite(10,0);

        //motor2 off
        digitalWrite(2,LOW);
        digitalWrite(4,LOW);
        analogWrite(9,0);
       
        //fire motor/relay off
        digitalWrite(12, LOW);
        fireFlag = 0;
        fireFlag2 = 0;
      }
   }
 
 
  if(digitalRead(FIRE_SWITCH) == HIGH)
  {
    if(fireFlag2 == 0)
    {
      lastStopTime = millis();
      digitalWrite(12, LOW);
      fireFlag = 0;
    }
   
    
  }//end commander else
 
  
}//end loop
