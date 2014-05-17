/*************************
* Crazy Quail Motor Control Box
* Scorpion contorller - PWM Signals
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
*
*
*
* Remote Interface                  Remote Port    Interpreted variable    
* Rotate Joystick                   A0             command.lookv
* Vertical Joystick (up/down)       A1             command.lookh
* Horizontal Joystick (left/down)   A2             command.walkv
*
*************************/

//change for max speed, but you may go 'out of bounds' for the scorpion
#define PWM_MIN 1250 //reduce for max speed
#define PWM_MAX 1750//increase for max speed


#define FIRE_ENABLE 12
#define FIRE_PWM 11
#define FIRE_SWITCH 5
 
#define PAN_DIRECTION_A 2
#define PAN_DIRECTION_B 4
#define PAN_PWM         9
 
#define TILT_DIRECTION_A 7
#define TILT_DIRECTION_B 8
#define TILT_PWM         10
 
#define ID 1    //ID for this unit
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
 
 #define SLOW_DELAY 100//delay before going to next jump, decrease to increase speed
 #define SLOW_JUMP 10 // jump interval increase to increase spee
 
 
 //include the I2C Wire library - needed for communication with the I2C chip attached to the LCD manual
//#include <Wire.h>
// include the RobotGeekLCD library
//#include <RobotGeekLCD.h>
 
 
 
 
#include <CommanderMesh.h>
 



 #include <Servo.h> 


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
 
 
Servo leftServo, rightServo;  // create servo object to control a servo 
 
int rotateSpeed = 90;
int servoVal = 1500;

void setup()
{

 leftServo.attach(9);  // attaches the servo on pin 9 to the servo object, left servo, tilt
 rightServo.attach(10);  // attaches the servo on pin 10 to the servo object , right servo ,pan


  SERIAL_PORT.begin(38400);
  pinMode(2,OUTPUT);
  pinMode(4,OUTPUT);
 
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
 
  
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
     digitalWrite(13,HIGH);
     delay(500);
     
     digitalWrite(13,LOW);
 
  
  
  pinMode(FIRE_SWITCH, INPUT_PULLUP);
 
  
  // initlaize the lcd object - this sets up all the variables and IIC setup for the LCD object to work
  //lcd.init();
  // Print a message to the LCD.
  //lcd.print("Crazy Quail!");
 //delay(1000);
 rightServo.writeMicroseconds(1500);//tilt
 leftServo.writeMicroseconds(1500); //rotate/pan
}
 
void loop()
{
   //check if a commander style packet was recieved
   if(command.ReadMsgs() > 0)
   {
     
    //check that the packet's id either matches the unit's id or the brodcast/sync id
    if(command.id == ID || command.id ==SYNC_ID)
    {
        //rotate joystick, pan absolute-ish
        if(command.lookV < (-1 * DEADBAND))
        {
          
          
     digitalWrite(13,LOW);
          
 
  
  
  
          if(servoVal > 1500)
          {
            servoVal = servoVal - SLOW_JUMP;
            delay(SLOW_DELAY);
          }
          
          else
          {
            servoVal = map(command.lookV, 0, -127, 1500, PWM_MIN);
          }
                            // sets the servo position according to the scaled value 
           //tempSpeed = -1*map(command.lookV, 0, -127, 90, PWM_MIN);
           
           leftServo.writeMicroseconds(servoVal);
           resetFlag = 1;
        }
       
        else if (command.lookV > DEADBAND)
        {
          
          
     digitalWrite(13,LOW);
           if(servoVal < 1500)
          {
            servoVal = servoVal + SLOW_JUMP;
            delay(SLOW_DELAY);
          }
          
          else
          {
            servoVal = map(command.lookV, 0, 127, 1500, PWM_MAX);
          }
                            // sets the servo position according to the scaled value 
           //tempSpeed = -1*map(command.lookV, 0, -127, 90, PWM_MIN);
           
           leftServo.writeMicroseconds(servoVal);
           resetFlag = 1;
           
           
          
          
         
        }
       
        else
        {
          
     digitalWrite(13,HIGH);
          if(resetFlag ==1)
          {
            
            
            
            if(servoVal <1450)
            {
               servoVal = servoVal +SLOW_JUMP;
          //Serial.println("mem");
               delay(SLOW_DELAY);
            resetFlag = 1;
            }
            else if(servoVal >1550)
            {
               servoVal = servoVal -SLOW_JUMP;
          //Serial.println("mem;");
               delay(SLOW_DELAY);
            resetFlag = 1;
            }
            else
            {
               servoVal = 1500;
            resetFlag = 0;
            }
            
            
    
            
           leftServo.writeMicroseconds(servoVal);
          }
        }
       
        
        
        //pan incremental
        if(command.lookH < (-1*DEADBAND) || command.lookH > DEADBAND)
        {
          //lcd.print("RARAWW");
          tempSpeed = tempSpeed + command.lookH/20;
          tempSpeed = min(tempSpeed, 255);
          tempSpeed = max(tempSpeed, -255);


         
          leftServo.write(map(tempSpeed, -255, 255, PWM_MIN, PWM_MAX));                  // sets the servo position according to the scaled value 




        }

        
        
        
        
        //tilt
        if(command.walkV < (-1*DEADBAND))
        {

          rightServo.write(map(command.walkV, 0, -127, 90, PWM_MIN));                  // sets the servo position according to the scaled value 

        }
       
        else if (command.walkV > DEADBAND)
        {
          rightServo.write(map(command.walkV, 0, 127, 90, PWM_MAX));                  // sets the servo position according to the scaled value 

         
        }
       
        else
        {
                  rightServo.write(90);                  // sets the servo position according to the scaled value 

         
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
        leftServo.writeMicroseconds(1500);                  // sets the servo position according to the scaled value 


        //motor2 off
             rightServo.writeMicroseconds(1500);                  // sets the servo position according to the scaled value 

        //fire motor/relay off
        digitalWrite(12, LOW);
        fireFlag = 0;
        fireFlag2 = 0;
    digitalWrite(13,LOW);
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
