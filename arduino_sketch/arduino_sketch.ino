#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#define DEBUG
#ifdef DEBUG
#include "DebugUtils.h"
#endif

#include <ax12.h>
#include <BioloidController.h>
#include <Servo.h>
#include <EncodersAB.h>
#include <SharpIR.h>
#include "CommunicationUtils.h"
#include "FreeSixIMU.h"
#include <Wire.h>

BioloidController bioloid = BioloidController(57142);  // SETUP SERIAL CONNECTION TO RX64 SERVOS


///////////////////////////////6DOF Info//////////////////////////////////////////////
float q[4]; //hold q values
FreeSixIMU my3IMU = FreeSixIMU();

////////////////////////MOTOR VARIABLES//////////////////////////////////////////
Servo leftdrive;                // Used to control HB25s motor controllers which take a servo pulse and convert it to PWM
Servo rightdrive;
int leftmotor = 0;              // motor power
int rightmotor = 0;             // motor power
int leftmotor_mapped = 1500;     // mapped to servo pulse (1000 to 2000 us)
int rightmotor_mapped = 1500;
int stallreset = 0;
int stall_left = 0;
int stall_right = 0;
bool use_stall_checks = 0;  //1 to use, 0 to turn off

///////////////////////RX64 SERVO VARIABLES//////////////////////////////////////
int RX64speed = 200;            // 1 to 1023... 1 is minimum speed, 1023 is maximum speed
int mode = 0;                   // where we are in the frame
int pan_low = 0;                // first byte to pan servo
int pan_high = 2;               // second byte to pan servo  //512 is starting position = 10 0000 0000
int tilt_low = 0;               // first byte to tilt servo
int tilt_high = 2;              // second byte to tilt servo  //512 is start position = 10 0000 0000
int checksum = 0;               // to check whether packet was received intact
int error = 1;                  // is 1 if checksum FAILS to match, otherwise zero
int gun = 0;                    // squirt gun on or off


////////////////////////////////SENSOR VARIABLES//////////////////////////////////
SharpIR FrontIR = SharpIR(GP2D12,1);
SharpIR LeftIR = SharpIR(GP2D12,2);
SharpIR RightIR = SharpIR(GP2D12,6);
int front_distance;    //variable for output voltage from front IR sensor
int left_distance;    //variable for output voltage from left IR sensor
int right_distance;      //variable for output voltage from right IR sensor
long encoder1;          //store encoder readings
long encoder2;
long previous_encoderL = 0;
long previous_encoderR = 0;
long time_since_zeroL = 0;
long time_since_zeroR = 0;
int stoppedflagL = 0;
int stoppedflagR = 0;
int signbitencoder2 = 0;  //used to deal with sending negative numbers over serial
int signbitencoder1 = 0;
bool debug = 1;  //set to 1 to send data to Processing, default is 1



///////////////////////////////STALL CHECK CODE/////////////////////////////////////////////////////
// returns 0 if no stall
// returns 1 if stall
////////////////////////////////////////////////////////////////////////////////////////////////////
int stallcheck_left(int min_encoder_counts)
{
  long current_encoder = Encoders.left;  //read encoder
  if(abs(current_encoder - previous_encoderL) < min_encoder_counts)
    {
      previous_encoderL = current_encoder;
      return 1;  // means we are stalled
    }
    else 
    {
      previous_encoderL = current_encoder;
      return 0;  //we are not stalled
    }
 }

int stallcheck_right(int min_encoder_counts)
{
  long current_encoder = Encoders.right;  //read encoder
  if(abs(current_encoder - previous_encoderR) < min_encoder_counts)
    {
      previous_encoderR = current_encoder;
      return 1;  // means we are stalled
    }
    else 
    {
      previous_encoderR = current_encoder;
      return 0;  //we are not stalled
    }
 }



////////////////////////THIS FUNCTION SENDS SPEED AND POSITION DATA TO 2 RX64s/////////////////////////////////////////////////
void syncwrite(int highbyte_servo1, int lowbyte_servo1, int highbyte_servo2, int lowbyte_servo2, int RXspeed)
{
  setTXall();
  ax12writeB(0xFF); //start
  ax12writeB(0xFF); //start
  ax12writeB(0xFE); //ID for broadcast
  ax12writeB(0x0E); //length = 14 (L+1)*N + 4, where L = 4 for position +speed data, and N = 2 for two RX64s
  ax12writeB(0x83);  //sync write instruction
  ax12writeB(0x1E);  //Goal_Position_L (lower bit of goal position address in RAM
  ax12writeB(0x04);  //length of data for all individual servos = 4 chars
  ax12writeB(0x01);  //ID servo 1
  /////////////////////////DATA SERVO1///////////////////////////
  ax12writeB(lowbyte_servo1&0xFF);  //send goal position data
  ax12writeB(highbyte_servo1&0xFF);  //
  ax12writeB(RXspeed&0xFF);                    //send move speed data
  ax12writeB((RXspeed>>8)&0x03);
  ////////////////////////////////////////////////////////////////
  ax12writeB(0x02);  //ID servo 2
  /////////////////////////////DATA SERVO2///////////////////////////
  ax12writeB(lowbyte_servo2&0xFF);  //send goal position data
  ax12writeB(highbyte_servo2&0xFF);  // 
  ax12writeB(RXspeed&0xFF);                    //send move speed data
  ax12writeB((RXspeed>>8)&0x03);
  ///////////////////////////////////////////////////////////////
  int checksum = ~((254 + 14 + 131 + 30 + 4 + 1 + (lowbyte_servo1&0xFF) + (highbyte_servo1&0xFF) + (RXspeed&0xFF) + ((RXspeed>>8)&0x03) + 2 + (highbyte_servo2&0xFF) + (lowbyte_servo2&0xFF) + (RXspeed&0xFF) + ((RXspeed>>8)&0x03))%256);
  ax12writeB(checksum); //checksum ~(FE + A + 83 + 1E + 2 + 1 + lowbyte_servo1&0xFF + highbyte_servo1&OxFF + 2 + highbyte_servo2&OxFF + lowbyte_servo2&OxFF)
}

void setup(){
    Encoders.Begin();
    Encoders.Reset();
    Serial.begin(38400);     //serial connector to computer
    leftdrive.attach(4);    // left drive HB25 controller to digital pin 4
    rightdrive.attach(2);   // right drive HB25 controller to digital pin 2
    pinMode(0,OUTPUT);          // status LED
    pinMode(7, OUTPUT);          // gun pin
    ////////////////////SETUP 6DOF//////////////////////////////////
    Wire.begin();
    delay(5);
    my3IMU.init();
    delay(5);
}

void loop(){
  
    if(use_stall_checks)
    {
      if((millis() - time_since_zeroL) > 200) {stoppedflagL = 1;}  //means Left motor has been stopped
      else {stoppedflagL = 0;}
      if((millis() - time_since_zeroR) > 200) {stoppedflagR = 1;}  //means Right motor has been stopped
      else {stoppedflagR = 0;}
    }     
  ////////////////////////////////// READING DATA FROM PROCESSING/////////////////////////////////
    while(Serial.available() > 0)
    {
        // We need to 0xFF at start of packet
        if(mode == 0)  // start of new packet
        {         
            if(Serial.read() == 0xFF)
            {
                mode = 1;
                digitalWrite(0,HIGH-digitalRead(0));
            }
        }
        else if(mode == 1)
        {   // next byte is pan first byte
            pan_low = Serial.read();  
            mode = 2;
        }
        else if(mode == 2)
        {   // next byte is pan second byte
            pan_high = Serial.read();
            mode = 3;
        }
        else if(mode == 3)
        {   // next byte is tilt first byte
            tilt_low = Serial.read();
            mode = 4;
        }
        else if(mode == 4)
        {   // next byte is tilt second byte
            tilt_high = Serial.read();
            mode = 5;
        }
        else if(mode == 5)
        {   // next byte is gun
            gun = Serial.read();
            mode = 6;   
        }  
        else if(mode == 6)
        {   // next byte is leftm otor power
            leftmotor = Serial.read();
            mode = 7;   
        }  
        else if(mode == 7)
        {   // next byte is right motor power
            rightmotor = Serial.read();
            mode = 8;   
        }  
        else if(mode == 8)
        {   // next byte is stallreset
            stallreset = Serial.read();
            mode = 9;   //that finishes the packet
        }
        else if(mode == 9)
        {   // next byte is checksum
            checksum = Serial.read();
            mode = 0;   //that finishes the packet
        }    
        if(checksum == (pan_low + pan_high + tilt_low + tilt_high + gun + leftmotor + rightmotor + stallreset)%256)
        {
          error = 0;  //packet received intact
        }
        else {error = 1;}  //problem with packet
    }
    
    if(leftmotor < 30 || rightmotor < 30 || leftmotor > 225 || rightmotor > 225){error = 1;}
   
    ///////////////////////////////////////TURN ON EVERYTHING///////////////////////////////////////////////////
    if(!error)  //only execute commands if packet was received intact
    {
      if(gun) {   digitalWrite(7,HIGH);     }    //turn on/off squirt gun
      else { digitalWrite(7,LOW); }
      syncwrite(pan_high, pan_low, tilt_high, tilt_low, RX64speed);   //move RX64 turret to position
      
      /////////////////////////////CONTROL LEFT MOTOR WITH STALL CHECK INCORPORATED////////////////////////////////////
    if(use_stall_checks)
    {
      if(stoppedflagL ==1 && abs(leftmotor - 127) > 3  ) // we were stopped but now power is applied again
      {
          leftdrive.writeMicroseconds(map(leftmotor, 0, 255, 1000, 2000));  //move briefly
          delay(300);
      }
      else 
      {
       if(abs(leftmotor - 127) > 3)  //power is applied
        {
          if(!stallcheck_left(5)) //not stalled
          {
             leftmotor_mapped = map(leftmotor, 0, 255, 1000, 2000);  // convert from power to servo pulse... 128 power corresponds to 1500 us pulse
             stall_left = 0;
          }
          else  // stalled
          {
            leftmotor_mapped = 1500;  // convert from power to servo pulse... 128 power corresponds to 1500 us pulse
            stall_left = 1;
          }
        }
        else
        {
          leftmotor_mapped = 1500;  // stop motor
        }
      }
    }
    else 
      leftmotor_mapped = map(leftmotor, 0, 255, 1000, 2000);  // convert from power to servo pulse... 128 power corresponds to 1500 us pulse 
      /////////////////////////////CONTROL RIGHT MOTOR WITH STALL CHECK INCORPORATED////////////////////////////////////  
    if(use_stall_checks)
    {
      if(stoppedflagR ==1 && abs(rightmotor - 127) > 3)  // we were stopped but now power is applied again
      {
          rightdrive.writeMicroseconds(map(rightmotor, 0, 255, 1000, 2000));  //move briefly
          delay(150);
      }
      else 
      {
       if(abs(rightmotor - 127) > 3)  //power is applied
        {
          if(!stallcheck_right(5)) //not stalled
          {
             rightmotor_mapped = map(rightmotor, 0, 255, 1000, 2000);  // convert from power to servo pulse... 128 power corresponds to 1500 us pulse
             stall_right = 0;
          }
          else  // stalled
          {
            rightmotor_mapped = 1500;  // convert from power to servo pulse... 128 power corresponds to 1500 us pulse
            stall_right = 1;
          }
        }
        else
        {
          rightmotor_mapped = 1500;  // stop motor
        }
      }
    }
    else
      rightmotor_mapped = map(rightmotor, 0, 255, 1000, 2000);  // convert from power to servo pulse... 128 power corresponds to 1500 us pulse
    
    rightdrive.writeMicroseconds(rightmotor_mapped);  // send a servo pulse to the HB25s */ 
    leftdrive.writeMicroseconds(leftmotor_mapped);  // send a servo pulse to the HB25s */ 
    if(use_stall_checks)
    {
      if(abs(rightmotor - 127) > 3) {time_since_zeroR = millis();}  //reset timers for the last time that a zero power was read
      if(abs(leftmotor - 127) > 3) {time_since_zeroL = millis();}
    }
   }
    ///////////////////////////////////////////// READ SENSORS /////////////////////////////////////////////
    front_distance = FrontIR.getData(); 
    left_distance = LeftIR.getData();
    right_distance = RightIR.getData();
    encoder1 = Encoders.left;
    if(encoder1 < 0) {
      encoder1 = abs(encoder1);  
      signbitencoder1 = 255; }
    else {signbitencoder1 = 10;}
    encoder2 = Encoders.right; 
    if(encoder2 < 0) {
      encoder2 = abs(encoder2);  
      signbitencoder2 = 255; }
    else {signbitencoder2 = 10;}  
    my3IMU.getQ(q);
      
    ////////////////////////////////////////SEND DATA TO PROCESSING////////////////////////////////////////////
    if(debug)
    {
      //Serial.print('F');
      Serial.print(front_distance);  // first byte of front distance
      Serial.print(",");
      Serial.print(left_distance);  // first byte of left distance
      Serial.print(",");
      Serial.print(right_distance);  // first byte of left distance
      Serial.print(",");
      Serial.print(encoder1&0xFF);  // first byte
      Serial.print(",");
      Serial.print((encoder1>>8)&0xFF);  // second byte
      Serial.print(",");
      Serial.print((encoder1>>16)&0xFF);  // third byte
      Serial.print(",");
      Serial.print((encoder1>>24)&0xFF);  // fourth byte
      Serial.print(",");
      Serial.print(signbitencoder1);  // sign byte
      Serial.print(",");
      Serial.print(encoder2&0xFF);  // first byte
      Serial.print(",");
      Serial.print((encoder2>>8)&0xFF);  // second byte
      Serial.print(",");
      Serial.print((encoder2>>16)&0xFF);  // third byte
      Serial.print(",");
      Serial.print((encoder2>>24)&0xFF);  // fourth byte
      Serial.print(",");
      Serial.print(signbitencoder2);  // sign byte
      Serial.print(",");
      Serial.print(stall_left);  // first byte of left distance
      Serial.print(",");
      Serial.print(stall_right);  // first byte of left distance
      Serial.print(",");
      serialPrintFloatArr(q, 4);
      Serial.println(""); //line break
    }
    delay(50);
    
}
