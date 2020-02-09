/*
    Copyright (C) 2019  Heriberto Leyva Díaz de León   
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License <http://www.gnu.org/licenses/> for more details.
    
    Creators whose codes inspired this project: 
    Debra Ansell : http://www.geekmomprojects.com/mpu-6050-redux-dmp-data-fusion-vs-complementary-filter/
 */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
MPU6050 mpu;
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno 

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float sensor_data[3];

unsigned long cal_drift_time;//used to get the drif
float gyro_yaw_drift;//calculated drift over time

float pitch_angle ;// angle used to compare in the PID
float roll_angle ;// angle used to compare in the PID
float yaw_angle ,yaw_angle_rate ; 

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
//safety variables
boolean SAFE_START = false;
boolean SPIN_MOTORS = true;
boolean MOVE_SERVOS = true;
boolean REBOOT = true ;
// variables for the final PWM signal for the four outputs, two ESC and two servos
unsigned long servo_right_timer , servo_left_timer, esc_right_timer , esc_left_timer;
unsigned long pwm_start_timer;
unsigned long pwm_timer;
//variables to add the PID outputs to the servo and ESC PWMs
int servo_right_adder;
int servo_left_adder;
int esc_right_adder;
int esc_left_adder;
//ISR variables to get inputs from the RC receiver
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, current_time, esc_loop_timer;
int receiver_ch1, receiver_ch2, receiver_ch3, receiver_ch4, receiver_ch5;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5;
int  yaw_rc, pitch_rc, throttle_rc, roll_rc, swa_rc;
//PID variables declaration
float error;
float Pterm_pitch, Iterm_pitch, Dterm_pitch, last_error_pitch, PID_pitch_out;
float Pterm_roll, Iterm_roll, Dterm_roll, last_error_roll, PID_roll_out;
float Pterm_yaw, Iterm_yaw, Dterm_yaw, last_error_yaw, PID_yaw_out;
//setpoint declaration
float set_pitch = 0 ;
float set_roll =  0 ;
float set_yaw = 0 ;

//+40deg =  2070 us,, -40deg = 1160 us,, 910 us range,, 1615 us = 0deg
const int MAX_pitch_output = 454 ;//454
const int MAX_roll_output  = 454 ;
const int MAX_yaw_output   = 454 ;
//PID constant values for all three axis
const float Kp_pitch = .25 ;//.5
const float Ki_pitch = 0 ;
const float Kd_pitch = 3.3;

const float Kp_roll = .4; //.2
const float Ki_roll = 0 ;
const float Kd_roll = 4.8;

const float Kp_yaw = 0.15 ;//.5
const float Ki_yaw = 0 ;
const float Kd_yaw = 3 ;
//time measuring variables for yaw angle calculations
float time_end , time_start, time_wait;
//int  sample_rate_us = 10000 ; //just in case I want to make this more robust in the future

float abs_yaw_angle = 0;
float yaw_adder;



void setup() {
    Serial.begin(115200);
  Serial.println("init\nRegister setup...");
  delay(500);
  PCICR |= (1 << PCIE2);                              // set PCIE2 to enable PCMSK2 scan
  PCMSK2 |= (1 << PCINT19);                           // set PCINT19 (digital input 3) to trigger an interrupt
  PCMSK2 |= (1 << PCINT20);                           // set PCINT20 (digital input 4) to trigger an interrupt
  PCMSK2 |= (1 << PCINT21);                           // set PCINT21 (digital input 5) to trigger an interrupt
  PCMSK2 |= (1 << PCINT22);                           // set PCINT22 (digital input 6) to trigger an interrupt
  PCMSK2 |= (1 << PCINT23);                           // set PCINT23 (digital input 7) to trigger an interrupt
  Serial.println("Register setup done.");
  delay( 500 );
  
  DDRB |= B00011110;//pins nine through twelve as outputs

  //Calibrate ESCs
  if( receiver_ch3 < 1010 ){
  SAFE_START = true;          //only proceed if the throttle is down
  }  
  if (MCUSR == 2){
    REBOOT = false ;          //do not proceed if the board was reset but not powered off
  }
  MCUSR = 0;                  //reset the reboot variable for future use
  if(SPIN_MOTORS && SAFE_START && REBOOT){ // if conditions are met, proceed with the ESC calibration

    for(int i = 2070; i > 1160; i-=4){

      unsigned long loop_start_t = micros();

      PORTB |= B00011000;

      delayMicroseconds(i);

      PORTB &= B00000000;

      while(loop_start_t + 4000 > micros());

    }

  }
  SAFE_START = false ;        //reset variable for future use

   // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

  // calculate the drift over time in 1000 readings in five millisecond intervals
  //this specific code will get the drift over five seconds and then average it
  for ( int i = 0; i < 1000 ; i++ ){
    cal_drift_time = micros() ;
    gyro_yaw_drift += mpu.getRotationZ();
    while( cal_drift_time + 5000 > micros() );
  }
  gyro_yaw_drift /= 1000;
  Serial.print("drifting by :");
        Serial.print(gyro_yaw_drift);
        Serial.println();


}


void loop() {

if( receiver_ch3 < 1010 ){
  SAFE_START = true;        //only start the control loop if the throttle is down
}
delay(100);
//-----------------------------------------------------------------------------
// start the control loop
//-----------------------------------------------------------------------------
while( SAFE_START ){
  time_start = micros();    //start time count in orde to calculate angles
//-----------------------------------------------------------------------------
// read the sensor data
//-----------------------------------------------------------------------------
getSensorData(); // get precise angle data from the DMP from the MPU6050

// get the not super precise data from the gyro, but something that can
// achieve yaw rotations without problem
yaw_angle_rate = mpu.getRotationZ();//get the rate of rotation 
yaw_angle_rate -= gyro_yaw_drift;   //substract the drift for accuracy

//calculate the absolute angle from  zero starting point
//this can go 400 or more degs or -400 or less degs for unrestricted yaw rotation
abs_yaw_angle =  abs_yaw_angle + (yaw_angle_rate/16.4)*time_wait;
//-----------------------------------------------------------------------------
// read receiver input
//-----------------------------------------------------------------------------
pitch_rc =    map( receiver_ch2, 1000, 2000, 1160, 2070);//input mapping to have all ESC and servos
throttle_rc = map( receiver_ch3, 1000, 2000, 1160, 2070);//be in the same range of operations
roll_rc =     map( receiver_ch4, 1000, 2000, 1160, 2070);//in regards to pwm 

//1615 is midpoint or 0deg

// set a dead band at zero to improve stability on pitch
if( pitch_rc > 1623 ) set_pitch = pitch_rc  ;
else if( pitch_rc <= 1607 ) set_pitch = pitch_rc  ;
else set_pitch = 1615 ;

// smooth set point creation for yaw
// this control loop goes at around 8.5 ms, that means 118 Hz
//This would mean an 180 deg yaw movement in about 2.1 secs
if( receiver_ch1 > 1515 ) {
  set_yaw = set_yaw + .7; // .7 is the rate of change of yaw
}                         // more than .7 means faster yaw rotation
                          // less than .7 means slower yaw rotation
else if( receiver_ch1 < 1485 ){
  set_yaw = set_yaw - .7;
}

// set a dead band at zero to improve stability on roll
if( roll_rc > 1623 ) set_roll = roll_rc ;
else if( roll_rc < 1607 ) set_roll =roll_rc ;
else set_roll = 1615 ;

//-----------------------------------------------------------------------------
// calculate PID
//-----------------------------------------------------------------------------
//pitch PID calculations
error = set_pitch - pitch_angle * 11.125 - 1615;
Pterm_pitch = error * Kp_pitch ;
Iterm_pitch += error ;
Dterm_pitch = (error - last_error_pitch) * Kd_pitch ;
last_error_pitch = error ;
PID_pitch_out = Pterm_pitch + (Iterm_pitch * Ki_pitch) + Dterm_pitch ;
if( PID_pitch_out > MAX_pitch_output ) PID_pitch_out = MAX_pitch_output;
if( PID_pitch_out < -MAX_pitch_output ) PID_pitch_out = -MAX_pitch_output;
error = set_roll + roll_angle * 11.125 -1615;
//roll PID calculations
Pterm_roll = error * Kp_roll ;
Iterm_roll += error ;
Dterm_roll = (error - last_error_roll) * Kd_roll ;
last_error_roll = error ;
PID_roll_out = Pterm_roll + (Iterm_roll * Ki_roll) + Dterm_roll ;
if( PID_roll_out > MAX_roll_output ) PID_roll_out = MAX_roll_output;
if( PID_roll_out < -MAX_roll_output ) PID_roll_out = -MAX_roll_output;
// yaw PID calculations
error = (set_yaw + abs_yaw_angle) * 11.125 ;
Pterm_yaw = error * Kp_yaw ;
Iterm_yaw += error ; 
Dterm_yaw = (error -last_error_yaw) * Kd_yaw ;
last_error_yaw = error ;
PID_yaw_out = Pterm_yaw + (Iterm_yaw * Ki_yaw) + Dterm_yaw ;
if( PID_yaw_out > MAX_yaw_output ) PID_yaw_out = MAX_yaw_output;
if( PID_yaw_out < -MAX_yaw_output ) PID_yaw_out = -MAX_yaw_output;
//-----------------------------------------------------------------------------
// write pwm to the servos and motors
//-----------------------------------------------------------------------------
servo_right_adder = 1615 + PID_pitch_out - PID_yaw_out ;
servo_left_adder  = 1615 - PID_pitch_out - PID_yaw_out + 185; // had to add some offset due to the latency sending signals
esc_right_adder =  throttle_rc + PID_roll_out -70;//70           for this ESC I had to do the same thing
esc_left_adder =   throttle_rc - PID_roll_out -0;

if( receiver_ch5 > 1500 ){ // safety feature, this kills the brushless motors if the
  SPIN_MOTORS = false;     // switch is flipped. Kill switch of spinning props
}
pwm_start_timer = micros();// pwm generation
servo_right_timer = pwm_start_timer + servo_right_adder ;
servo_left_timer  = pwm_start_timer + servo_left_adder ;
esc_right_timer   = pwm_start_timer + esc_right_adder ;
esc_left_timer    = pwm_start_timer + esc_left_adder ;
if(SPIN_MOTORS){// move ahead if the killswitch has not been flipped
  PORTB |= B00011000;
}
if(MOVE_SERVOS){// move ahead if this is true
  PORTB |= B00000110;
}
while(PORTB >= 2){// do not stop until all pwm signals have been sent, max time 2070 microseconds
  pwm_timer = micros();
  if(servo_right_timer <= pwm_timer) PORTB &= B11111101;// multiply by one to do not affect other outputs
  if(servo_left_timer <= pwm_timer) PORTB &=  B11111011;// multiply by zero to end pwm signal
  if(esc_right_timer <= pwm_timer) PORTB &=   B11110111;
  if(esc_left_timer <= pwm_timer) PORTB &=    B11101111;
}

time_end =micros();// end of the count of time
time_wait =(time_end-time_start)*.000001; //calculate cycle time, and convert to seconds to be used in yaw
                                          //angle calculations
}
}
//interrupt service routine for the input of the RC receiver
ISR(PCINT2_vect){
  current_time = micros();
  //Channel 1()=========================================
  //Channel 2(YAW)=========================================
  if(PIND & B00001000 ){                                       //Is input 9 high?
    if(last_channel_1 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_1 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_ch1 = current_time - timer_1;         //Channel 2 is current_time - timer_2
  }
  //Channel 3(PITCH)=========================================
  if(PIND & B00010000 ){                                       //Is input 10 high?
    if(last_channel_2 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_ch2 = current_time - timer_2;         //Channel 3 is current_time - timer_3
  }
  //Channel 4(THROTTLE)=========================================
  if(PIND & B00100000 ){                                       //Is input 11 high?
    if(last_channel_3 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_ch3 = current_time - timer_3;         //Channel 4 is current_time - timer_4
  }
  //Channel 5(ROLL)=========================================
  if(PIND & B01000000 ){                                       //Is input 12 high?
    if(last_channel_4 == 0){                                   //Input 12 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_5 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 12 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_ch4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
  //Channel 6(SWA)=========================================
  if(PIND & B10000000 ){                                       //Is input 13 high?
    if(last_channel_5 == 0){                                   //Input 13 changed from 0 to 1
      last_channel_5 = 1;                                      //Remember current input state
      timer_5 = current_time;                                  //Set timer_6 to current_time
    }
  }
  else if(last_channel_5 == 1){                                //Input 12 is not high and changed from 1 to 0
    last_channel_5 = 0;                                        //Remember current input state
    receiver_ch5 = current_time - timer_5;        
  }
}
// getting the angle data from the MPU6050 with the Digital Motion Processor from the MPU6050
void getSensorData ( ){
   // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
    }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  if(fifoCount < packetSize){
          //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
      // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
  while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
  }
        
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer); 
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(sensor_data, &q, &gravity);

            pitch_angle = sensor_data[1] * 180 / M_PI ;
            roll_angle  = sensor_data[2] * 180 / M_PI ;
            yaw_angle   = sensor_data[0] * 180 / M_PI ;
    }
    
}
