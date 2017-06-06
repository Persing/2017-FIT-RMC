//Author - Nicholas Persing
#include <PID_v1.h> //Pid controll library, availble in Arduino libraries, must download before use
#include <Encoder.h>//use encoder library from Arduino library, 
                    //must download it before this can be compiled
#include <Servo.h>//uses servo becuase PWM signals the Arduino sends 
                  //is a differnt frequency then the Talon srx expects, 
                  //but servo can be used instead of standard pwm
#include <Wire.h>

#define pwmPin 5
#define i2cAddress 8 

//wheels | number - i2c adress
//#1 - 8
//#2 - 9
//#3 - 10
//#4 - 11

//Wheel locations - top down view

//  Front
//  2---1
//  |   |
//  |   |
//  |   |
//  3---4

#define timeout 100 //alter this to change how long the driver can continue without user input

  
double RPM, //The curent RPM of the wheels
       percentPower = 0, // The output signal to the drive controller
       targetRPM; //The desired RPM of the wheels

Servo controller; //The pwm for the Talon SRX motorcontrollers don't 
                  //match up with arduino's pwm output. However the Servo library does

Encoder encoder(2,3); // New quadature encoder using interupt pins 2 and 3

long encoderPOS = 0, prevPOS = 0; //the current and previous position of the encoder

unsigned long curTime, prevTime, lastCMD; // Current and previous time

PID drivePID( &RPM,//gives the pid controller access to current RPM as input
              &percentPower, //output for the pid loop
              &targetRPM, //this is the target the PID loop is trending towards
              0.8, // Proportional 0.8
              0.1, // Integral     0.1
              0.02, // Derivitive  0.02
              DIRECT);

int dirModifier = 1; //see dirModifier comments in setup()
  
void setup() {
  
  // since motors on one side of the robot will need to to rotate clockwise while the other is counter-clockwise
  //one side of the robots wheel rotations should be reversed
  //if(i2cAddress == 1 || i2cAddress == 4) 
    //dirModifier = -1;
  
  Serial.begin(9600);
  Serial.println("starting...");

  controller.attach(pwmPin);
  Serial.println("PWM controller ready");

  targetRPM = 0; //no motion is desired yet
  drivePID.SetOutputLimits(-100, 100); // min of -100% and max of 100% output power from the PID controller
  drivePID.SetSampleTime(50); //sets a faster sample time than default 
  drivePID.SetMode(MANUAL); // PID controller is only active when set to AUTOMATIC 
  Serial.println("PID control ready.");

  Wire.begin(i2cAddress); //join i2c bus with adress 8
  Wire.onReceive(updateRPM); //calls updateRPM() when data is received to 
  Serial.println("Done. System Ready.");

}

void loop() {
  encoderPOS = encoder.read(); //updates the encoder position should be called each cycle
  int PMWvalue = percentPower * 5 + 1500; //scales pmw output to 1000-2000
  controller.writeMicroseconds(PMWvalue); //writes the pmw value to the ESC 
  
  if(targetRPM == 0){ //turns off the pid loop and shuts down the motors
    drivePID.SetMode(MANUAL);
    percentPower = 0; 
  }else{ // turns on the pid controller
    drivePID.SetMode(AUTOMATIC); 
  }

  curTime = millis();// retreives current time
  if (curTime - prevTime >= 20){ // set time interval for rpm calculation
    double revs = (encoderPOS-prevPOS)/(49.4*80.0); //converts the differnce in encoder positions to number of rotation since last check
    RPM=revs*60000.0/(double)(curTime - prevTime); //converts the number of rotations to RPM
    prevTime = curTime; // saves the time that the last check ran
    prevPOS = encoderPOS; //saves the previous encoder position at last check
  }
  
  drivePID.Compute(); //updates the pid loop, only changes the value once each sample time but this should be called every loop

  // when the controller hasn't recieved a command for longer than timeout it shuts down the motor to prevent run away when connection is lost
  if(curTime - lastCMD >= timeout){ 
    targetRPM = 0;
  }
  
  Serial.println(targetRPM);

}

void updateRPM(int howMany){
  noInterrupts(); //prevents the interupts from the encoder from interfereing with the i2c communication
  int x = Wire.read(); //receive byte as an integer
  targetRPM = 2 * dirModifier * ((double) (x) - 50); //0 - 100, 50 is stop -----! 255 is max foward, 0 is max reverse and 128 is stop
  lastCMD = millis();
  interrupts(); //re-enables the interrups for the encoder
}


