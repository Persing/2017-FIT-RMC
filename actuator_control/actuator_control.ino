//Author - Nicholas Persing
#include <Wire.h>

#define leftPowerPin 5
#define rightPowerPin 6
#define leftDirectionPin 7
#define rightDirectionPin 8
#define leftLimitSwitch 11
#define rightLimitSwitch 12


#define i2cAddress 16


int dir = LOW;

unsigned long prevTime = 0;
int timeout = 120; //timeout limit to prevent runaway incase of signal loss
int leftPowerLevel = 0;
int rightPowerLevel = 0;


void setup() {
  Serial.begin(9600);
  
  pinMode(leftPowerPin, OUTPUT);
  pinMode(leftDirectionPin, OUTPUT);
  pinMode(leftLimitSwitch, INPUT); //limit switch wasn't implemented on the hardware.
  pinMode(rightPowerPin, OUTPUT);
  pinMode(rightDirectionPin, OUTPUT);
  pinMode(rightLimitSwitch, INPUT);//limit switch wasn't implemented on the hardware.

  Wire.begin(i2cAddress);     
  Wire.onReceive(setDirection);
  delay(110);
}

void loop() {
  if(millis() - prevTime < timeout){ // if last instruction has not timed out
    
    if(digitalRead(leftLimitSwitch) == HIGH && dir == HIGH){ //if at limit, stop
      leftPowerLevel = 0;
    }else{
      leftPowerLevel = 255;
    }
    
    if(digitalRead(rightLimitSwitch) == HIGH && dir == HIGH){ //if at limit, stop
      rightPowerLevel = 0;
    }else{
      rightPowerLevel = 255;
    }
    
    analogWrite(leftPowerPin, leftPowerLevel); // write power to the motor controller 
    digitalWrite(leftDirectionPin, dir);      //write the direction to motor controller
    analogWrite(rightPowerPin, rightPowerLevel);
    digitalWrite(rightDirectionPin, dir);
   
   }else{ //if the timeout time has passed since last instruction shut off motor controllers
    analogWrite(leftPowerPin, 0);  
    analogWrite(rightPowerPin, 0);
  }
}

void setDirection(int howMany){
  noInterrupts(); //prevent any interrupts from corrupting i2c read
  int x = Wire.read();
  if(x == 1){ //extend actuator - on collection bin the motors are reversed
    dir = HIGH;
  }else if(x == 0){ //retract actuator 
    dir = LOW;
  }
  Serial.println(x);
  prevTime = millis(); // mark time for last instruction received 
  interrupts(); //reenable interrupts 
}

