//Author - Nicholas Persing
//
#include <Wire.h>

#define powerPin 5
#define directionPin 7


#define i2cAddress 12

// Impeller - 14
// Auger - 12


int dir = HIGH; //check this is the right value for the correct forward rotation 

unsigned long prevTime = 0;
int timeout = 100; // determines how long the driver can operate withou user input -- not used for auger or impeller currently!!
int powerLevel = 0;


void setup() {
  //sets corresponding pins for output so they can write to the motor controller
  pinMode(powerPin, OUTPUT); 
  pinMode(directionPin, OUTPUT);
  
  Wire.begin(i2cAddress);     
  Wire.onReceive(setDirection);
  delay(timeout+10); // saftey to ensure that the device is automactly timedout at start so no motors will turn on without user input
}

void loop() {
//  if(millis() - prevTime < timeout){
    analogWrite(powerPin, powerLevel);
    digitalWrite(directionPin, dir);
//   }else{
//    analogWrite(powerPin, 0);
//  }
}

void setDirection(int howMany){
  noInterrupts();
  int x = Wire.read();

  if(x == 0){         //forward operation 
    dir = HIGH;
    powerLevel = 150; //power level min 0, max 255
  }else if(x == 1){  //reverse 
    dir = LOW;
    powerLevel = 150; //power level min 0, max 255
  }else{            //shutdown 
    powerLevel = 0; 
  }
  prevTime = millis();
  interrupts();
}

