//Author - Nicholas Persing
#include <Encoder.h> //use encoder library from Arduino library, 
                     //must download it before this can be compiled
#include <Servo.h> //uses servo becuase PWM signals the Arduino sends 
                   //is a differnt frequency then the Talon srx expects, 
                   //but servo can be used instead of standard pwm
#include <Wire.h>

#define pmwPin 5

#define i2cAddress 14


double percent = 0; //percent power sent to the motor controller
double rpm, targetRPM;

Servo controller;
Encoder encoder(2,3);
long encoderPOS = 0;
unsigned long curTime = 0;
unsigned long prevTime = 0;
long lastPOS = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...\n");
  controller.attach(5);
  Wire.begin(i2cAddress);     
  Wire.onReceive(setRotation);
  targetRPM=0;
}

void loop() {
  encoderPOS = encoder.read();

  if(targetRPM != 0){
    readRPM();
    if(rpm <= targetRPM && percent <= 100){ // if the current rpm is lower than target increase the power
      if(abs(rpm - targetRPM)>=200 )// if its far lower increase power a significant amount
        percent += 5;
      else // if the rpm is close to target make smaller change
        percent += 1;
    }else if(rpm >= targetRPM && percent >= -100){ // rpm is too high, so reduce power
      if(abs(rpm - targetRPM)>=200 ) if the differnce is large, make large change
        percent -= 3;
      else //otherwise make a smaller change 
        percent -= 1;
    }
    Serial.println(rpm);
  }else{
    percent = 0;
  }

  int PMWvalue = percent * 5 + 1500; // converts the percent power to the correct pwm value, 1500 is 0, 1000 max reverse, 2000 max forward
  controller.writeMicroseconds(PMWvalue); // send the pwm signal
  delay(20);

}

double readRPM(){
  curTime = millis();
  double revs = (encoderPOS-lastPOS)/(80.0); //converts change in encoder postion to number of revolutions
  rpm=revs*60000.0/(double)(curTime - prevTime); // coverts the revolution to rpm 
  prevTime = curTime; //mark the last time rpm was measured 
  lastPOS = encoderPOS; // mark the last postion of the encoder
  return rpm;
}

void setRotation(int howMany){
  noInterrupts(); // this prevents encoder interrupts from currupting the i2c message
  int x = Wire.read(); //read value from i2c line
  if(x == 1){  //set to forward
    targetRPM = 2000;
  }else if(x == 0){ //set to reverse
    targetRPM = -2000;
  }else{ //shut off impeller
    targetRPM = 0;
  }
  interrupts(); //reenable the interrupts for encoder, some ticks may be missing but effect is minimal. 
}

