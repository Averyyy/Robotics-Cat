//left front down: 0 
//left back down: 1
//right front down: 2
//right back down: 3

//up:
//left front : 4 
//left back : 5
//right front : 6
//right back : 7

//head: 12



#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//#include "WriteInstinct/OpenCat.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  130 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  470 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

//sensor
int previous = 0;
boolean pao = true;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  /*
     In theory the internal oscillator (clock) is 25MHz but it really isn't
     that precise. You can 'calibrate' this by tweaking this number until
     you get the PWM update frequency you're expecting!
     The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     is used for calculating things like writeMicroseconds()
     Analog servos run at ~50 Hz updates, It is importaint to use an
     oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     1) Attach the oscilloscope to one of the PWM signal pins and ground on
        the I2C PCA9685 chip you are setting the value for.
     2) Adjust setOscillatorFrequency() until the PWM update frequency is the
        expected value (50Hz for most ESCs)
     Setting the value here is specific to each individual I2C PCA9685 chip and
     affects the calculations for the PWM update frequency.
     Failure to correctly set the int.osc value will cause unexpected PWM results
  */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  //sensor
//  SensorSetup();
  pao = true;
  previous = 0;
  previous = analogRead(A0);
  
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
//  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
//  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
//  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}


void loop() {
  //previous
//  initiate();
//  delay(1000);
//  stand();
//  delay(1000);
//  test();
//  delay(1000);
  int ran = random(10);
  int sensorValue = analogRead(A0);
  Serial.println(sensorValue);

  delay(100);        // delay in between reads for stability

    if(sensorValue > 700){
     stand();
     delay(1000);
     test();
     delay(1000);
    }
    else if(sensorValue < 400){

      pwm.setPWM(4,0,SERVOMIN);
      pwm.setPWM(6,0,SERVOMAX);
      pwm.setPWM(0,0,300);
      pwm.setPWM(2,0,300);
      delay(100);
    }
    else{
      if (ran == 6){
        tail();
        delay(100);
      }
      else if (ran == 7){
        pwm.setPWM(12, 0 , 300);
        pwm.setPWM(12, 0, 230);
        delay(100);
      }
      else if(ran == 8){
        Serial.print("af");
        af();
      }
      
      initiate();
      
    }

  delay(10);
  

}

void tail(){
  pwm.setPWM(8,0, SERVOMIN);
  pwm.setPWM(8,0, SERVOMAX);
}

void af(){
    for (uint16_t servonum = 0; servonum < 2; servonum++) {
    pwm.setPWM(servonum, 0, SERVOMAX-200);

  }
  for (uint16_t servonum = 2; servonum < 4; servonum++) {
    pwm.setPWM(servonum, 0, SERVOMIN+200);

  }
  }
void initiate(){
//  down
//  for (uint16_t servonum = 0; servonum < 2; servonum++) {
//    pwm.setPWM(servonum, 0, SERVOMIN);
//    Serial.println(SERVOMIN);
//  }
//  for (uint16_t servonum = 2; servonum < 4; servonum++) {
//    pwm.setPWM(servonum, 0, SERVOMAX);
//    Serial.println(SERVOMAX);
//  }
  for (uint16_t servonum = 0; servonum < 2; servonum++) {
    pwm.setPWM(servonum, 0, SERVOMAX);

  }
  for (uint16_t servonum = 2; servonum < 4; servonum++) {
    pwm.setPWM(servonum, 0, SERVOMIN);

  }
   pwm.setPWM(12, 0, SERVOMIN+100);
   
   for (uint16_t servonum = 4; servonum < 8; servonum++) {
    pwm.setPWM(servonum, 0, 300);
//    Serial.print(pulselen);
  } 
  
  
//  //up
//    uint16_t pulselen = (SERVOMAX - SERVOMIN)/2 + SERVOMIN;
//  for (uint16_t servonum = 4; servonum < 8; servonum++) {
//    pwm.setPWM(servonum, 0, pulselen);
//    Serial.print(pulselen);
//  }


  
  
  }
void stand() {
  //
  //  for (uint16_t pulselen = 0; pulselen < 150; pulselen++) {
  //      for (uint16_t servonum = 0; servonum < 2; servonum++){
  //        pwm.setPWM(servonum, 0, pulselen);
  //      }
  //      delay(25);
  //    }
  //
  //  for (uint16_t pulselen = 150; pulselen > 0; pulselen-) {
  //      for (uint16_t servonum = 0; servonum < 2; servonum++){
  //        pwm.setPWM(servonum, 0, pulselen);
  //      }
  //      delay(25);
  //    }
  //


  //

// I calibrate all servos to 1/2 of total length
  uint16_t pulselen = (SERVOMAX - SERVOMIN)/2 + SERVOMIN;
  for (uint16_t servonum = 0; servonum < 2; servonum = servonum+1) {
    pwm.setPWM(servonum, 0, pulselen);
//    Serial.print(pulselen);
  }
  delay(500);
  for (uint16_t servonum = 2; servonum < 4; servonum = servonum+1) {
    pwm.setPWM(servonum, 0, pulselen);
//    Serial.print(pulselen);
  }
    pwm.setPWM(12, 0, pulselen+70);
  

//slow
//for (uint16_t i = SERVOMIN; i < pulselen; i++) {
//      for (uint16_t servonum = 0; servonum < 1; servonum++) {
//        pwm.setPWM(servonum, 0, SERVOMAX + SERVOMIN - i);
//      }
//      for (uint16_t servonum = 2; servonum < 3; servonum++) {
//        pwm.setPWM(servonum, 0,  i);
//      }
//  
//      delay(25);
//    }
  
  //up

  
  for (uint16_t servonum = 4; servonum < 8; servonum++) {
    pwm.setPWM(servonum, 0, pulselen);
//    Serial.print(pulselen);
  } 





  //
  //  for (uint16_t pulselen = 0; pulselen < 300; pulselen++) {
  //    for (uint16_t servonum = 0; servonum < 1; servonum++){
  //    pwm.setPWM(servonum, 0, pulselen);
  //    }
  //  }
  //
  //  delay(1500);
  //
  //  for (uint16_t pulselen = 300; pulselen > 600; pulselen--) {
  //    for (uint16_t servonum = 0; servonum < 2; servonum++){
  //    pwm.setPWM(servonum, 0, pulselen);
  //    }
  //  }
  //
  //  delay(1500);
  //
}

void test(){
  
  //this to move

  uint16_t pulselen = (SERVOMAX - SERVOMIN)/2 + SERVOMIN;
  for (uint16_t servonum = 2; servonum < 4; servonum = servonum+1) {
    pwm.setPWM(servonum, 0, pulselen);
//    Serial.print(pulselen);
  }
  
    for (uint16_t i = SERVOMIN; i < SERVOMAX-100; i++) {
        pwm.setPWM(0, 0, SERVOMAX + SERVOMIN - i);
      delay(25);
    }

    pwm.setPWM(4,0,SERVOMAX);
    
  }
