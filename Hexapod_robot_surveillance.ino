#include <Adafruit_PWMServoDriver.h>
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  125                                                 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  625                                                 // this is the 'maximum' pulse length count (out of 4096)
#define DHTPIN 2 
#define DHTTYPE DHT11 
/* The main algorithm behind the movement of hexapod to implement the wave gate
 */
int mq=A0;
int x;

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
DHT dht(DHTPIN, DHTTYPE);

// each servo motor is positioned at a specific angle in order to keep the hexapod in standing condition.
#define normal_leg11 50
#define normal_leg12 60
#define normal_leg13 105 //close to body
#define normal_leg21 65
#define normal_leg22 50
#define normal_leg23 65 //close to body
#define normal_leg31 60
#define normal_leg32 60
#define normal_leg33 30 //close to body
#define normal_leg41 50
#define normal_leg42 35
#define normal_leg43 100 //close to body
#define normal_leg51 65
#define normal_leg52 60
#define normal_leg53 65 //close to body
#define normal_leg61 45
#define normal_leg62 60
#define normal_leg63 39 //close to body

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// delay of 25 milliseconds is given between // consecutive rotations of different servos.
#define DELAY 100


char command='n';

void setup() {
  Serial.begin(9600);

  pwm1.begin();
  pwm1.setPWMFreq(SERVO_FREQ);  // This is the PWM1 frequencY

  pwm2.begin();
  pwm2.setPWMFreq(SERVO_FREQ);  // This is the PWM2 frequency

  Serial.println(F("DHT ON"));
  dht.begin();

  pinMode(A0,INPUT);
}

void loop() {

//DHT11
  delay(1000);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));

//MQ6 sensor
  x=analogRead(A0);
  Serial.println(x);
  delay(500);

//hexapod
  if (Serial.available()>0) {
    command = Serial.read();
    Serial.print("Received input: ");
    Serial.println(command);
    Serial.read();
  }
    if(command=='f'){
      Serial.println("Forward");
      Serial.println(command);
      moveForward();
    }
  
    else if(command=='b'){
      Serial.println("back");
      Serial.println(command);
      moveBackward();
    }
    else if(command=='r'){
      Serial.println("right");
      Serial.println(command);
      moveRight();
    }
    else if(command=='l'){
      Serial.println("left");
      Serial.println(command);
      moveLeft();
    }
    else if(command=='c'){
      Serial.println("clockwise");
      Serial.println(command);
      turnClockWise();
    }
    else if(command=='a'){
      Serial.println("anticlockwise");
      Serial.println(command);
      turnAntiClockWise();
    }
    else if(command=='n'){
      Serial.println(command);
      move2normalAngle();
    }
    else {
      Serial.println("Invalid input");
    }
}

//gets angle in degree and returns the pulse width
uint16_t atop(uint16_t angle)
{  
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// This function will move all servos to their normal angle. 
void move2normalAngle() 
{
  // Leg 1
  pwm1.setPWM(0,0,(atop(normal_leg11)));
  delay(15);
  pwm1.setPWM(1,0,(atop(normal_leg12)));
  delay(15);
  pwm1.setPWM(3,0,(atop(normal_leg13)));
  delay(15);
  // Leg 2
  pwm1.setPWM(4,0,(atop(normal_leg21)));
  delay(15);
  pwm1.setPWM(6,0,(atop(normal_leg22)));
  delay(15);
  pwm1.setPWM(8,0,(atop(normal_leg23)));
  delay(15);
  // Leg 3
  pwm1.setPWM(10,0,(atop(normal_leg31)));
  delay(15);
  pwm1.setPWM(12,0,(atop(normal_leg32)));
  delay(15);
  pwm1.setPWM(14,0,(atop(normal_leg33)));
  delay(15);
  // Leg 4
  pwm2.setPWM(0,0,(atop(normal_leg41)));
  delay(15);
  pwm2.setPWM(1,0,(atop(normal_leg42)));
  delay(15);
  pwm2.setPWM(3,0,(atop(normal_leg43)));
  delay(15);
  // Leg 5
  pwm2.setPWM(4,0,(atop(normal_leg51)));
  delay(15);
  pwm2.setPWM(6,0,(atop(normal_leg52)));
  delay(15);
  pwm2.setPWM(8,0,(atop(normal_leg53)));
  delay(15);
  // Leg 6
  pwm2.setPWM(10,0,(atop(normal_leg61)));
  delay(15);
  pwm2.setPWM(12,0,(atop(normal_leg62)));
  delay(15);
  pwm2.setPWM(14,0,(atop(normal_leg63)));
  delay(15);

  Serial.println("normal position");
}
 
// This function will move hexapod in forward direction
void moveForward() 
{
  // Leg 1
  pwm1.setPWM(0,0,(atop(normal_leg11 + 10)));
  delay(DELAY);
  pwm1.setPWM(3,0,(atop(normal_leg13 - 20)));
  delay(DELAY);
  pwm1.setPWM(0,0,(atop(normal_leg11 - 10)));
  delay(DELAY);
  // Leg 5
  pwm2.setPWM(4,0,(atop(normal_leg51 + 10)));
  delay(DELAY);
  pwm2.setPWM(8,0,(atop(normal_leg53 + 20)));
  delay(DELAY);
  pwm2.setPWM(4,0,(atop(normal_leg51 - 10)));
  delay(DELAY);
  // Leg 3
  pwm1.setPWM(10,0,(atop(normal_leg31 + 10)));
  delay(DELAY);
  pwm1.setPWM(14,0,(atop(normal_leg33 - 20)));
  delay(DELAY);
  pwm1.setPWM(10,0,(atop(normal_leg31 - 10)));
  delay(DELAY);

  delay(DELAY);
  
  // Leg 6
  pwm2.setPWM(10,0,(atop(normal_leg61 + 10)));
  delay(DELAY);
  pwm2.setPWM(14,0,(atop(normal_leg63 + 20)));
  delay(DELAY);
  pwm2.setPWM(10,0,(atop(normal_leg61 - 10)));
  delay(DELAY);
  // Leg 2
  pwm1.setPWM(4,0,(atop(normal_leg21 + 10)));
  delay(DELAY);
  pwm1.setPWM(8,0,(atop(normal_leg23 - 20)));
  delay(DELAY);
  pwm1.setPWM(4,0,(atop(normal_leg21 - 10)));
  delay(DELAY);
  // Leg 4
  pwm2.setPWM(0,0,(atop(normal_leg41 + 10)));
  delay(DELAY);
  pwm2.setPWM(3,0,(atop(normal_leg43 + 20)));
  delay(DELAY);
  pwm2.setPWM(0,0,(atop(normal_leg41 - 10)));
  delay(DELAY);

  // move all servos to their normal angle
  move2normalAngle();
  delay(DELAY);
}
 
// This function will move hexapod in backward direction
void moveBackward() 
{
  // Leg 6
  pwm2.setPWM(10,0,(atop(normal_leg61 + 10)));
  delay(DELAY);
  pwm2.setPWM(14,0,(atop(normal_leg63 - 20)));
  delay(DELAY);
  pwm2.setPWM(10,0,(atop(normal_leg61 - 10)));
  delay(DELAY);
  // Leg 4
  pwm2.setPWM(0,0,(atop(normal_leg41 + 10)));
  delay(DELAY);
  pwm2.setPWM(3,0,(atop(normal_leg43 - 20)));
  delay(DELAY);
  pwm2.setPWM(0,0,(atop(normal_leg41 - 10)));
  delay(DELAY);
  // Leg 2
  pwm1.setPWM(4,0,(atop(normal_leg21 + 10)));
  delay(DELAY);
  pwm1.setPWM(8,0,(atop(normal_leg23 + 20)));
  delay(DELAY);
  pwm1.setPWM(4,0,(atop(normal_leg21 - 10)));
  delay(DELAY);
  
  delay(DELAY);

  // Leg 3
  pwm1.setPWM(10,0,(atop(normal_leg31 + 10)));
  delay(DELAY);
  pwm1.setPWM(14,0,(atop(normal_leg33 + 20)));
  delay(DELAY);
  pwm1.setPWM(10,0,(atop(normal_leg31 - 10)));
  delay(DELAY);
  // Leg 5
  pwm2.setPWM(4,0,(atop(normal_leg51 + 10)));
  delay(DELAY);
  pwm2.setPWM(8,0,(atop(normal_leg53 - 20)));
  delay(DELAY);
  pwm2.setPWM(4,0,(atop(normal_leg51 - 10)));
  delay(DELAY);
  // Leg 1
  pwm1.setPWM(0,0,(atop(normal_leg11 + 10)));
  delay(DELAY);
  pwm1.setPWM(3,0,(atop(normal_leg13 + 20)));
  delay(DELAY);
  pwm1.setPWM(0,0,(atop(normal_leg11 - 10)));
  delay(DELAY);
  // move all servos to their normal angle
  move2normalAngle();
  delay(15);
}
 
// This function will move hexapod towards right
void moveRight() 
{
  // Leg 1
  pwm1.setPWM(1,0,(atop(normal_leg12 + 20)));
  delay(DELAY);
  pwm1.setPWM(0,0,(atop(normal_leg11 - 20)));
  delay(DELAY);
  pwm1.setPWM(1,0,(atop(normal_leg12)));
  delay(DELAY);
  // Leg 3
  pwm1.setPWM(12,0,(atop(normal_leg32 + 20)));
  delay(DELAY);
  pwm1.setPWM(10,0,(atop(normal_leg31 - 20)));
  delay(DELAY);
  pwm1.setPWM(12,0,(atop(normal_leg32)));
  delay(DELAY);
  // Leg 5
  pwm2.setPWM(6,0,(atop(normal_leg52 + 20)));
  delay(DELAY);
  pwm2.setPWM(4,0,(atop(normal_leg51 + 20)));
  delay(DELAY);
  pwm2.setPWM(6,0,(atop(normal_leg52)));
  delay(DELAY);

  delay(DELAY);

  // Leg 2
  pwm1.setPWM(6,0,(atop(normal_leg22 + 20)));
  delay(DELAY);
  pwm1.setPWM(4,0,(atop(normal_leg21 - 20)));
  delay(DELAY);
  pwm1.setPWM(6,0,(atop(normal_leg22)));
  delay(DELAY);
  // Leg 4
  pwm2.setPWM(1,0,(atop(normal_leg42 + 20)));
  delay(DELAY);
  pwm2.setPWM(0,0,(atop(normal_leg41 + 20)));
  delay(DELAY);
  pwm2.setPWM(1,0,(atop(normal_leg42)));
  delay(DELAY);
  // Leg 6
  pwm2.setPWM(12,0,(atop(normal_leg62 + 20)));
  delay(DELAY);
  pwm2.setPWM(10,0,(atop(normal_leg61 + 20)));
  delay(DELAY);
  pwm2.setPWM(12,0,(atop(normal_leg62)));
  // move all servos to their normal angle
  move2normalAngle();
  delay(DELAY);
}

// This function will move hexapod towards left 
void moveLeft()  {
  // Leg 1
  pwm1.setPWM(1,0,(atop(normal_leg12 + 20)));
  delay(DELAY);
  pwm1.setPWM(0,0,(atop(normal_leg11 + 20)));
  delay(DELAY);
  pwm1.setPWM(1,0,(atop(normal_leg12)));
  delay(DELAY);
  // Leg 3
  pwm1.setPWM(12,0,(atop(normal_leg32 + 20)));
  delay(DELAY);
  pwm1.setPWM(10,0,(atop(normal_leg31 + 20)));
  delay(DELAY);
  pwm1.setPWM(12,0,(atop(normal_leg32)));
  delay(DELAY);
  // Leg 5
  pwm2.setPWM(6,0,(atop(normal_leg52 + 20)));
  delay(DELAY);
  pwm2.setPWM(4,0,(atop(normal_leg51 - 20)));
  delay(DELAY);
  pwm2.setPWM(6,0,(atop(normal_leg52)));
  delay(DELAY);

  delay(DELAY);
  
  // Leg 2
  pwm1.setPWM(6,0,(atop(normal_leg22 + 20)));
  delay(DELAY);
  pwm1.setPWM(4,0,(atop(normal_leg21 + 20)));
  delay(DELAY);
  pwm1.setPWM(6,0,(atop(normal_leg22)));
  delay(DELAY);
  // Leg 4
  pwm2.setPWM(1,0,(atop(normal_leg42 + 20)));
  delay(DELAY);
  pwm2.setPWM(0,0,(atop(normal_leg41 - 20)));
  delay(DELAY);
  pwm2.setPWM(1,0,(normal_leg42));
  delay(DELAY);
  // Leg 6
  pwm2.setPWM(12,0,(atop(normal_leg62 + 20)));
  delay(DELAY);
  pwm2.setPWM(10,0,(atop(normal_leg61 - 20)));
  delay(DELAY);
  pwm2.setPWM(12,0,(atop(normal_leg62)));
  delay(DELAY);
  
  // move all servos to their normal angle
  move2normalAngle();
  delay(DELAY);
}
 
// this function will turn hexapod in clockwise direction 
void turnClockWise() 
{
  // Leg 1
  pwm1.setPWM(0,0,(atop(normal_leg11 + 10)));
  delay(DELAY);
  pwm1.setPWM(3,0,(atop(normal_leg13 - 25)));
  delay(DELAY);
  pwm1.setPWM(0,0,(atop(normal_leg11)));
  delay(DELAY);
  // Leg 3
  pwm1.setPWM(10,0,(atop(normal_leg31 + 10)));
  delay(DELAY);
  pwm1.setPWM(14,0,(atop(normal_leg33 - 25)));
  delay(DELAY);
  pwm1.setPWM(10,0,(atop(normal_leg31)));
  delay(DELAY);
  // Leg 5
  pwm2.setPWM(4,0,(atop(normal_leg51 + 10)));
  delay(DELAY);
  pwm2.setPWM(8,0,(atop(normal_leg51 - 25)));
  delay(DELAY);
  pwm2.setPWM(4,0,(atop(normal_leg51)));
  delay(DELAY);

  delay(DELAY);
  
  // Leg 2
  pwm1.setPWM(4,0,(atop(normal_leg21 + 10)));
  delay(DELAY);
  pwm1.setPWM(8,0,(atop(normal_leg23 - 25)));
  delay(DELAY);
  pwm1.setPWM(4,0,(atop(normal_leg21)));
  delay(DELAY);
  // Leg 4
  pwm2.setPWM(0,0,(atop(normal_leg41 + 10)));
  delay(DELAY);
  pwm2.setPWM(3,0,(atop(normal_leg43 - 25)));
  delay(DELAY);
  pwm2.setPWM(0,0,(atop(normal_leg41)));
  delay(DELAY);
  // Leg 6
  pwm2.setPWM(10,0,(atop(normal_leg61 + 10)));
  delay(DELAY);
  pwm2.setPWM(14,0,(atop(normal_leg63 - 25)));
  delay(DELAY);
  pwm2.setPWM(10,0,(atop(normal_leg61)));
  delay(DELAY);
  
  // move all servos to their normal angle
  move2normalAngle();
  delay(DELAY);
}

// This function will turn hexapod in anti-clockwise direction 
void turnAntiClockWise() 
{
  // Leg 1
  pwm1.setPWM(0,0,(atop(normal_leg11 + 10)));
  delay(DELAY);
  pwm1.setPWM(3,0,(atop(normal_leg13 + 25)));
  delay(DELAY);
  pwm1.setPWM(0,0,(atop(normal_leg11)));
  delay(DELAY);
  // Leg 3
  pwm1.setPWM(10,0,(atop(normal_leg31 + 10)));
  delay(DELAY);
  pwm1.setPWM(14,0,(atop(normal_leg33 + 25)));
  delay(DELAY);
  pwm1.setPWM(10,0,(atop(normal_leg31)));
  delay(DELAY);
  // Leg 5
  pwm2.setPWM(4,0,(atop(normal_leg51 + 10)));
  delay(DELAY);
  pwm2.setPWM(8,0,(atop(normal_leg51 + 25)));
  delay(DELAY);
  pwm2.setPWM(4,0,(atop(normal_leg51)));
  delay(DELAY);

  delay(DELAY);
  
  // Leg 2
  pwm1.setPWM(4,0,(atop(normal_leg21 + 10)));
  delay(DELAY);
  pwm1.setPWM(8,0,(atop(normal_leg23 + 25)));
  delay(DELAY);
  pwm1.setPWM(4,0,(atop(normal_leg21)));
  delay(DELAY);
  // Leg 4
  pwm2.setPWM(0,0,(atop(normal_leg41 + 10)));
  delay(DELAY);
  pwm2.setPWM(3,0,(atop(normal_leg43 + 25)));
  delay(DELAY);
  pwm2.setPWM(0,0,(atop(normal_leg41)));
  delay(DELAY);
  // Leg 6
  pwm2.setPWM(10,0,(atop(normal_leg61 + 10)));
  delay(DELAY);
  pwm2.setPWM(14,0,(atop(normal_leg63 + 25)));
  delay(DELAY);
  pwm2.setPWM(10,0,(atop(normal_leg61)));
  delay(DELAY);
  
  // move all servos to their normal angle
  move2normalAngle();
  delay(DELAY);
}
