#include <LiquidCrystal.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define up 0
#define down 1
#define right 2
#define left 3
#define select 4
#define non 5
/* Define modes
    1: Control
    2: IR
    3: Wall follow
*/
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
int mode = 0;
String info;
double infoSolved;
int state = 0;
double distance;
double distanceClosest = 100;
double distanceWall1, distanceWall2, distanceWall, distanceFront, calibrate;
int angle, angleClosest;
char angleString[10], aString[10], bString[10], cString[10], dString[10], eString[10], fString[10];
int rotRightIR;
void PrintMessage(String message)
{
  Serial.print(message);
  Serial.write(13);
  Serial.write(10);
}
/*****CONTROL MANUAL*****/
void turnRight(unsigned int a) {
  if (userPress() == right) {
    while (userPress() == right) {
      itoa(a, aString, 10);
      Serial.setTimeout(100);
      PrintMessage(String("CMD_ACT_ROT_1_") + aString);
      wait(30);
    }
  }
}
void turnLeft(unsigned int b) {
  if (userPress() == left) {
    while (userPress() == left) {
      itoa(b, bString, 10);
      Serial.setTimeout(100);
      PrintMessage(String("CMD_ACT_ROT_0_") + bString);
      wait(30);
    }
  }
}
void forward(unsigned int c) {
  if (userPress() == up) {
    while (userPress() == up) {
      itoa(c, cString, 10);
      Serial.setTimeout(100);
      PrintMessage(String("CMD_ACT_LAT_1_") + cString);
      wait(300);
    }
  }
}
void backward(unsigned int d) {
  if (userPress() == down) {
    while (userPress() == down) {
      itoa(d, dString, 10);
      Serial.setTimeout(100);
      PrintMessage(String("CMD_ACT_LAT_0_") + dString);
      wait(300);
    }
  }
}
void returnMenu() {
  if (userPress() == select) {
    while (userPress() == select);
    lcd.clear();
    mode = 0;
    state = 0;
    angle = 0;
    angleClosest = 0;
    distanceClosest = 100;
  }
}
/*****CONTROL AUTO*****/
void turnRightAuto(unsigned int a) {
  itoa(a, aString, 10);
  Serial.setTimeout(100);
  PrintMessage(String("CMD_ACT_ROT_1_") + aString);
  wait(10);
}
void turnLeftAuto(unsigned int b) {
  itoa(b, bString, 10);
  Serial.setTimeout(100);
  PrintMessage(String("CMD_ACT_ROT_0_") + bString);
  wait(10);
}
void forwardAuto(double x) {
  itoa(x, cString, 10);
  Serial.setTimeout(100);
  PrintMessage(String("CMD_ACT_LAT_1_") + cString);
  wait(300);
}
void backwardAuto(double y) {
  itoa(y, dString, 10);
  Serial.setTimeout(100);
  PrintMessage(String("CMD_ACT_LAT_0_") + dString);
  wait(300);
}
/*****ROTATE SENSOR*****/
void receiveIR() {
  Serial.setTimeout(100);
  PrintMessage("CMD_SEN_IR");
  wait(10);
  while (!Serial.available());
  String info = Serial.readStringUntil('\n');
  infoSolved = info.toDouble();
  distance = infoSolved;
  if (isnan(infoSolved)) distance = 5;
}
void rotateRightIR(unsigned int e) {
  Serial.setTimeout(100);
  rotRightIR = 360 - e;
  itoa(rotRightIR, eString, 10);
  PrintMessage(String("CMD_SEN_ROT_") + eString);
}
void rotateLeftIR(unsigned int f) {
  Serial.setTimeout(100);
  itoa(f, fString, 10);
  PrintMessage(String("CMD_SEN_ROT_") + fString);
}
void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.setCursor(0, 1);
  wait(1000);                // Wait 1 second
  PrintMessage("CMD_START");  // Start the robot
  userPress();
  screenStartUp();
  cli(); // clear all interrupt
  //setup timer 2 for delaying
  TCCR2A = 0;
  TCCR2B |= (1 << CS21) | (1 << CS22) | (1 << CS20);
  TCNT2 = 0;
  // set up for user input
  ADMUX |= (1 << REFS0);//AVcc w external capacitor at AREF pin
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // division 128
  sei();
}
int userPress()
{
  ADMUX &= ~(1 << MUX3) & ~(1 << MUX2) & ~(1 << MUX1) & ~(1 << MUX0);
  ADCSRA |= (1 << ADSC) ;
  while (!(ADCSRA & (1 << ADIF))) ;
  ADCSRA |= (1 << ADIF);
  if (ADC > 900) return non;
  if (ADC < 900) {
    wait(20);
    if (ADC < 55) return right;
    if (ADC < 185) return up;
    if (ADC < 395) return down;
    if (ADC < 587) return left;
    if (ADC < 780) return select;
  }
  return non; // Set NoDirection always pressing
}

void loop() {
  userPress();
  if (state == 0) {
    selectModes();
  }
  switch (mode) {
    case 0:
      screenStartUp();
      break;
    case 1:
      screenControl();
      break;
    case 2:
      screenSweep();
      break;
    case 3:
      screenWallFollow();
      break;
  }
}
void studentID() {
  lcd.setCursor(0, 0);
  lcd.print("13758317");
}
void screenStartUp() {
  studentID();
  lcd.setCursor(0, 1);
  lcd.print("Main menu");
}
/*****CONTROL MODE*****/
void screenControl() {
  studentID();
  lcd.setCursor(0, 1);
  lcd.print("Control");
  /*User press select to turn on control*/
  if (userPress() == select && state == 0) {
    while (userPress() == select);
    lcd.clear();
    lcd.setCursor(12, 1);
    lcd.print("On");
    state = 1;
  }
  /*Control mode ON*/
  if (state == 1) {
    turnRight(1);
    turnLeft(1);
    forward(1);
    backward(1);
    returnMenu();
  }
}
/*****SWEEP MODE*****/
void screenSweep() {
  studentID();
  lcd.setCursor(0, 1);
  lcd.print("Sweep");
  /*User press select to select Sweep*/
  if (userPress() == select && state == 0) {
    while (userPress() == select);
    lcd.clear();
    lcd.setCursor(12, 1);
    lcd.print("Wait");
    state = 1;
  }
  if (state == 1) {
    if (userPress() == up) {
      while (userPress() == up);
      state = 2;
    }
  }
  /***** Sweep ON *****/
  sweepFunction();
}

void sweepFunction() {
  if (state == 2) {
    lcd.setCursor(12, 1);
    lcd.print("On  ");
    rotateRightIR(angle);
    angle++;
    receiveIR();
    if (isnan(distance)) distance = 5;
    if (distance < distanceClosest) {
      distanceClosest = distance;
      angleClosest = angle;
    }
    if (angle == 360) {
      angle = 360 - angleClosest;
      state = 3;
    }
  }
  if (state == 3) {
    turnLeftAuto(1);
    angle--;
    if (angle == -1) state = 4;
  }
  if (state == 4) {
    /** Reset variable and go back main menu **/
    if (userPress() == select && state != 0) {
      returnMenu();
    }
  }
}

void screenWallFollow() {
  studentID();
  lcd.setCursor(0, 1);
  lcd.print("Wall follow");
  /** TURN ON WALL FOLLOW **/
  if (userPress() == select && state == 0) {
    while (userPress() == select);
    lcd.clear();
    lcd.setCursor(12, 1);
    lcd.print("ON");
    state = 2;
  }
  /** TURN OFF WALL FOLLOW AND GO BACK MAIN MENU **/
  returnMenu();
  /* Start Sweep*/
  sweepFunction();
  /** FINISHED SWEEP, START WALL FOLLOW**/
  if (state == 4) {
    rotateLeftIR(0);
    receiveIR();
    distanceFront = distance;
    returnMenu();
    if (distanceFront <= 2) {
      PrintMessage("CMD_ACT_LAT_0_0.3");
      turnRightAuto(90);
    }
    returnMenu();
    if (distanceFront > 2) PrintMessage("CMD_ACT_LAT_1_1");
    rotateLeftIR(70);
    returnMenu();
    wait(10);
    receiveIR();
    distanceWall1 = distance;
    rotateLeftIR(80);
    returnMenu();
    receiveIR();
    distanceWall2 = distance;
    /** Calculate distanceWall **/
    distanceWall =  (distanceWall1 * distanceWall2 * sin(0.2618)) / sqrt(distanceWall1 * distanceWall1 + distanceWall2 * distanceWall2 - 2 * distanceWall1 * distanceWall2 * cos(0.2618));
    if (isnan(distanceWall) || distanceWall == 0 || isnan(distanceWall1) || isnan(distanceWall2)) distanceWall = 5;
    /*** Fix car distance ***/
    if (distanceWall < 1.5) {
      turnRightAuto(90);
      PrintMessage("CMD_ACT_LAT_1_0.5");
      turnLeftAuto(90);
      returnMenu();
    }
    if (distanceWall > 2.5) {
      turnRightAuto(90);
      PrintMessage("CMD_ACT_LAT_0_0.5");
      turnLeftAuto(90);
      returnMenu();
    }
    if (distanceWall < 2 && distanceWall > 1.5 ) {
      turnRightAuto(1);
      wait(50);
      returnMenu();
    }
    if (distanceWall > 2 && distanceWall < 2.5) {
      turnLeftAuto(1);
      wait(50);
      returnMenu();
    }
  }
}
void selectModes() {
  if (userPress() == down) {
    while (userPress() == down);
    lcd.clear();
    mode++;
  }
  if (mode > 3) mode = 0;
}

void wait(unsigned int milisecond) /* Delay for button*/
{
  unsigned int timerCount = 0;
  TCCR2A = 0; /*Normal mode*/
  TCCR2B = (1 << CS22); /*prescaler 1/64*/
  TCNT2 = 5;
  while (timerCount < milisecond)
  {
    if (TIFR2 & 0x01) // check if 1ms has occurred
    {
      timerCount++;
      TCNT2 = 6;
      TIFR2 &= (1 << TOV2);
    }
  }
}
