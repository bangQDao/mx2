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
#define water 1
#define fire 2
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
int second, minute;
String info;
double infoSolved;
double distanceWall, distanceWall1, distanceWall2, distance, distanceFront, allign;
int rotRightIR;
int x = 8;
int y = 16;
int pose = right;
int on = 0;
int k = 0;
int goalType, goalCollected, yold, xold, poseold;
int rightCheck, frontCheck, leftCheck;
char angleString[10], aString[10], bString[10], cString[10], dString[10], eString[10], fString[10];

int Map[19][20] = {
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
};


void PrintMessage(String message)
{
  Serial.print(message);
  Serial.write(13);
  Serial.write(10);
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
void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.setCursor(0, 1);
  wait(1000);                // Wait 1 second
  PrintMessage("CMD_START");  // Start the robot
  userPress();
  screenStartUp();
  cli(); // clear all interrupt
  // set up timer 1 for showing time
  TCCR1A = 0; // normal mode
  TCCR1B = 0; // clear clock
  TCNT1 = 0;
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); // prescaler 1024
  OCR1A = 15624; // 1/(16x10^6)*1024*15625 = 1s
  TIMSK1 |= (1 << OCIE1A);
  //setup timer 2 for delaying
  TCCR2A = 0;
  TCCR2B |= (1 << CS21) | (1 << CS22) | (1 << CS20);
  TCNT2 = 0;
  // set up for user input
  ADMUX |= (1 << REFS0);//AVcc w external capacitor at AREF pin
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // division 128
}
ISR(TIMER1_COMPA_vect) {
  second++;
  if (second > 59) {
    second = 0;
    minute++;
  }
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
void mapGoal() {
  if (y - 2 > 0 && x - 2 > 0 && Map[y - 2][x - 2] != 1 ) Map[y - 2][x - 2] = 2;
  if (y - 1 > 0 && x - 2 > 0 && Map[y - 1][x - 2] != 1 ) Map[y - 1][x - 2] = 2;
  if (y > 0 && x - 2 > 0 && Map[y][x - 2] != 1 ) Map[y][x - 2] = 2;
  if (y + 1 > 0 && x - 2 > 0 && Map[y + 1][x - 2] != 1 ) Map[y + 1][x - 2] = 2;
  if (y + 2 > 0 && x - 2 > 0 && Map[y + 2][x - 2] != 1 ) Map[y + 2][x - 2] = 2;

  if (y - 2 > 0 && x - 1 > 0 && Map[y - 2][x - 1] != 1 ) Map[y - 2][x - 1] = 2;
  if (y - 1 > 0 && x - 1 > 0 && Map[y - 1][x - 1] != 1 ) Map[y - 1][x - 1] = 2;
  if (y  > 0 && x - 1 > 0 && Map[y][x - 1] != 1) Map[y][x - 1] = 2;
  if (y + 1 > 0 && x - 1 > 0 && Map[y + 1][x - 1] != 1 ) Map[y + 1][x - 1] = 2;
  if (y + 2 > 0 && x - 1 > 0 && Map[y + 2][x - 1] != 1) Map[y + 2][x - 1] = 2;

  if (y - 2 > 0 && x  > 0 && Map[y - 2][x] != 1)Map[y - 2][x ] = 2;
  if (y - 1 > 0 && x  > 0 && Map[y - 1][x] != 1) Map[y - 1][x ] = 2;
  if (y  > 0 && x  > 0 && Map[y][x] != 1) Map[y][x] = 2;
  if (y + 1 > 0 && x  > 0 && Map[y + 1][x] != 1) Map[y + 1][x] = 2;
  if (y + 2 > 0 && x  > 0 && Map[y + 2][x] != 1) Map[y + 2][x] = 2;

  if (y - 2 > 0 && x + 1 > 0 && Map[y - 2][x + 1] != 1) Map[y - 2][x + 1] = 2;
  if (y - 1 > 0 && x + 1 > 0 && Map[y - 1][x + 1] != 1) Map[y - 1][x + 1] = 2;
  if (y  > 0 && x + 1 > 0 && Map[y][x + 1] != 1 ) Map[y ][x + 1] = 2;
  if (y + 1 > 0 && x + 1 > 0 && Map[y + 1][x + 1] != 1) Map[y + 1][x + 1] = 2;
  if (y + 2 > 0 && x + 1 > 0 && Map[y + 2][x + 1] != 1) Map[y + 2][x + 1] = 2;

  if (y - 2 > 0 && x + 2 > 0 && Map[y - 2][x + 2] != 1) Map[y - 2][x + 2] = 2;
  if (y - 1 > 0 && x + 2 > 0 && Map[y - 1][x + 2] != 1) Map[y - 1][x + 2] = 2;
  if (y  > 0 && x + 2 > 0 && Map[y][x + 2] != 1) Map[y][x + 2] = 2;
  if (y + 1 > 0 && x + 2 > 0 && Map[y + 1][x + 2] != 1) Map[y + 1][x + 2] = 2;
  if (y + 2 > 0 && x + 2 > 0 && Map[y + 2 ][x + 2] != 1) Map[y + 2 ][x + 2] = 2;
  yold = y;
  xold = x;
  poseold = pose;
}
void resetMap() {
  for (int i = 0; i < 21; i++)
  {
    for (int j = 0; j < 20; j++)
    {
      if (Map[j][i] == 2 )
      {
        Map[j][i] = 0;
      }
    }
  }
}
void checkGoal() {
  Serial.setTimeout(100);
  PrintMessage("CMD_SEN_ID");
  while (!Serial.available());
  String info = Serial.readStringUntil('\n');
  infoSolved = info.toDouble();
  goalType = infoSolved;
  checkGoalCollected();
  if (goalType == water && goalCollected == 0 && on == 1) {
    mapGoal();
    on = 3;
  }
  if (goalType == water && goalCollected == 0 && on == 2) {
    mapGoal();
    on = 4;
  }
  if (goalType == fire && goalCollected == water && on == 1) {
    mapGoal();
    on = 3;
  }
  if (goalType == fire && goalCollected == water) {
    mapGoal();
    on = 4;
  }
}
void checkGoalCollected() {
  PrintMessage("CMD_SEN_GOAL");
  while (!Serial.available());
  String info = Serial.readStringUntil('\n');
  infoSolved = info.toDouble();
  goalCollected = infoSolved;
  if (goalCollected == water && x == xold && y == yold && k == 0) {
    k = 1;
    if (on == 3) {
      resetMap();
        pose= poseold;
      on = 1;
    }
    if (on == 4) {
      resetMap();
        poseold = pose;
      on = 2;
    }
  }
  if (goalCollected == fire && pose == poseold && x == xold && y == yold && k == 1) {
    k = 2;
    if (on == 3) {
      resetMap();
      poseold = pose;
      on = 1;
    }
    if (on == 4) {
      resetMap();
      poseold = pose;
      on = 2;
    }
  }
}
/*****CONTROL AUTO*****/
void turnRight() {
  Serial.setTimeout(100);
  PrintMessage("CMD_ACT_ROT_1_90");
  wait(10);
}
void turnLeft() {
  Serial.setTimeout(100);
  PrintMessage("CMD_ACT_ROT_0_90");
  wait(10);
}
void forward(double x) {
  itoa(x, cString, 10);
  Serial.setTimeout(100);
  PrintMessage(String("CMD_ACT_LAT_1_") + cString);
  wait(300);
}
void backward(double y) {
  itoa(y, dString, 10);
  Serial.setTimeout(100);
  PrintMessage(String("CMD_ACT_LAT_0_") + dString);
  wait(300);
}
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
/**** Screen ****/
void screenStartUp() {
  lcd.setCursor(0, 1);
  lcd.print("13758317");
  if (minute > 10) {
    lcd.setCursor(0, 0);
    lcd.print(minute);
  }
  else {
    lcd.setCursor(1, 0);
    lcd.print(minute);
  }
  lcd.setCursor(2, 0);
  lcd.print(":");
  lcd.setCursor(3, 0);
  lcd.print(second);
  if (second == 0) {
    lcd.setCursor(4, 0);
    lcd.print(" ");
  }
}
void mode() {
  if (goalCollected == 0) {
    lcd.setCursor(10, 0); lcd.print("W");
  }
  if (goalCollected == water) {
    lcd.setCursor(10, 0); lcd.print("F");
  }
  if (goalCollected == fire && on != 5) {
    lcd.setCursor(10, 0); lcd.print("H");
  }
  if (on == 5) {
    lcd.setCursor(10, 0); lcd.print("C");
  }
}
void loop() { 
  for(int m =1; m<20;m++){
    if (Map[1][m]==1) Map[1][m]=0;
  }
 
  mode();
  userPress();
  screenStartUp();
  if (userPress() == select) {
    while (userPress() == select);
    sei();
    PrintMessage("CMD_ACT_LAT_1_7");
    on = 1;
  }
  if (goalCollected == fire && x == 8 && y == 16 && pose == left) {
    on = 5;
  }
  if (on == 1) {
    rightWallFollow();
  }
  if (x == 12 && y == 2&&on==1) {
    turnLeftAuto(180);
    forward(1);
    pose = left;
    on = 2;
    x = 11;
  }
  if (on == 2) {
    leftWallFollow();
  }
  if (on == 3) {
    rightWallFollowCollectGoal();
  }
  if (on == 4) {
    leftWallFollowCollectGoal();
  }
  if (on == 5) {
    forward(6);
    PrintMessage("CMD_CLOSE");
  }

}
void rightWallFollow() {
  if (on == 1) {
    if (pose == right) {
      rightCheck = y + 1;
      frontCheck = x + 1;
      if (Map[rightCheck][x] == 0) {
        turnRight();
        PrintMessage("CMD_ACT_LAT_1_1.2");
        y++;
        pose = down;
       if(xold==x&&yold==y) PrintMessage("CMD_CLOSE");
      }
      if (pose == right) {
        if (Map[rightCheck][x] == 1) {
          if (Map[y][frontCheck] == 1) {
            turnLeft();
            pose = up;
          }
          if (pose == right) {
            if (Map[y][frontCheck] == 0) {
              rightSideWall();
              x++;
            }
          }
        }
      }
    }
  }
  checkGoal();
  
  if (on == 1) {
    if (pose == down) {
      rightCheck = x - 1;
      frontCheck = y + 1;
      if (Map[y][rightCheck] == 0) {
        turnRight();
        PrintMessage("CMD_ACT_LAT_1_1.2");
        pose = left;
        x--;
      }
      if (pose == down) {
        if (Map[y][rightCheck] == 1) {
          if (Map[frontCheck][x] == 1) {
            turnLeft();
            pose = right;
          }
          if (pose == down) {
            if (Map[frontCheck][x] == 0) {
              rightSideWall();
              y++;
            }
          }
        }
      }
    }
  }
  checkGoal();
  
  if (on == 1) {
    if (pose == left) {
      rightCheck = y - 1;
      frontCheck = x - 1;
      if (Map[rightCheck][x] == 0) {
        turnRight();
        PrintMessage("CMD_ACT_LAT_1_1.2");
        pose = up;
        y--;
      }
      if (pose == left) {
        if (Map[rightCheck][x] == 1) {
          if (Map[y][frontCheck] == 1) {
            turnLeft();
            pose = down;
          }
          if (pose == left) {
            if (Map[y][frontCheck] == 0) {
              rightSideWall();
              x--;
            }
          }
        }
      }
    }
  }

  checkGoal();
  if (on == 1) {
    if (pose == up) {
      rightCheck = x + 1;
      frontCheck = y - 1;
      if (Map[y][rightCheck] == 0) {
        turnRight();
        PrintMessage("CMD_ACT_LAT_1_1.2");
        pose = right;
        x++;
      }
      if (pose == up) {
        if (Map[y][rightCheck] == 1) {
          if (Map[frontCheck][x] == 1) {
            turnLeft();
            pose = left;
          }
          if (pose == up) {
            if (Map[frontCheck][x] == 0) {
              rightSideWall();
              y--;
            }
          }
        }
      }
    }
  }
  checkGoal();
}

void rightWallFollowCollectGoal() {
   if (on == 3) {
    if (pose == right) {
      rightCheck = y + 1;
      frontCheck = x + 1;
      if (Map[rightCheck][x] == 2) {
        turnRight();
        PrintMessage("CMD_ACT_LAT_1_1");
        y++;
        pose = down;
      }
      if (pose == right) {
        if (Map[rightCheck][x] == 1||Map[rightCheck][x] == 0) {
          if (Map[y][frontCheck] == 1||Map[y][frontCheck] == 0) {
            turnLeft();
            pose = up;
            
          }
          if (pose == right) {
            if (Map[y][frontCheck] == 2) {
              rightSideWall();
              x++;
            }
          }
        }
      }
    }
  }
  checkGoalCollected();
  
  if (on == 3) {
    if (pose == down) {
      rightCheck = x - 1;
      frontCheck = y + 1;
      if (Map[y][rightCheck] == 2) {
        turnRight();
        PrintMessage("CMD_ACT_LAT_1_1");
        pose = left;
        x--;
      }
      if (pose == down) {
        if (Map[y][rightCheck] == 1||Map[y][rightCheck] == 0) {
          if (Map[frontCheck][x] == 1||Map[frontCheck][x] == 0) {
            turnLeft();
            pose = right;
          }
          if (pose == down) {
            if (Map[frontCheck][x] == 2) {
              rightSideWall();
              y++;
            }
          }
        }
      }
    }
  }
  checkGoalCollected();
  
  if (on == 3) {
    if (pose == left) {
      rightCheck = y - 1;
      frontCheck = x - 1;
      if (Map[rightCheck][x] == 2) {
        turnRight();
        PrintMessage("CMD_ACT_LAT_1_1");
        pose = up;
        y--;
      }
      if (pose == left) {
        if (Map[rightCheck][x] == 1||Map[rightCheck][x] == 0) {
          if (Map[y][frontCheck] == 1||Map[y][frontCheck] == 0) {
            turnLeft();
            pose = down;
          }
          if (pose == left) {
            if (Map[y][frontCheck] == 2) {
              rightSideWall();
              x--;
            }
          }
        }
      }
    }
  }

  checkGoalCollected();
  if (on == 3) {
    if (pose == up) {
      rightCheck = x + 1;
      frontCheck = y - 1;
      if (Map[y][rightCheck] == 2) {
        turnRight();
        PrintMessage("CMD_ACT_LAT_1_1");
        pose = right;
        x++;
      }
      if (pose == up) {
        if (Map[y][rightCheck] == 1||Map[y][rightCheck] == 0) {
          if (Map[frontCheck][x] == 1||Map[frontCheck][x] == 0) {
            turnLeft();
            pose = left;
          }
          if (pose == up) {
            if (Map[frontCheck][x] == 2) {
              rightSideWall();
              y--;
            }
          }
        }
      }
    }
  }
  checkGoalCollected();
}

void leftWallFollow() {
  if (on == 2) {
    if (pose == right) {
      leftCheck = y - 1;
      frontCheck = x + 1;
      if (Map[leftCheck][x] == 0) {
        turnLeft();
        PrintMessage("CMD_ACT_LAT_1_1.2");
        y--;
        pose = up;
      }
      if (pose == right) {
        if (Map[leftCheck][x] == 1) {
          if (Map[y][frontCheck] == 1) {
            turnRight();
            pose = down;
          }
          if (pose == right) {
            if (Map[y][frontCheck] == 0) {
              leftSideWall();
              x++;
            }
          }
        }
      }
    }
  }
  checkGoal();

  if (on == 2) {
    if (pose == down) {
      leftCheck = x + 1;
      frontCheck = y + 1;
      if (Map[y][leftCheck] == 0) {
        turnLeft();
        PrintMessage("CMD_ACT_LAT_1_1.2");
        pose = right;
        x++;
      }
      if (pose == down) {
        if (Map[y][leftCheck] == 1) {
          if (Map[frontCheck][x] == 1) {
            turnRight();
            pose = left;
          }
          if (pose == down) {
            if (Map[frontCheck][x] == 0) {
              leftSideWall();
              y++;
            }
          }
        }
      }
    }
  }

  checkGoal();
  if (on == 2) {
    if (pose == left) {
      leftCheck = y + 1;
      frontCheck = x - 1;
      if (Map[leftCheck][x] == 0) {
        turnLeft();
        PrintMessage("CMD_ACT_LAT_1_1.2");
        pose = down;
        y++;
      }
      if (pose == left) {
        if (Map[leftCheck][x] == 1) {
          if (Map[y][frontCheck] == 1) {
            turnRight();
            pose = up;
          }
          if (pose == left) {
            if (Map[y][frontCheck] == 0) {
              leftSideWall();
              x--;
            }
          }
        }
      }
    }

  }
  checkGoal();
  if (on == 2) {
    if (pose == up) {
      leftCheck = x - 1;
      frontCheck = y - 1;
      if (Map[y][leftCheck] == 0) {
        turnLeft();
        PrintMessage("CMD_ACT_LAT_1_1.2");
        pose = left;
        x--;
      }
      if (pose == up) {
        if (Map[y][leftCheck] == 1) {
          if (Map[frontCheck][x] == 1) {
            turnRight();
            pose = right;
          }
          if (pose == up) {
            if (Map[frontCheck][x] == 0) {
              leftSideWall();
              y--;
            }
          }
        }
      }
    }
  }
  checkGoal();
}
void leftWallFollowCollectGoal() {
  if (on == 4) {
    if (pose == right) {
      leftCheck = y - 1;
      frontCheck = x + 1;
      if (Map[leftCheck][x] == 2) {
        turnLeft();
        forward(1);
        y--;
        pose = up;
      }
      if (pose == right) {
        if (Map[leftCheck][x] == 1 || Map[leftCheck][x] == 0) {
          if (Map[y][frontCheck] == 1 || Map[y][frontCheck] == 0) {
            turnRight();
            pose = down;
          }
          if (pose == right) {
            if (Map[y][frontCheck] == 2) {
              leftSideWall();
              x++;
            }
          }
        }
      }
    }
  }
  checkGoalCollected();

  if (on == 4) {
    if (pose == down) {
      leftCheck = x + 1;
      frontCheck = y + 1;
      if (Map[y][leftCheck] == 2) {
        turnLeft();
        forward(1);
        pose = right;
        x++;
      }
      if (pose == down) {
        if (Map[y][leftCheck] == 1 || Map[y][leftCheck] == 0) {
          if (Map[frontCheck][x] == 1 || Map[frontCheck][x] == 0) {
            turnRight();
            pose = left;
          }
          if (pose == down) {
            if (Map[frontCheck][x] == 2) {
              leftSideWall();
              y++;
            }
          }
        }
      }
    }
  }

  checkGoalCollected();
  if (on == 4) {
    if (pose == left) {
      leftCheck = y + 1;
      frontCheck = x - 1;
      if (Map[leftCheck][x] == 2) {
        turnLeft();
        forward(1);
        pose = down;
        y++;
      }
      if (pose == left) {
        if (Map[leftCheck][x] == 1 || Map[leftCheck][x] == 0) {
          if (Map[y][frontCheck] == 1 || Map[y][frontCheck] == 0) {
            turnRight();
            pose = up;
          }
          if (pose == left) {
            if (Map[y][frontCheck] == 2) {
              leftSideWall();
              x--;
            }
          }
        }
      }
    }
  }
  checkGoalCollected();

  if (on == 4) {
    if (pose == up) {
      leftCheck = x - 1;
      frontCheck = y - 1;
      if (Map[y][leftCheck] == 2) {
        turnLeft();
        forward(1);
        pose = left;
        x--;
      }
      if (pose == up) {
        if (Map[y][leftCheck] == 1 || Map[y][leftCheck] == 0) {
          if (Map[frontCheck][x] == 1 || Map[frontCheck][x] == 0) {
            turnRight();
            pose = right;
          }
          if (pose == up) {
            if (Map[frontCheck][x] == 2) {
              leftSideWall();
              y--;
            }
          }
        }
      }
    }
  }
  checkGoalCollected();
}
void rightSideWall() {
  rotateLeftIR(0);
  receiveIR();
  distanceFront = distance;
  rotateRightIR(70);
  wait(10);
  receiveIR();
  distanceWall1 = distance;
  rotateRightIR(80);
  receiveIR();
  distanceWall2 = distance;
  /** Calculate distanceWall **/
  distanceWall =  (distanceWall1 * distanceWall2 * sin(0.2618)) / sqrt(distanceWall1 * distanceWall1 + distanceWall2 * distanceWall2 - 2 * distanceWall1 * distanceWall2 * cos(0.2618));
  if (isnan(distanceWall) || distanceWall == 0 || isnan(distanceWall1) || isnan(distanceWall2)) distanceWall = 5;

  if (distanceWall < 0.6 && distanceWall > 0.4) {
    Serial.setTimeout(100);
    turnLeftAuto(1);
    wait(50);
  }
  if (distanceWall >= 0.6 && distanceWall <= 0.9) {
    Serial.setTimeout(100);
    turnRightAuto(1);
  }
  if (distanceWall <= 0.4) {
    Serial.setTimeout(100);
    turnLeft();
    wait(50);
    Serial.setTimeout(100);
    PrintMessage("CMD_ACT_LAT_1_0.3");
    wait(100);
    Serial.setTimeout(100);
    turnRightAuto(88);
    wait(50);
  }
  if (on == 1 || on == 2) {
    if (distanceWall >= 0.9 && distanceWall < 2.3) {
      Serial.setTimeout(100);
      turnRight();
      wait(10);
      Serial.setTimeout(100);
      PrintMessage("CMD_ACT_LAT_1_0.3");
      wait(100);
      Serial.setTimeout(100);
      turnLeftAuto(88);
      wait(50);
    }
    if (isnan(distanceFront)) {
      distanceFront = 5;
    }
    if (distanceFront > 1.5) {
      Serial.setTimeout(100);
      PrintMessage("CMD_ACT_LAT_1_1");
      wait(100);
    }
    if (distanceFront <= 1.5) {
      Serial.setTimeout(100);
      PrintMessage("CMD_ACT_LAT_1_0.3");
      wait(100);
    }
  }
  if (on == 3 || on == 4) {
    if (isnan(distanceFront)) {
      distanceFront = 5;
    }
    if (distanceFront > 1.5) {
      Serial.setTimeout(100);
      PrintMessage("CMD_ACT_LAT_1_0.8");
      wait(100);
    }
    if (distanceFront <= 1.5) {
      Serial.setTimeout(100);
      PrintMessage("CMD_ACT_LAT_1_0.3");
      wait(100);
    }
  }
}
void leftSideWall() {
  rotateLeftIR(0);
  receiveIR();
  distanceFront = distance;
  rotateLeftIR(70);
  wait(10);
  receiveIR();
  distanceWall1 = distance;
  rotateLeftIR(80);
  receiveIR();
  distanceWall2 = distance;
  /** Calculate distanceWall **/
  distanceWall =  (distanceWall1 * distanceWall2 * sin(0.2618)) / sqrt(distanceWall1 * distanceWall1 + distanceWall2 * distanceWall2 - 2 * distanceWall1 * distanceWall2 * cos(0.2618));
  if (isnan(distanceWall) || distanceWall == 0 || isnan(distanceWall1) || isnan(distanceWall2)) distanceWall = 5;

  if (distanceWall < 0.8 && distanceWall > 0.6) {
    Serial.setTimeout(100);
    turnRightAuto(1);
    wait(50);
  }
  if (distanceWall >= 0.8 && distanceWall <= 1.1) {
    Serial.setTimeout(100);
    turnLeftAuto(1);
  }
  if (distanceWall <= 0.6) {
    Serial.setTimeout(100);
    turnRight();
    wait(50);
    Serial.setTimeout(100);
    PrintMessage("CMD_ACT_LAT_1_0.2");
    wait(100);
    Serial.setTimeout(100);
    turnLeftAuto(88);
    wait(50);
  }
  if (on == 1 || on == 2) {
    if (distanceWall >= 1.1 && distanceWall < 2.3) {
      Serial.setTimeout(100);
      turnLeft();
      wait(10);
      Serial.setTimeout(100);
      PrintMessage("CMD_ACT_LAT_1_0.3");
      wait(100);
      Serial.setTimeout(100);
      turnRightAuto(88);
      wait(50);
    }
    if (isnan(distanceFront)) {
      distanceFront = 5;
    }
    if (distanceFront > 1.5) {
      Serial.setTimeout(100);
      PrintMessage("CMD_ACT_LAT_1_0.9");
      wait(100);
    }
    if (distanceFront <= 1.5) {
      Serial.setTimeout(100);
      PrintMessage("CMD_ACT_LAT_1_0.3");
      wait(100);
    }
  }
  if (on == 3 || on == 4) {
    if (isnan(distanceFront)) {
      distanceFront = 5;
    }
    if (distanceFront > 1.5) {
      Serial.setTimeout(100);
      PrintMessage("CMD_ACT_LAT_1_0.7");
      wait(100);
    }
    if (distanceFront <= 1.5) {
      Serial.setTimeout(100);
      PrintMessage("CMD_ACT_LAT_1_0.3");
      wait(100);
    }
  }
}
