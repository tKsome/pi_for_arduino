/***************************************************************
   Motor driver definitions
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
  由于使用了不同的电机驱动，这里需要改进
   
   *************************************************************/
 
#ifdef USE_BASE
 

#ifdef POLOLU_VNH5019
/* Include the Pololu library */
#include "DualVNH5019MotorShield.h"
 
/* Create the motor driver object */
DualVNH5019MotorShield drive;
 
/* Wrap the motor driver initialization */
void initMotorController() {
  drive.init();
}
 
/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) drive.setM1Speed(spd);
  else drive.setM2Speed(spd);
}
 
// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#elif defined POLOLU_MC33926
/* Include the Pololu library */
#include "DualMC33926MotorShield.h"
 
/* Create the motor driver object */
DualMC33926MotorShield drive;
 
/* Wrap the motor driver initialization */
void initMotorController() {
  drive.init();
}
 
/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) drive.setM1Speed(spd);
  else drive.setM2Speed(spd);
}
 
// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
//�����ﲻ�ÿ�
 
#elif defined L298_MOTOR_DRIVER
 
boolean directionLeft = false;
boolean directionRight = false;
//��������
boolean direction(int i) {
  if (i == LEFT) {
    return directionLeft;
  } else {
    return directionRight;
  }
}
void initMotorController() {
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
}

void setMotorSpeed(int i, int spd) {
  unsigned char reverse = 0;
 
  if (spd < 0)
  {
    spd = -spd;
    reverse = 1;
  }
  if (spd > 255)
    spd = 255;
  //��������������Ƴ����������޸�
  if (i == LEFT) {
    if (reverse == 0) {
      analogWrite(LEFT_MOTOR_FORWARD, spd);
      analogWrite(LEFT_MOTOR_BACKWARD, 0);
      directionLeft = FORWARDS;
    }
    else if (reverse == 1) {
      analogWrite(LEFT_MOTOR_BACKWARD, spd);
      analogWrite(LEFT_MOTOR_FORWARD, 0);
      directionLeft = BACKWARDS;
    }
  }
  else {
    if (reverse == 0) {
      analogWrite(RIGHT_MOTOR_FORWARD, spd);
      analogWrite(RIGHT_MOTOR_BACKWARD, 0);
      directionRight = FORWARDS;
    }
    else if (reverse == 1) {
      analogWrite(RIGHT_MOTOR_BACKWARD, spd);
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
      directionRight = BACKWARDS;
    }
  }//�
}
 
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}





#elif defined FUCKING_GARBAGE_MOTOR_DRIVER

// 这里自己来写新的电机驱动程序
boolean directionLeft = false;
boolean directionRight = false;
//��������
boolean direction(int i) {
  if (i == LEFT) {
    return directionLeft;
  } else {
    return directionRight;
  }
}
void initMotorController() {
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
}


void setMotorSpeed(int i, int spd) {
  unsigned char reverse = 0;
 
  if (spd < 0)
  {
    spd = -spd;
    reverse = 1;
  }
  if (spd > 255)
    spd = 255;
  //��������������Ƴ����������޸�
  if (i == LEFT) 
  {
    if (reverse == 0) {
      analogWrite(LEFT_MOTOR_PWM, spd);
      digitalWrite(LEFT_MOTOR_IN2, LOW);
      digitalWrite(LEFT_MOTOR_IN1, HIGH);
      directionLeft = FORWARDS;
    }
    else if (reverse == 1) {
      analogWrite(LEFT_MOTOR_PWM, spd);
      digitalWrite(LEFT_MOTOR_IN2, HIGH);
      digitalWrite(LEFT_MOTOR_IN1, LOW);
      directionLeft = BACKWARDS;
    }
  }
  else 
  {
    if (reverse == 0) {
      analogWrite(RIGHT_MOTOR_PWM, spd);
      digitalWrite(RIGHT_MOTOR_IN2, LOW);
      digitalWrite(RIGHT_MOTOR_IN1, HIGH);
      directionRight = FORWARDS;
    }
    else if (reverse == 1) {
      analogWrite(RIGHT_MOTOR_PWM, spd);
      digitalWrite(RIGHT_MOTOR_IN2, HIGH);
      digitalWrite(RIGHT_MOTOR_IN1, LOW);
      directionRight = BACKWARDS;
    }
  }//�
}
 
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}

#else
#error A motor driver must be selected!
#endif
 
#endif
