/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/
//

 
#ifdef FUCKING_GARBAGE_MOTOR_DRIVER 

  #define RIGHT_MOTOR_PWM 10
  #define RIGHT_MOTOR_IN2 9 
  #define RIGHT_MOTOR_IN1 8
  
  #define LEFT_MOTOR_PWM 7
  #define LEFT_MOTOR_IN2 5
  #define LEFT_MOTOR_IN1 6

  
#elif L298_MOTOR_DRIVER

  #define RIGHT_MOTOR_BACKWARD 4
  #define LEFT_MOTOR_BACKWARD  8
  #define RIGHT_MOTOR_FORWARD  5
  #define LEFT_MOTOR_FORWARD   9

  
#endif
void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
boolean direction(int i);
