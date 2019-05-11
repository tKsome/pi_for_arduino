//这是在官方包基础上修改来的
#include <string.h>
 
#define USE_BASE      // Enable the base controller code 是否使用base controller
//#undef USE_BASE     // Disable the base controller code
 
/* Define the motor controller and encoder library you are using */
//定义电机控制方式，使用ifdef 的方式，和正常的if 语句类似
#ifdef USE_BASE
/* The Pololu VNH5019 dual motor driver shield */
//#define POLOLU_VNH5019
 
/* The Pololu MC33926 dual motor driver shield */
//#define POLOLU_MC33926
 
/* The RoboGaia encoder shield */
//#define ROBOGAIA
 
/* Encoders directly attached to Arduino board */
//#define ARDUINO_ENC_COUNTER

#define FUCKING_GARBAGE_MOTOR_DRIVER
 
/* L298 Motor driver*/
//#define L298_MOTOR_DRIVER　//这里只是借用名称，之后需要自己更改驱动
#endif
 
//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos //不适用SERVOS
 
/* Serial port baud rate */
#define BAUDRATE     57600 //波特率
 
/* Maximum PWM signal */
#define MAX_PWM        255 //最大pwm
#define MAX_INPUT        255 //最大速度限制，防止倒转

 
#if defined(ARDUINO) && ARDUINO >= 100 //不知到用处
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
 
/* Include definition of serial commands */
#include "commands.h"
 
/* Sensor functions */
//#include "sensors.h"　//不使用
 
/* Include servo support if required */
#ifdef USE_SERVOS
#include <Servo.h>
#include "servos.h"
#endif
 
#ifdef USE_BASE
/* Motor driver function definitions */
#include "motor_driver.h"//加载电机控制头文件
 
/* Encoder driver function definitions */
#include "encoder_driver.h"//加载编码器控制头文件
 
/* PID parameters and functions */
#include "diff_controller.h"//加载PID控制
 
/* Run the PID loop at 30 times per second */
#define PID_RATE 30     // Hz　pid控制频率
 
/* Convert the rate into an interval */
const int PID_INTERVAL=1000/PID_RATE;


/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;
 
/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
#define AUTO_STOP_INTERVAL 2000  // 用来控制每次串口命令的作用时间（对电机），每次电机运行这个时间后停止，等待下次命令
long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif
 
/* Variable initialization */
 
// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;
 
// Variable to hold an input character
char chr;
 
// Variable to hold the current single-character command
char cmd;
 
// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];
 
// The arguments converted to integers
long arg1;
long arg2;
 
/* C下面出现警告，原文为： cmd = NULL; */
void resetCommand() {
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}
 
//运行命令，命令在commands.h中被定义
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4]; //由于左右电机使用不同的pid参数，一面要4个
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
 
  switch (cmd) {
    case READ_PIDIN: //与commands.h中的匹配 这里为添加的，原代码中没有
      Serial.print( readPidIn(LEFT));
      Serial.print(" ");
      Serial.println( readPidIn(RIGHT));
      break;
    case READ_PIDOUT://为了方便之后的PID调整
      Serial.print( readPidOut(LEFT));
      Serial.print(" ");
      Serial.println( readPidOut(RIGHT));
      break;
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
      /*case PING:
        Serial.println(Ping(arg1));
        break;*/
#ifdef USE_SERVOS
    case SERVO_WRITE:
      servos[arg1].setTargetPosition(arg2);
      Serial.println("OK");
      break;
    case SERVO_READ:
      Serial.println(servos[arg1].getServo().read());
      break;
#endif
 
#ifdef USE_BASE
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();   // 初始化pid
      Serial.println("RESET_ENCODERS OK");
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        resetPID();  
        moving = 0;
      }
      else moving = 1;
      leftPID.TargetTicksPerFrame = arg1;   //从串口读到的电机速度赋值给这个变量，并传递到doPID中
      rightPID.TargetTicksPerFrame = arg2;
      Serial.println("OK");
 //     setMotorSpeeds(arg1, arg2);
//  test   test    test   test   test   test   test   test   test   test  //




 //  test   test    test   test   test   test   test   test   test   test  //    
      break;
    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != '\0') {
        pid_args[i] = atoi(str);
        i++;
      }
      lKp = pid_args[0];
      lKd = pid_args[1];
      lKi = pid_args[2];
      lKo = pid_args[3];
 
      rKp = pid_args[0];
      rKd = pid_args[1];
      rKi = pid_args[2];
      rKo = pid_args[3];
 
      Serial.println("OK");
      break;
#endif
    default:
      Serial.println("Invalid Command");
      break;
  }
}
 
/* Setup function--runs once at startup. */
//记得初始化那些函数
void setup() {
  Serial.begin(BAUDRATE);
 
  // Initialize the motor controller if used */
#ifdef USE_BASE
#ifdef ARDUINO_ENC_COUNTER
  //set as inputs
  DDRD &= ~(1 << LEFT_ENC_PIN_A);
  DDRD &= ~(1 << LEFT_ENC_PIN_B);
  DDRC &= ~(1 << RIGHT_ENC_PIN_A);
  DDRC &= ~(1 << RIGHT_ENC_PIN_B);
 
  //enable pull up resistors
  PORTD |= (1 << LEFT_ENC_PIN_A);
  PORTD |= (1 << LEFT_ENC_PIN_B);
  PORTC |= (1 << RIGHT_ENC_PIN_A);
  PORTC |= (1 << RIGHT_ENC_PIN_B);
 
  // tell pin change mask to listen to left encoder pins
  PCMSK2 |= (1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B);
  // tell pin change mask to listen to right encoder pins
  PCMSK1 |= (1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B);
 
  // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
  PCICR |= (1 << PCIE1) | (1 << PCIE2);
#endif
  initEncoders();
  initMotorController();
  resetPID();
#endif
 
  /* Attach servos if used */
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].initServo(
      servoPins[i],
      stepDelay[i],
      servoInitPosition[i]);
  }
#endif
}
 
/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
 
    // Read the next character
    chr = Serial.read();
 
    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
 
  // If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
 
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    ;
    setMotorSpeeds(0, 0);   // 用来控制每次串口命令的作用时间（对电机），每次电机运行这个时间后停止，等待下次命令
    moving = 0;
  }
#endif
 
  // Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}
