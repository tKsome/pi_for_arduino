/* *************************************************************
   Encoder definitions
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   ************************************************************ */
 
#ifdef USE_BASE
 
 
#include "motor_driver.h"//���������ͷ�ļ�
#include "commands.h"
/* Wrap the encoder reset function */
 
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
 
/* ����*/
 
int left_rotate = 0;
int right_rotate = 0;
 
 
//�对编码器进行初始化（利用中断来）
void initEncoders() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(19, INPUT);
  pinMode(18, INPUT);
  attachInterrupt(0, encoderLeftISR, CHANGE);
  attachInterrupt(1, encoderLeftISR,  CHANGE);
  attachInterrupt(4, encoderRightISR, CHANGE);
  attachInterrupt(5, encoderRightISR, CHANGE);
}
 
//�������޷��ֱ
void encoderLeftISR() {
  if (direction(LEFT) == BACKWARDS) {
 
    left_enc_pos--;
  }
 
  else {
    left_enc_pos++;
  }
}
 
void encoderRightISR() {
  if (direction(RIGHT) == BACKWARDS) {
    right_enc_pos--;
  } else {
    right_enc_pos++;
  }
}
 
long readEncoder(int i) {
  long encVal = 0L;
  if (i == LEFT)  {
    noInterrupts();
    encVal = left_enc_pos;
    interrupts();
  }
  else {
    noInterrupts();
    encVal = right_enc_pos;
    interrupts();
  }
  return encVal;
}
 
/* Wrap the encoder reset function */
//���ñ���������
void resetEncoder(int i) {
  if (i == LEFT) {
    left_enc_pos = 0L;
    return;
  } else {
    right_enc_pos = 0L;
    return;
  }
}
 
/* Wrap the encoder reset function */
//�����װ
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}
 
#endif
