/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void initEncoders();//��ʼ��������
void encoderRightISR1();//�������жϷ���
void encoderLeftISR1();//�������жϷ���
void encoderRightISR();//�������жϷ���
void encoderLeftISR();//�������жϷ���
