/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void initEncoders();//初始化编码器
void encoderRightISR1();//编码器中断服务
void encoderLeftISR1();//编码器中断服务
void encoderRightISR();//编码器中断服务
void encoderLeftISR();//编码器中断服务
