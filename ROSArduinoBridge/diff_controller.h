/* Functions and type-defs for PID control.
 Taken mostly from Mike Ferguson's ArbotiX code which lives at:
 
 http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
 */

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
   */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  int kk;   //这是用来测试串口直接输出的速度的

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
   * to allow tuning changes,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   */
  //int Ierror;
  int ITerm;                    //integrated term

    long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int lKp = 500;
int lKd = 0;
int lKi = 8;
int lKo = 1000;

int rKp = 500;
int rKd = 0;
int rKi = 8;
int rKo = 1000;

float lKp2 ;
float lKd2 ;
float lKi2 ;
float lKo2 ;

float rKp2 ;
float rKd2 ;
float rKi2 ;
float rKo2 ;


float iiii;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
 * when turning PID on to start moving
 * In particular, assign both Encoder and PrevEnc the current encoder value
 * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
 * Note that the assumption here is that PID is only turned on
 * when going from stop to moving, that's why we can init everything on zero.
 */
void resetPID(){
  leftPID.TargetTicksPerFrame = 0.0;
  leftPID.Encoder = readEncoder(LEFT);
  leftPID.PrevEnc = leftPID.Encoder;
  leftPID.output = 0;
  leftPID.PrevInput = 0;
  leftPID.ITerm = 0;

  rightPID.TargetTicksPerFrame = 0.0;
  rightPID.Encoder = readEncoder(RIGHT);
  rightPID.PrevEnc = rightPID.Encoder;
  rightPID.output = 0;
  rightPID.PrevInput = 0;
  rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPIDL(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  int kk=p->TargetTicksPerFrame;  //kk表示从串口获取的速度控制信息 相当于setpoint，但是否需要加系数
  lKp2 = lKp*0.001;
  lKd2 = lKd*0.001;
  lKi2 = lKi*0.001;
  lKo2 = lKo*0.001;
  input =p->Encoder - p->PrevEnc;     // 轮速脉冲，当前车轮每转一圈约1350个计数

  // input明显会出问题，引发问题的意义目前还不明确，暂时使用程序进行限制


  Perror = p->TargetTicksPerFrame - input;    // pid中的error，串口的速度控制信息减去轮速脉冲（控制速度-测量速度）
  // Perror=0;//暂时中止PID功能


  //22这里的pid公式比较奇怪，(Kp * error - Kd * err +  term) / Ko  ，不清楚这个公式是哪种pid的变种

  output = (lKp2 * Perror - lKd2 * (input - p->PrevInput) + p->ITerm) / lKo2;  // 22(input - p->PrevInput) 微分项，当次轮速和上次轮速之差           

  p->PrevEnc = p->Encoder;   //编码器读数赋值给上一次，用来求编码器速度


 
  output += p->output;  

  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates（饱和）
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else

      p->ITerm += lKi2 * Perror;  //   积分项，对比例进行累加积分，并在积分过程中直接进行积分控制（即直接乘Ki）

  p->output = output;
  p->PrevInput = input;

  p->kk = kk;  //22测试用

  // 速度检测

  //   Serial.print("LEFT: input   ");
  //  Serial.print(kk);
  //  Serial.print("   ");
  //  Serial.print("speed  ");
  //  Serial.print(input);
  //   Serial.print("   ");
  //   Serial.print("output_pwm  ");
  //   Serial.println(output);


}
void doPIDR(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  int kk=p->TargetTicksPerFrame;  //kk表示从串口获取的速度控制信息 相当于setpoint，但是否需要加系数
  rKp2 = rKp*0.001;
  rKd2 = rKd*0.001;
  rKi2 = rKi*0.001;
  rKo2 = rKo*0.001;

  input =p->Encoder - p->PrevEnc;     // 轮速脉冲，当前车轮每转一圈约1350个计数

  Perror = p->TargetTicksPerFrame - input;    // pid中的error，串口的速度控制信息减去轮速脉冲（控制速度-测量速度）
  // Perror=0;//暂时中止PID功能


  //22这里的pid公式比较奇怪，(Kp * error - Kd * err +  term) / Ko  ，不清楚这个公式是哪种pid的变种

  output = (rKp2 * Perror - rKd2 * (input - p->PrevInput) + p->ITerm) / rKo2;  // 22(input - p->PrevInput) 微分项，当次轮速和上次轮速之差           
  iiii=output;
  p->PrevEnc = p->Encoder;   //编码器读数赋值给上一次，用来求编码器速度


  // 最新测试表明，下面段程序意义非常重要，之后需要查询对应意义
  output += p->output;  //22 对输出进行积分是个什么操作？？这段程序是否有效？？


  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates（饱和）
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else

      p->ITerm += rKi2 * Perror;  // 22  积分项，对比例进行累加积分，并在积分过程中直接进行积分控制（即直接乘Ki）

  p->output = output;
  p->PrevInput = input;

  p->kk = kk;  //22测试用

 // Serial.print(" LRIGHT:    output   ");
 // Serial.print(rKp2);
 // Serial.print("   ");
 // Serial.print("speed  ");
 // Serial.print(input);
 // Serial.print("   ");
 // Serial.print("output_pwm  ");
 // Serial.println(output);


}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);

  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
     * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
     * PrevInput is considered a good proxy to detect
     * whether reset has already happened
     */
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPIDR(&rightPID);
  doPIDL(&leftPID);

  /* Set the motor speeds accordingly */
  // setMotorSpeeds(leftPID.output, rightPID.output);

  //  test   test    test   test   test   test   test   test   test   test  //

  setMotorSpeeds(leftPID.output, rightPID.output);


  //  test   test    test   test   test   test   test   test   test   test  //  


}
long readPidIn(int i) {
  long pidin=0;
  if (i == LEFT){
    pidin = leftPID.PrevInput;
  }
  else {
    pidin = rightPID.PrevInput;
  }
  return pidin;
}

long readPidOut(int i) {
  long pidout=0;
  if (i == LEFT){
    pidout = leftPID.output;
  }
  else {
    pidout = rightPID.output;
  }
  return pidout;
}
