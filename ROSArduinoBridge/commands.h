/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/
 
#ifndef COMMANDS_H
#define COMMANDS_H
 
#define ANALOG_READ    'a' // 模拟读取
#define GET_BAUDRATE   'b' // 获取波特率
#define PIN_MODE       'c' // 接口模式
#define DIGITAL_READ   'd' // 数字读取
#define READ_ENCODERS  'e' // 读取编码器数据 输出为：左数据，右数据
#define MOTOR_SPEEDS   'm' // 设置马达速度，脉冲计数 如： m 20 20 会以一个很低的速度转动
//#define PING           'p' // 无用
#define RESET_ENCODERS 'r' // 重设编码器读数
#define SERVO_WRITE    's' // 舵机相关，我这里没有用到
#define SERVO_READ     't' // 舵机相关，我这里没有用到
#define UPDATE_PID     'u' // 更新pid参数
#define DIGITAL_WRITE  'w' // 数字端口设置
#define ANALOG_WRITE   'x' // 模拟端口设置
#define READ_PIDOUT  'f'   // 读取pid计算出的速度（脉冲数计数）
#define READ_PIDIN  'i'    // 读取进入pid的速度
 
#define LEFT            0
#define RIGHT           1
 
#define FORWARDS true   //FORWARDS前进代表bool真值true
#define BACKWARDS false //BACKWARDS后退代表bool假值false
 
#endif
