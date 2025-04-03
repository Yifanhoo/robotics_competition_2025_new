#ifndef SERVO_H
#define SERVO_H

// 舵机通道定义
#define SERVO_CHANNEL_1      1  // 第一关节舵机
#define SERVO_CHANNEL_2      2  // 第二关节舵机
#define SERVO_CHANNEL_3      3  // 抓爪舵机

// 舵机脉冲范围（微秒）
#define SERVO_MIN_PULSE      500   // 最小脉冲宽度
#define SERVO_MAX_PULSE      2500  // 最大脉冲宽度
#define SERVO_CENTER_PULSE   1500  // 中间位置脉冲宽度

// 舵机频率（Hz）
#define SERVO_FREQUENCY      50    // 标准舵机PWM频率

// 舵机控制结构体
typedef struct {
    int channel;              // 舵机通道
    double current_pulse;     // 当前脉冲宽度（微秒）
    double target_pulse;      // 目标脉冲宽度（微秒）
    double speed;             // 调整速度（微秒/次更新）
    double min_pulse;         // 最小脉冲限制
    double max_pulse;         // 最大脉冲限制
    int is_moving;            // 是否正在移动
} Servo_t;

// 全局舵机数组声明
extern Servo_t Servos[3];

// 基本函数声明
void Servo_Init(void);                         // 初始化舵机系统
void Servo_Task(void);                         // 舵机控制任务（应在主循环中定期调用）
void Servo_SetPulse(int servo_id, double pulse_width); // 设置舵机脉冲宽度
void Servo_SetSpeed(int servo_id, double speed);       // 设置舵机移动速度
int Servo_IsMoving(int servo_id);                      // 检查舵机是否在移动
void Servo_Stop(int servo_id);                         // 停止舵机移动

// 实际硬件控制函数（需要根据具体硬件平台实现）
void Servo_HardwareInit(void);                         // 初始化舵机硬件
void Servo_HardwareSetPulse(int channel, double pulse_width); // 设置硬件脉冲

#endif /* SERVO_H */ 