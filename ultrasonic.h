#ifndef ULTRASONIC_H
#define ULTRASONIC_H

// 引脚定义（需要根据实际硬件连接修改）
#define TRIG_PIN    GPIO_PIN_2  // 触发引脚
#define TRIG_PORT   GPIOA      // 触发引脚端口
#define ECHO_PIN    GPIO_PIN_3  // 回响引脚
#define ECHO_PORT   GPIOA      // 回响引脚端口

// 超声波相关常量
#define SOUND_SPEED     340     // 声速（米/秒）
#define US_TO_MM        0.17f   // 微秒到毫米的转换系数 (340m/s / 2 = 0.17mm/us)
#define MAX_DISTANCE    4000    // 最大测量距离（毫米）
#define MIN_DISTANCE    20      // 最小测量距离（毫米）
#define TIMEOUT_US      25000   // 超时时间（微秒）

// 函数声明
void Ultrasonic_Init(void);              // 初始化超声波模块
int Read_Ultrasonic(void);               // 读取距离（毫米）
void Ultrasonic_TriggerMeasurement(void);// 触发测量
uint32_t Ultrasonic_WaitEcho(void);      // 等待并测量回响时间

#endif /* ULTRASONIC_H */ 