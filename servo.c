#include "servo.h"
#include <string.h>

// 舵机全局数组
Servo_t Servos[3];

// 如果没有定义 HAL_TICK 函数，添加一个简单实现
#ifndef HAL_TICK
unsigned int HAL_TICK(void)
{
    // 返回系统时钟，实际项目中应替换为实际的时钟函数
    return 0;
}
#endif

/**
 * 初始化舵机系统
 * 配置舵机参数并将舵机移动到中间位置
 */
void Servo_Init(void)
{
    // 清空舵机数组
    memset(Servos, 0, sizeof(Servos));
    
    // 配置各舵机参数
    for (int i = 0; i < 3; i++) {
        Servos[i].channel = i + 1;            // 通道号为1-3
        Servos[i].current_pulse = SERVO_CENTER_PULSE;
        Servos[i].target_pulse = SERVO_CENTER_PULSE;
        Servos[i].speed = 50.0;               // 默认速度(us/更新)
        Servos[i].min_pulse = SERVO_MIN_PULSE;
        Servos[i].max_pulse = SERVO_MAX_PULSE;
        Servos[i].is_moving = 0;
    }
    
    // 初始化硬件
    Servo_HardwareInit();
    
    // 将所有舵机设置到中间位置
    for (int i = 0; i < 3; i++) {
        Servo_HardwareSetPulse(Servos[i].channel, Servos[i].current_pulse);
    }
}

/**
 * 设置舵机脉冲宽度
 * @param servo_id 舵机ID (1-3)
 * @param pulse_width 脉冲宽度 (微秒)
 */
void Servo_SetPulse(int servo_id, double pulse_width)
{
    // 确保servo_id在有效范围内(1-3)
    if (servo_id < 1 || servo_id > 3) {
        return;
    }
    
    // 获取舵机索引(0-2)
    int index = servo_id - 1;
    
    // 限制脉冲宽度在有效范围内
    if (pulse_width < Servos[index].min_pulse) {
        pulse_width = Servos[index].min_pulse;
    }
    else if (pulse_width > Servos[index].max_pulse) {
        pulse_width = Servos[index].max_pulse;
    }
    
    // 设置目标脉冲宽度
    Servos[index].target_pulse = pulse_width;
    Servos[index].is_moving = 1;
}

/**
 * 设置舵机移动速度
 * @param servo_id 舵机ID (1-3)
 * @param speed 移动速度 (微秒/更新)
 */
void Servo_SetSpeed(int servo_id, double speed)
{
    // 确保servo_id在有效范围内(1-3)
    if (servo_id < 1 || servo_id > 3) {
        return;
    }
    
    // 获取舵机索引(0-2)
    int index = servo_id - 1;
    
    // 确保速度为正值
    if (speed < 0) {
        speed = 0;
    }
    
    // 设置速度
    Servos[index].speed = speed;
}

/**
 * 检查舵机是否在移动
 * @param servo_id 舵机ID (1-3)
 * @return 1:正在移动 0:静止
 */
int Servo_IsMoving(int servo_id)
{
    // 确保servo_id在有效范围内(1-3)
    if (servo_id < 1 || servo_id > 3) {
        return 0;
    }
    
    // 获取舵机索引(0-2)
    int index = servo_id - 1;
    
    return Servos[index].is_moving;
}

/**
 * 停止舵机移动
 * @param servo_id 舵机ID (1-3)
 */
void Servo_Stop(int servo_id)
{
    // 确保servo_id在有效范围内(1-3)
    if (servo_id < 1 || servo_id > 3) {
        return;
    }
    
    // 获取舵机索引(0-2)
    int index = servo_id - 1;
    
    // 将当前位置设为目标位置，停止移动
    Servos[index].target_pulse = Servos[index].current_pulse;
    Servos[index].is_moving = 0;
}

/**
 * 舵机控制任务，应当在主循环中定期调用
 * 处理舵机平滑移动
 */
void Servo_Task(void)
{
    static unsigned int last_update = 0;
    unsigned int current_time = HAL_TICK();
    
    // 每20ms更新一次舵机位置
    if (current_time - last_update < 20) {
        return;
    }
    
    last_update = current_time;
    
    // 处理每个舵机
    for (int i = 0; i < 3; i++) {
        // 如果不需要移动，跳过
        if (!Servos[i].is_moving) {
            continue;
        }
        
        // 计算当前位置与目标位置之间的差异
        double diff = Servos[i].target_pulse - Servos[i].current_pulse;
        
        // 如果差异很小，直接到达目标位置
        if (fabs(diff) <= Servos[i].speed) {
            Servos[i].current_pulse = Servos[i].target_pulse;
            Servos[i].is_moving = 0;
        } else {
            // 否则，朝目标方向移动一步
            if (diff > 0) {
                Servos[i].current_pulse += Servos[i].speed;
            } else {
                Servos[i].current_pulse -= Servos[i].speed;
            }
        }
        
        // 应用到硬件
        Servo_HardwareSetPulse(Servos[i].channel, Servos[i].current_pulse);
    }
}

// 以下函数需要根据具体硬件平台实现

/**
 * 初始化舵机硬件
 * 配置PWM输出引脚和定时器
 */
void Servo_HardwareInit(void)
{
    // TODO: 根据实际硬件实现
    // 例如在STM32上初始化定时器PWM输出
}

/**
 * 设置硬件脉冲宽度
 * @param channel 舵机通道
 * @param pulse_width 脉冲宽度 (微秒)
 */
void Servo_HardwareSetPulse(int channel, double pulse_width)
{
    // TODO: 根据实际硬件实现
    // 例如在STM32上设置PWM占空比
    
    // 计算PWM周期
    // 对于50Hz的舵机PWM，周期为20ms = 20000us
    // 占空比 = pulse_width / 20000 * 100%
    
    // 执行实际的硬件脉冲设置
} 