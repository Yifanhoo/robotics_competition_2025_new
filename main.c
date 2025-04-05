#include <stdio.h>
#include "Mission.h"
#include "Chassis.h"
#include "sensor.h"
#include "arm.h"
#include "servo.h"

int Read_Ultrasonic(void) { return 200; } // 模拟200mm距离

// 物体检测传感器模拟函数
int Object_Detected(void) { return 1; } // 假设总是能检测到物体

// 舵机硬件控制函数（模拟）
void Servo_HardwareInit(void) {
    printf("舵机硬件初始化\n");
}

void Servo_HardwareSetPulse(int channel, double pulse_width) {
    printf("设置舵机 %d 脉冲宽度: %.2f us\n", channel, pulse_width);
}

// HAL时钟模拟函数
static unsigned int system_time = 0;
unsigned int HAL_TICK(void) {
    return system_time;
}

// 演示：测试机械臂逆运动学功能
void test_arm_inverse_kinematics(void) {
    printf("\n===== 测试机械臂逆运动学 =====\n");
    
    // 测试几个位置点
    double test_positions[][2] = {
        {150, 150},  // 前方偏上
        {200, 0},    // 正前方
        {100, -100}, // 前方偏下
        {50, 200},   // 近处抬高
        {300, 50}    // 远处
    };
    
    for (int i = 0; i < 5; i++) {
        double x = test_positions[i][0];
        double y = test_positions[i][1];
        
        printf("目标位置 (%.1f, %.1f): ", x, y);
        
        ArmAngles_t angles = Arm_CalculateAngles(x, y);
        
        if (angles.reachable) {
            printf("可达, 角度: theta1=%.2f°, theta2=%.2f°\n", 
                   angles.theta1 * 180.0 / M_PI, 
                   angles.theta2 * 180.0 / M_PI);
        } else {
            printf("不可达, 错误代码: %d\n", Arm.error_code);
        }
    }
}

// 设置火种位置
void setup_fire_positions(void) {
    printf("\n===== 设置火种位置 =====\n");
    
    // 清除之前的火种位置
    Mission_ClearFirePositions();
    
    // 设置火种位置（示例坐标，需要根据实际情况调整）
    Mission_SetFirePosition(0, 200.0, 150.0);  // 第一个火种位置
    Mission_SetFirePosition(1, 300.0, 150.0);  // 第二个火种位置
    Mission_SetFirePosition(2, 400.0, 150.0);  // 第三个火种位置
    
    printf("火种位置设置完成\n");
}

// 演示：设置机械臂任务序列
void setup_arm_mission_sequence(void) {
    printf("\n===== 设置机械臂任务序列 =====\n");
    
    // 清除之前的任务
    Mission_Init();
    
    // 添加一系列任务
    printf("添加任务: 机械臂回到初始位置\n");
    Mission_Add(MISSION_ARM_HOME, 0);
    
    // 获取下一个可用的火种位置
    double fire_x, fire_y;
    int fire_index = Mission_GetNextAvailableFirePosition(&fire_x, &fire_y);
    
    if (fire_index >= 0) {
        printf("添加任务: 前往火种位置 (%.1f, %.1f)\n", fire_x, fire_y);
        Mission_AddWithPosition(MISSION_GOTO_FIRE, fire_x, fire_y);
        
        printf("添加任务: 放置火种\n");
        Mission_Add(MISSION_PLACE_FIRE, fire_index);
    }
    
    printf("添加任务: 机械臂移动到指定位置\n");
    Mission_AddWithPosition(MISSION_ARM_MOVE_TO, 150.0, 100.0);
    
    printf("添加任务: 释放物体\n");
    Mission_Add(MISSION_ARM_RELEASE, 0);
    
    printf("添加任务: 机械臂回到休息位置\n");
    Mission_Add(MISSION_ARM_REST, 0);
    
    // 开始执行任务
    Mission_Start();
}

// 主函数
int main() {
    printf("===== 机器人控制系统启动 =====\n");
    
    // 初始化系统
    Mission_Init();
    
    // 设置火种位置
    setup_fire_positions();
    
    // 测试机械臂逆运动学
    test_arm_inverse_kinematics();
    
    // 设置任务序列
    setup_arm_mission_sequence();
    
    // 主循环
    printf("\n===== 开始任务执行 =====\n");
    for (int i = 0; i < 100; i++) {
        // 更新系统时间（模拟）
        system_time += 50; // 增加50ms
        
        // 执行任务
        Mission_Task();
        
        // 打印当前状态
        if (i % 10 == 0) {
            printf("时间: %dms, 任务索引: %d/%d, 机械臂状态: %d\n", 
                  system_time, current_mission_index, mission_queue_size, Arm.state);
        }
        
        // 检查任务是否完成
        if (Mission_IsCompleted()) {
            printf("所有任务完成!\n");
            break;
        }
    }
    
    printf("===== 机器人控制系统关闭 =====\n");
    return 0;
} 