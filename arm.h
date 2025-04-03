#ifndef ARM_H
#define ARM_H

#include <math.h>

// 如果没有定义 M_PI 常量
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 错误代码定义
#define ARM_ERROR_NONE              0  // 无错误
#define ARM_ERROR_OUT_OF_RANGE      1  // 目标点超出范围
#define ARM_ERROR_TOO_CLOSE         2  // 目标点太靠近基座
#define ARM_ERROR_AT_ORIGIN         3  // 目标点在原点
#define ARM_ERROR_JOINT1_LIMIT      4  // 第一关节角度超出范围
#define ARM_ERROR_JOINT2_LIMIT      5  // 第二关节角度超出范围
#define ARM_ERROR_ANGLE_LIMIT       6  // 角度超出范围
#define ARM_ERROR_MOVE_TIMEOUT      7  // 移动超时
#define ARM_ERROR_BASE_ANGLE_LIMIT  8  // 底座角度超出范围
#define ARM_ERROR_OBJECT_LOST       9  // 物体丢失
#define ARM_ERROR_GRAB_FAILED      10  // 抓取失败

// 机械臂位置状态
typedef enum {
    ARM_STATE_IDLE = 0,       // 空闲状态
    ARM_STATE_MOVING,         // 运动中
    ARM_STATE_GRABBING,       // 抓取中
    ARM_STATE_HOLDING,        // 持有物体
    ARM_STATE_RELEASING,      // 释放物体
    ARM_STATE_ERROR           // 错误状态
} ArmState_t;

// 抓爪状态
typedef enum {
    CLAW_STATE_OPEN = 0,      // 抓爪打开
    CLAW_STATE_CLOSING,       // 抓爪正在闭合
    CLAW_STATE_CLOSED,        // 抓爪闭合
    CLAW_STATE_OPENING        // 抓爪正在打开
} ClawState_t;

// 三连杆角度
typedef struct {
    double theta0;            // 底座水平旋转角度（弧度）
    double theta1;            // 关节1角度（弧度）
    double theta2;            // 关节2角度（弧度）
    int reachable;            // 是否可达（1:可达，0:不可达）
} ArmAngles_t;

// 目标位置
typedef struct {
    double x;                 // X坐标（mm）
    double y;                 // Y坐标（mm）
    double z;                 // Z坐标（mm）
} ArmPosition_t;

// 机械臂配置
typedef struct {
    double L1;                // 第一关节长度（mm）
    double L2;                // 第二关节长度（mm）
    
    double servo0_min_pulse;  // 底座舵机最小脉冲宽度
    double servo0_max_pulse;  // 底座舵机最大脉冲宽度
    double servo0_min_angle;  // 底座舵机最小角度（弧度）
    double servo0_max_angle;  // 底座舵机最大角度（弧度）
    
    double servo1_min_pulse;  // 舵机1最小脉冲宽度
    double servo1_max_pulse;  // 舵机1最大脉冲宽度
    double servo1_min_angle;  // 舵机1最小角度（弧度）
    double servo1_max_angle;  // 舵机1最大角度（弧度）
    
    double servo2_min_pulse;  // 舵机2最小脉冲宽度
    double servo2_max_pulse;  // 舵机2最大脉冲宽度
    double servo2_min_angle;  // 舵机2最小角度（弧度）
    double servo2_max_angle;  // 舵机2最大角度（弧度）
    
    double claw_min_pulse;    // 抓爪最小脉冲宽度
    double claw_max_pulse;    // 抓爪最大脉冲宽度
    double claw_open_pulse;   // 抓爪打开脉冲宽度
    double claw_close_pulse;  // 抓爪闭合脉冲宽度
    
    double move_speed;        // 移动速度（弧度/秒）
    double grab_speed;        // 抓取速度（脉冲/秒）
    double rotate_speed;      // 底座旋转速度（弧度/秒）
} ArmConfig_t;

// 机械臂实例
typedef struct {
    ArmState_t state;          // 当前状态
    ClawState_t claw_state;    // 抓爪状态
    
    ArmPosition_t current_pos; // 当前位置
    ArmPosition_t target_pos;  // 目标位置
    
    ArmAngles_t current_angles; // 当前角度
    ArmAngles_t target_angles;  // 目标角度
    
    ArmConfig_t config;        // 配置参数
    
    unsigned int last_update;  // 上次更新时间
    unsigned int action_start; // 动作开始时间
    unsigned int action_timeout; // 动作超时时间
    
    int has_object;           // 是否抓取到物体
    int error_code;           // 错误代码
} Arm_t;

// 全局实例声明
extern Arm_t Arm;

// 基础函数声明
void Arm_Init(void);                          // 初始化机械臂
void Arm_Task(void);                          // 机械臂控制任务（应当在主循环中定期调用）
void Arm_SetServoAngle(int servo_id, double angle); // 设置舵机角度

// 运动控制函数
int Arm_MoveTo(double x, double y, double z);       // 移动到指定位置（3D）
int Arm_MoveToXY(double x, double y);               // 移动到指定位置（2D平面）
int Arm_Rotate(double angle);                       // 旋转底座到指定角度
int Arm_MoveToAngles(double theta0, double theta1, double theta2); // 移动到指定角度
void Arm_Stop(void);                          // 停止移动

// 抓取控制函数
void Arm_OpenClaw(void);                      // 打开抓爪
void Arm_CloseClaw(void);                     // 闭合抓爪
int Arm_Grab(void);                           // 抓取当前位置的物体
int Arm_Release(void);                        // 释放物体

// 状态查询函数
int Arm_IsMoving(void);                       // 检查是否在移动
int Arm_IsHolding(void);                      // 检查是否持有物体
int Arm_HasError(void);                       // 检查是否有错误
int Arm_IsAtPosition(double x, double y, double z, double tolerance); // 检查是否到达指定位置

// 逆运动学计算
ArmAngles_t Arm_CalculateAngles(double x, double y, double z); // 计算到达指定位置的关节角度
ArmAngles_t Arm_Calculate2DAngles(double x, double y);  // 计算2D平面位置的关节角度

// 预设位置函数
void Arm_MoveToHome(void);                    // 移动到初始位置
void Arm_MoveToRest(void);                    // 移动到休息位置
void Arm_MoveToGrabPosition(void);            // 移动到抓取位置
void Arm_MoveToPutPosition(void);             // 移动到放置位置
void Arm_RotateToAngle(double angle);         // 旋转底座到指定角度（弧度）
void Arm_RotateToDegrees(double degrees);     // 旋转底座到指定角度（度）

// 新增函数声明
const char* Arm_GetErrorString(void);

#endif /* ARM_H */ 