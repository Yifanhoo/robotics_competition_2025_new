#include "arm.h"
#include "sensor.h"

// 机械臂抓取距离限制（单位：毫米）
#define DISTANCE_GRAB_MIN  100.0  // 最小抓取距离
#define DISTANCE_GRAB_MAX  300.0  // 最大抓取距离

// 机械臂全局实例
Arm_t Arm;

// 外部硬件接口函数声明
// 这些函数需要在其他文件中实现，用于舵机控制
extern void Servo_SetPulse(int servo_id, double pulse_width);
extern int Object_Detected(void);  // 检测物体是否存在（用于确认抓取）

// 如果没有定义 HAL_TICK 函数，添加一个简单实现
#ifndef HAL_TICK
unsigned int HAL_TICK(void)
{
    // 返回系统时钟，实际项目中应替换为实际的时钟函数
    // 例如，在 STM32 上可以使用 HAL_GetTick()
    return 0;
}
#endif

// 默认配置参数
#define DEFAULT_ARM_L1          150.0   // 第一关节长度 (mm)
#define DEFAULT_ARM_L2          150.0   // 第二关节长度 (mm)
#define DEFAULT_SERVO0_MIN_PULSE 500.0  // 底座舵机最小脉冲 (us)
#define DEFAULT_SERVO0_MAX_PULSE 2500.0 // 底座舵机最大脉冲 (us)
#define DEFAULT_SERVO0_MIN_ANGLE -1.57  // 底座舵机最小角度 (-90度，弧度)
#define DEFAULT_SERVO0_MAX_ANGLE 1.57   // 底座舵机最大角度 (90度，弧度)
#define DEFAULT_SERVO1_MIN_PULSE 500.0  // 舵机1最小脉冲 (us)
#define DEFAULT_SERVO1_MAX_PULSE 2500.0 // 舵机1最大脉冲 (us)
#define DEFAULT_SERVO1_MIN_ANGLE -1.57  // 舵机1最小角度 (-90度，弧度)
#define DEFAULT_SERVO1_MAX_ANGLE 1.57   // 舵机1最大角度 (90度，弧度)
#define DEFAULT_SERVO2_MIN_PULSE 500.0  // 舵机2最小脉冲 (us)
#define DEFAULT_SERVO2_MAX_PULSE 2500.0 // 舵机2最大脉冲 (us)
#define DEFAULT_SERVO2_MIN_ANGLE 0.0    // 舵机2最小角度 (0度，弧度)
#define DEFAULT_SERVO2_MAX_ANGLE 2.09   // 舵机2最大角度 (120度，弧度)
#define DEFAULT_CLAW_MIN_PULSE   500.0  // 抓爪最小脉冲 (us)
#define DEFAULT_CLAW_MAX_PULSE   2500.0 // 抓爪最大脉冲 (us)
#define DEFAULT_CLAW_OPEN_PULSE  1000.0 // 抓爪打开脉冲 (us)
#define DEFAULT_CLAW_CLOSE_PULSE 2000.0 // 抓爪闭合脉冲 (us)
#define DEFAULT_MOVE_SPEED       0.5    // 默认移动速度 (弧度/秒)
#define DEFAULT_GRAB_SPEED       100.0  // 默认抓取速度 (脉冲/秒)
#define DEFAULT_ROTATE_SPEED     0.5    // 默认旋转速度 (弧度/秒)

// 动作超时时间
#define ARM_MOVE_TIMEOUT    5000  // 移动超时 (ms)
#define ARM_GRAB_TIMEOUT    2000  // 抓取超时 (ms)
#define ARM_RELEASE_TIMEOUT 2000  // 释放超时 (ms)

// 预设位置
#define ARM_HOME_X          200.0  // 初始位置X (mm)
#define ARM_HOME_Y          0.0    // 初始位置Y (mm)
#define ARM_REST_X          100.0  // 休息位置X (mm)
#define ARM_REST_Y          200.0  // 休息位置Y (mm)
#define ARM_GRAB_X          200.0  // 抓取位置X (mm)
#define ARM_GRAB_Y          50.0   // 抓取位置Y (mm)
#define ARM_PUT_X           100.0  // 放置位置X (mm)
#define ARM_PUT_Y          -100.0  // 放置位置Y (mm)

// 位置容差 (mm)
#define ARM_POSITION_TOLERANCE 5.0

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

/**
 * 初始化机械臂
 * 设置默认参数并将机械臂移动到初始位置
 */
void Arm_Init(void)
{
    // 初始化机械臂状态
    Arm.state = ARM_STATE_IDLE;
    Arm.claw_state = CLAW_STATE_OPEN;
    Arm.has_object = 0;
    Arm.error_code = 0;
    
    // 设置默认配置
    Arm.config.L1 = DEFAULT_ARM_L1;
    Arm.config.L2 = DEFAULT_ARM_L2;
    
    // 底座舵机配置
    Arm.config.servo0_min_pulse = DEFAULT_SERVO0_MIN_PULSE;
    Arm.config.servo0_max_pulse = DEFAULT_SERVO0_MAX_PULSE;
    Arm.config.servo0_min_angle = DEFAULT_SERVO0_MIN_ANGLE;
    Arm.config.servo0_max_angle = DEFAULT_SERVO0_MAX_ANGLE;
    Arm.config.rotate_speed = DEFAULT_ROTATE_SPEED;
    
    // 第一关节舵机配置
    Arm.config.servo1_min_pulse = DEFAULT_SERVO1_MIN_PULSE;
    Arm.config.servo1_max_pulse = DEFAULT_SERVO1_MAX_PULSE;
    Arm.config.servo1_min_angle = DEFAULT_SERVO1_MIN_ANGLE;
    Arm.config.servo1_max_angle = DEFAULT_SERVO1_MAX_ANGLE;
    
    // 第二关节舵机配置
    Arm.config.servo2_min_pulse = DEFAULT_SERVO2_MIN_PULSE;
    Arm.config.servo2_max_pulse = DEFAULT_SERVO2_MAX_PULSE;
    Arm.config.servo2_min_angle = DEFAULT_SERVO2_MIN_ANGLE;
    Arm.config.servo2_max_angle = DEFAULT_SERVO2_MAX_ANGLE;
    
    // 抓爪舵机配置
    Arm.config.claw_min_pulse = DEFAULT_CLAW_MIN_PULSE;
    Arm.config.claw_max_pulse = DEFAULT_CLAW_MAX_PULSE;
    Arm.config.claw_open_pulse = DEFAULT_CLAW_OPEN_PULSE;
    Arm.config.claw_close_pulse = DEFAULT_CLAW_CLOSE_PULSE;
    
    Arm.config.move_speed = DEFAULT_MOVE_SPEED;
    Arm.config.grab_speed = DEFAULT_GRAB_SPEED;
    
    // 初始位置为原点附近
    Arm.current_pos.x = 0.0;
    Arm.current_pos.y = 0.0;
    
    // 初始化所有角度为0
    Arm.current_angles.theta0 = 0.0;
    Arm.current_angles.theta1 = 0.0;
    Arm.current_angles.theta2 = 0.0;
    
    // 记录当前时间
    Arm.last_update = HAL_TICK();
    
    // 打开抓爪
    Arm_OpenClaw();
    
    // 移动到初始位置
    Arm_MoveToHome();
}

/**
 * 将弧度角度转换为舵机脉冲宽度
 * @param servo_id 舵机ID (0: 底座, 1: 第一关节, 2: 第二关节, 3: 抓爪)
 * @param angle 角度 (弧度)
 * @return 脉冲宽度 (微秒)
 */
static double Arm_AngleToPulse(int servo_id, double angle)
{
    double pulse = 0.0;
    
    switch(servo_id) {
        case 0: // 底座舵机
            if (angle < Arm.config.servo0_min_angle) 
                angle = Arm.config.servo0_min_angle;
            if (angle > Arm.config.servo0_max_angle) 
                angle = Arm.config.servo0_max_angle;
                
            pulse = Arm.config.servo0_min_pulse + 
                    (angle - Arm.config.servo0_min_angle) * 
                    (Arm.config.servo0_max_pulse - Arm.config.servo0_min_pulse) / 
                    (Arm.config.servo0_max_angle - Arm.config.servo0_min_angle);
            break;
            
        case 1: // 第一关节舵机
            if (angle < Arm.config.servo1_min_angle) 
                angle = Arm.config.servo1_min_angle;
            if (angle > Arm.config.servo1_max_angle) 
                angle = Arm.config.servo1_max_angle;
                
            pulse = Arm.config.servo1_min_pulse + 
                    (angle - Arm.config.servo1_min_angle) * 
                    (Arm.config.servo1_max_pulse - Arm.config.servo1_min_pulse) / 
                    (Arm.config.servo1_max_angle - Arm.config.servo1_min_angle);
            break;
            
        case 2: // 第二关节舵机
            if (angle < Arm.config.servo2_min_angle) 
                angle = Arm.config.servo2_min_angle;
            if (angle > Arm.config.servo2_max_angle) 
                angle = Arm.config.servo2_max_angle;
                
            pulse = Arm.config.servo2_min_pulse + 
                    (angle - Arm.config.servo2_min_angle) * 
                    (Arm.config.servo2_max_pulse - Arm.config.servo2_min_pulse) / 
                    (Arm.config.servo2_max_angle - Arm.config.servo2_min_angle);
            break;
            
        case 3: // 抓爪舵机，只有开闭两个状态
            if (Arm.claw_state == CLAW_STATE_OPEN || Arm.claw_state == CLAW_STATE_OPENING)
                pulse = Arm.config.claw_open_pulse;
            else
                pulse = Arm.config.claw_close_pulse;
            break;
            
        default:
            break;
    }
    
    return pulse;
}

/**
 * 设置舵机角度
 * @param servo_id 舵机ID (1: 第一关节, 2: 第二关节, 3: 抓爪)
 * @param angle 角度 (弧度)
 */
void Arm_SetServoAngle(int servo_id, double angle)
{
    // 计算对应的脉冲宽度
    double pulse = Arm_AngleToPulse(servo_id, angle);
    
    // 设置舵机脉冲
    Servo_SetPulse(servo_id, pulse);
    
    // 更新当前角度
    if (servo_id == 1) {
        Arm.current_angles.theta1 = angle;
    } else if (servo_id == 2) {
        Arm.current_angles.theta2 = angle;
    }
}

/**
 * 根据当前的关节角度更新位置坐标
 */
static void Arm_UpdatePosition(void)
{
    double theta1 = Arm.current_angles.theta1;
    double theta2 = Arm.current_angles.theta2;
    double L1 = Arm.config.L1;
    double L2 = Arm.config.L2;
    
    // 正运动学计算
    Arm.current_pos.x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    Arm.current_pos.y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
}

/**
 * 计算到达指定位置的关节角度
 * @param x X坐标 (mm)
 * @param y Y坐标 (mm)
 * @return 关节角度结构体
 */
ArmAngles_t Arm_CalculateAngles(double x, double y)
{
    ArmAngles_t result = {0.0, 0.0, 0}; // 初始化结果
    double L1 = Arm.config.L1;
    double L2 = Arm.config.L2;
    
    double D_sq = x*x + y*y;
    double D = sqrt(D_sq);
    
    // --- 检查可达性 ---
    // 1. 是否超出最大伸展范围
    if (D > L1 + L2) {
        Arm.error_code = 1; // 设置错误代码：目标点超出范围
        return result;
    }
    
    // 2. 是否小于最小范围 (对于 L1 != L2 的情况)
    if (D < fabs(L1 - L2)) {
        Arm.error_code = 2; // 设置错误代码：目标点太靠近基座
        return result;
    }
    
    // 3. 特殊情况：目标点在原点
    if (D == 0 && L1+L2 > 0) {
        Arm.error_code = 3; // 设置错误代码：目标点在原点，角度未定义
        return result;
    }
    
    // --- 计算 theta2 ---
    double cos_theta2_arg = (D_sq - L1*L1 - L2*L2) / (2.0 * L1 * L2);
    
    // 由于浮点数精度问题，参数可能略微超出 [-1, 1]，需要夹紧
    if (cos_theta2_arg > 1.0) {
        cos_theta2_arg = 1.0;
    } else if (cos_theta2_arg < -1.0) {
        cos_theta2_arg = -1.0;
    }
    
    // 计算 theta2 (肘部向上解, 弧度)
    double theta2_rad = acos(cos_theta2_arg);
    
    // --- 计算 theta1 ---
    double alpha_rad = atan2(y, x); // 目标点与原点连线和 X 轴的夹角
    
    // 计算 beta (L1 与 D 之间的夹角)
    double cos_beta_arg = (L1*L1 + D_sq - L2*L2) / (2.0 * L1 * D);
    
    // 同样，夹紧参数到 [-1, 1]
    if (cos_beta_arg > 1.0) {
        cos_beta_arg = 1.0;
    } else if (cos_beta_arg < -1.0) {
        cos_beta_arg = -1.0;
    }
    
    double beta_rad = acos(cos_beta_arg);
    
    // 计算 theta1 (对应肘部向上解, 弧度)
    double theta1_rad = alpha_rad - beta_rad;
    
    // --- 检查角度是否在舵机范围内 ---
    if (theta1_rad < Arm.config.servo1_min_angle || theta1_rad > Arm.config.servo1_max_angle) {
        Arm.error_code = 4; // 设置错误代码：第一关节角度超出范围
        return result;
    }
    
    if (theta2_rad < Arm.config.servo2_min_angle || theta2_rad > Arm.config.servo2_max_angle) {
        Arm.error_code = 5; // 设置错误代码：第二关节角度超出范围
        return result;
    }
    
    // --- 填充结果 ---
    result.theta1 = theta1_rad;
    result.theta2 = theta2_rad;
    result.reachable = 1; // 标记为可达
    
    return result;
}

/**
 * 移动到指定位置
 * @param x X坐标 (mm)
 * @param y Y坐标 (mm)
 * @return 1:成功启动移动 0:目标不可达
 */
int Arm_MoveTo(double x, double y)
{
    // 计算新的关节角度
    ArmAngles_t angles = Arm_CalculateAngles(x, y);
    
    // 检查是否可达
    if (!angles.reachable) {
        return 0;
    }
    
    // 设置目标位置
    Arm.target_pos.x = x;
    Arm.target_pos.y = y;
    Arm.target_angles = angles;
    
    // 设置状态为移动中
    Arm.state = ARM_STATE_MOVING;
    Arm.action_start = HAL_TICK();
    Arm.action_timeout = ARM_MOVE_TIMEOUT;
    
    return 1;
}

/**
 * 移动到指定角度
 * @param theta1 第一关节角度 (弧度)
 * @param theta2 第二关节角度 (弧度)
 * @return 1:成功启动移动 0:角度超出范围
 */
int Arm_MoveToAngles(double theta1, double theta2)
{
    // 检查角度是否在范围内
    if (theta1 < Arm.config.servo1_min_angle || theta1 > Arm.config.servo1_max_angle ||
        theta2 < Arm.config.servo2_min_angle || theta2 > Arm.config.servo2_max_angle) {
        Arm.error_code = 6; // 设置错误代码：角度超出范围
        return 0;
    }
    
    // 设置目标角度
    Arm.target_angles.theta1 = theta1;
    Arm.target_angles.theta2 = theta2;
    Arm.target_angles.reachable = 1;
    
    // 计算目标位置
    double L1 = Arm.config.L1;
    double L2 = Arm.config.L2;
    Arm.target_pos.x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    Arm.target_pos.y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
    
    // 设置状态为移动中
    Arm.state = ARM_STATE_MOVING;
    Arm.action_start = HAL_TICK();
    Arm.action_timeout = ARM_MOVE_TIMEOUT;
    
    return 1;
}

/**
 * 停止移动
 */
void Arm_Stop(void)
{
    if (Arm.state == ARM_STATE_MOVING) {
        // 更新位置
        Arm_UpdatePosition();
        // 停止移动
        Arm.state = ARM_STATE_IDLE;
    }
}

/**
 * 打开抓爪
 */
void Arm_OpenClaw(void)
{
    Arm.claw_state = CLAW_STATE_OPENING;
    Servo_SetPulse(3, Arm.config.claw_open_pulse);
    Arm.has_object = 0;
}

/**
 * 闭合抓爪
 */
void Arm_CloseClaw(void)
{
    Arm.claw_state = CLAW_STATE_CLOSING;
    Servo_SetPulse(3, Arm.config.claw_close_pulse);
    
    // 如果在抓取状态，检测是否成功抓取到物体
    if (Arm.state == ARM_STATE_GRABBING) {
        Arm.has_object = Object_Detected();
    }
}

/**
 * 抓取当前位置的物体
 * @return 1:成功启动抓取 0:系统繁忙
 */
int Arm_Grab(void)
{
    // 检查当前状态是否允许抓取
    if (Arm.state != ARM_STATE_IDLE) {
        return 0;
    }
    
    // 设置状态为抓取中
    Arm.state = ARM_STATE_GRABBING;
    Arm.claw_state = CLAW_STATE_CLOSING;
    Arm.action_start = HAL_TICK();
    Arm.action_timeout = ARM_GRAB_TIMEOUT;
    
    // 闭合抓爪
    Arm_CloseClaw();
    
    return 1;
}

/**
 * 释放物体
 * @return 1:成功启动释放 0:系统繁忙或没有持有物体
 */
int Arm_Release(void)
{
    // 检查当前状态是否允许释放
    if (Arm.state != ARM_STATE_IDLE || !Arm.has_object) {
        return 0;
    }
    
    // 设置状态为释放中
    Arm.state = ARM_STATE_RELEASING;
    Arm.action_start = HAL_TICK();
    Arm.action_timeout = ARM_RELEASE_TIMEOUT;
    
    // 打开抓爪
    Arm_OpenClaw();
    
    return 1;
}

/**
 * 检查是否在移动
 * @return 1:正在移动 0:不在移动
 */
int Arm_IsMoving(void)
{
    return (Arm.state == ARM_STATE_MOVING);
}

/**
 * 检查是否持有物体
 * @return 1:持有物体 0:没有持有物体
 */
int Arm_IsHolding(void)
{
    return Arm.has_object;
}

/**
 * 检查是否有错误
 * @return 1:有错误 0:没有错误
 */
int Arm_HasError(void)
{
    return (Arm.state == ARM_STATE_ERROR);
}

/**
 * 检查是否到达指定位置
 * @param x X坐标 (mm)
 * @param y Y坐标 (mm)
 * @param tolerance 位置容差 (mm)
 * @return 1:已到达 0:未到达
 */
int Arm_IsAtPosition(double x, double y, double tolerance)
{
    double dx = Arm.current_pos.x - x;
    double dy = Arm.current_pos.y - y;
    double distance = sqrt(dx*dx + dy*dy);
    
    return (distance <= tolerance);
}

/**
 * 移动到初始位置
 */
void Arm_MoveToHome(void)
{
    Arm_MoveTo(ARM_HOME_X, ARM_HOME_Y);
}

/**
 * 移动到休息位置
 */
void Arm_MoveToRest(void)
{
    Arm_MoveTo(ARM_REST_X, ARM_REST_Y);
}

/**
 * 移动到抓取位置
 */
void Arm_MoveToGrabPosition(void)
{
    // 使用传感器获取物体位置
    Sensor_ReadDistance();
    
    // 计算物体位置，默认在机器人前方
    double distance = Sensor.distance_mm;
    
    // 限制抓取距离在有效范围内
    if (distance < DISTANCE_GRAB_MIN) {
        distance = DISTANCE_GRAB_MIN;
    } else if (distance > DISTANCE_GRAB_MAX) {
        distance = DISTANCE_GRAB_MAX;
    }
    
    // 使用默认抓取位置
    Arm_MoveTo(ARM_GRAB_X, ARM_GRAB_Y);
}

/**
 * 移动到放置位置
 */
void Arm_MoveToPutPosition(void)
{
    Arm_MoveTo(ARM_PUT_X, ARM_PUT_Y);
}

/**
 * 机械臂控制任务，应当在主循环中定期调用
 */
void Arm_Task(void)
{
    unsigned int current_time = HAL_TICK();
    double elapsed_time = (current_time - Arm.last_update) / 1000.0; // 转换为秒
    Arm.last_update = current_time;
    
    // 状态机处理
    switch (Arm.state) {
        case ARM_STATE_IDLE:
            // 空闲状态，无需处理
            break;
            
        case ARM_STATE_MOVING:
            // 处理移动状态
            if (current_time - Arm.action_start > Arm.action_timeout) {
                // 移动超时
                Arm.state = ARM_STATE_ERROR;
                Arm.error_code = ARM_ERROR_MOVE_TIMEOUT;
            } else {
                // 计算角度差值
                double d_theta0 = Arm.target_angles.theta0 - Arm.current_angles.theta0;
                double d_theta1 = Arm.target_angles.theta1 - Arm.current_angles.theta1;
                double d_theta2 = Arm.target_angles.theta2 - Arm.current_angles.theta2;
                
                // 计算最大移动步长
                double max_step = Arm.config.move_speed * elapsed_time;
                double max_rotate_step = Arm.config.rotate_speed * elapsed_time;
                
                // 限制移动步长
                if (fabs(d_theta0) > max_rotate_step) {
                    d_theta0 = (d_theta0 > 0) ? max_rotate_step : -max_rotate_step;
                }
                
                if (fabs(d_theta1) > max_step) {
                    d_theta1 = (d_theta1 > 0) ? max_step : -max_step;
                }
                
                if (fabs(d_theta2) > max_step) {
                    d_theta2 = (d_theta2 > 0) ? max_step : -max_step;
                }
                
                // 更新当前角度
                double new_theta0 = Arm.current_angles.theta0 + d_theta0;
                double new_theta1 = Arm.current_angles.theta1 + d_theta1;
                double new_theta2 = Arm.current_angles.theta2 + d_theta2;
                
                // 设置舵机角度
                double pulse0 = Arm_AngleToPulse(0, new_theta0);
                Servo_SetPulse(0, pulse0);
                Arm.current_angles.theta0 = new_theta0;
                
                Arm_SetServoAngle(1, new_theta1);
                Arm_SetServoAngle(2, new_theta2);
                
                // 更新位置
                Arm_UpdatePosition();
                
                // 检查是否到达目标
                if (Arm_IsAtPosition(Arm.target_pos.x, Arm.target_pos.y, ARM_POSITION_TOLERANCE) &&
                    fabs(Arm.current_angles.theta0 - Arm.target_angles.theta0) < 0.01) { // 约0.6度的容差
                    
                    // 设置最终角度
                    double pulse0 = Arm_AngleToPulse(0, Arm.target_angles.theta0);
                    Servo_SetPulse(0, pulse0);
                    Arm.current_angles.theta0 = Arm.target_angles.theta0;
                    
                    Arm_SetServoAngle(1, Arm.target_angles.theta1);
                    Arm_SetServoAngle(2, Arm.target_angles.theta2);
                    
                    // 更新位置
                    Arm_UpdatePosition();
                    
                    // 移动完成
                    Arm.state = ARM_STATE_IDLE;
                }
            }
            break;
            
        case ARM_STATE_GRABBING:
            // 处理抓取状态
            if (current_time - Arm.action_start > Arm.action_timeout) {
                // 抓取超时
                Arm.state = ARM_STATE_IDLE;
                
                // 检查是否成功抓取到物体
                Arm.has_object = Object_Detected();
                
                // 更新抓爪状态
                Arm.claw_state = Arm.has_object ? CLAW_STATE_CLOSED : CLAW_STATE_OPEN;
            }
            break;
            
        case ARM_STATE_HOLDING:
            // 持有物体状态，检查是否依然持有物体
            if (!Object_Detected()) {
                // 物体丢失
                Arm.has_object = 0;
                Arm.state = ARM_STATE_ERROR;
                Arm.error_code = ARM_ERROR_OBJECT_LOST;
            }
            break;
            
        case ARM_STATE_RELEASING:
            // 处理释放状态
            if (current_time - Arm.action_start > Arm.action_timeout) {
                // 释放超时或完成
                Arm.state = ARM_STATE_IDLE;
                Arm.has_object = 0;
                Arm.claw_state = CLAW_STATE_OPEN;
            }
            break;
            
        case ARM_STATE_ERROR:
            // 错误状态，需要外部干预
            break;
            
        default:
            // 未知状态，重置为空闲
            Arm.state = ARM_STATE_IDLE;
            break;
    }
    
    // 处理抓爪状态
    switch (Arm.claw_state) {
        case CLAW_STATE_OPENING:
            // 抓爪正在打开
            if (current_time - Arm.action_start > 500) { // 假设打开需要500ms
                Arm.claw_state = CLAW_STATE_OPEN;
                Arm.has_object = 0;
            }
            break;
            
        case CLAW_STATE_CLOSING:
            // 抓爪正在闭合
            if (current_time - Arm.action_start > 500) { // 假设闭合需要500ms
                Arm.claw_state = CLAW_STATE_CLOSED;
                
                // 如果在抓取状态，检测是否成功抓取到物体
                if (Arm.state == ARM_STATE_GRABBING) {
                    Arm.has_object = Object_Detected();
                    
                    // 如果抓取成功，更新状态
                    if (Arm.has_object) {
                        Arm.state = ARM_STATE_HOLDING;
                    } else {
                        Arm.state = ARM_STATE_IDLE;
                    }
                }
            }
            break;
            
        default:
            // 其他状态无需处理
            break;
    }
}

/**
 * 旋转底座到指定角度（弧度）
 * @param angle 目标角度（弧度）
 */
void Arm_RotateToAngle(double angle)
{
    // 限制角度在有效范围内
    if (angle < Arm.config.servo0_min_angle) {
        angle = Arm.config.servo0_min_angle;
    } else if (angle > Arm.config.servo0_max_angle) {
        angle = Arm.config.servo0_max_angle;
    }
    
    // 设置目标角度
    Arm.target_angles.theta0 = angle;
    
    // 计算脉冲宽度并设置舵机
    double pulse = Arm_AngleToPulse(0, angle);
    Servo_SetPulse(0, pulse);
    
    // 更新当前角度
    Arm.current_angles.theta0 = angle;
}

/**
 * 旋转底座到指定角度（度）
 * @param degrees 目标角度（度）
 */
void Arm_RotateToDegrees(double degrees)
{
    // 将角度转换为弧度
    double radians = degrees * M_PI / 180.0;
    Arm_RotateToAngle(radians);
}

/**
 * 旋转底座
 * @param angle 目标角度（弧度）
 * @return 1:成功启动旋转 0:角度超出范围
 */
int Arm_Rotate(double angle)
{
    // 检查角度是否在范围内
    if (angle < Arm.config.servo0_min_angle || angle > Arm.config.servo0_max_angle) {
        Arm.error_code = 8; // 设置错误代码：底座角度超出范围
        return 0;
    }
    
    // 设置目标角度
    Arm.target_angles.theta0 = angle;
    
    // 更新状态
    Arm.state = ARM_STATE_MOVING;
    Arm.action_start = HAL_TICK();
    Arm.action_timeout = ARM_MOVE_TIMEOUT;
    
    return 1;
}

/**
 * 获取错误描述
 * @return 错误描述字符串
 */
const char* Arm_GetErrorString(void)
{
    switch(Arm.error_code) {
        case ARM_ERROR_NONE:
            return "无错误";
        case ARM_ERROR_OUT_OF_RANGE:
            return "目标点超出范围";
        case ARM_ERROR_TOO_CLOSE:
            return "目标点太靠近基座";
        case ARM_ERROR_AT_ORIGIN:
            return "目标点在原点";
        case ARM_ERROR_JOINT1_LIMIT:
            return "第一关节角度超出范围";
        case ARM_ERROR_JOINT2_LIMIT:
            return "第二关节角度超出范围";
        case ARM_ERROR_ANGLE_LIMIT:
            return "角度超出范围";
        case ARM_ERROR_MOVE_TIMEOUT:
            return "移动超时";
        case ARM_ERROR_BASE_ANGLE_LIMIT:
            return "底座角度超出范围";
        case ARM_ERROR_OBJECT_LOST:
            return "物体丢失";
        case ARM_ERROR_GRAB_FAILED:
            return "抓取失败";
        default:
            return "未知错误";
    }
} 