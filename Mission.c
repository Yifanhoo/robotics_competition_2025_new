#include "Mission.h"
#include "Chassis.h"
#include "sensor.h"
#include "arm.h"
#include "servo.h"

// 任务队列最大长度
#define MAX_MISSION_QUEUE 20

// 测距传感器参数
#define DISTANCE_DETECT_THRESHOLD  300  // 物体检测阈值，单位毫米
#define DISTANCE_GRAB_MIN          50   // 最小抓取距离，单位毫米
#define DISTANCE_GRAB_MAX          150  // 最大抓取距离，单位毫米

// 全局任务队列
static Mission_t mission_queue[MAX_MISSION_QUEUE];
static int mission_queue_size = 0;
static int current_mission_index = 0;
static int mission_running = 0;

// 抓取状态机
typedef enum {
    GRAB_STATE_INIT,        // 初始状态
    GRAB_STATE_APPROACHING, // 接近物体
    GRAB_STATE_ADJUSTING,   // 调整位置
    GRAB_STATE_GRABBING,    // 执行抓取
    GRAB_STATE_COMPLETE     // 完成抓取
} GrabState_t;

static GrabState_t grab_state = GRAB_STATE_INIT;

// 任务系统初始化
void Mission_Init(void)
{
    // 初始化任务队列
    mission_queue_size = 0;
    current_mission_index = 0;
    mission_running = 0;
    grab_state = GRAB_STATE_INIT;
    
    // 初始化传感器
    Sensor_Init();
    
    // 初始化底盘
    Chassis_Init();
    
    // 初始化舵机
    Servo_Init();
    
    // 初始化机械臂
    Arm_Init();
}

// 添加任务到队列
void Mission_Add(MissionType_t type, int param)
{
    if (mission_queue_size < MAX_MISSION_QUEUE)
    {
        mission_queue[mission_queue_size].type = type;
        mission_queue[mission_queue_size].param = param;
        mission_queue[mission_queue_size].completed = 0;
        mission_queue[mission_queue_size].param_x = 0.0;
        mission_queue[mission_queue_size].param_y = 0.0;
        mission_queue_size++;
    }
}

// 添加带位置参数的任务到队列
void Mission_AddWithPosition(MissionType_t type, double x, double y)
{
    if (mission_queue_size < MAX_MISSION_QUEUE)
    {
        mission_queue[mission_queue_size].type = type;
        mission_queue[mission_queue_size].param = 0;
        mission_queue[mission_queue_size].completed = 0;
        mission_queue[mission_queue_size].param_x = x;
        mission_queue[mission_queue_size].param_y = y;
        mission_queue_size++;
    }
}

// 开始执行任务队列
void Mission_Start(void)
{
    if (mission_queue_size > 0)
    {
        current_mission_index = 0;
        mission_running = 1;
    }
}

// 停止任务执行
void Mission_Stop(void)
{
    mission_running = 0;
    
    // 停止底盘
    Chassis.act = STOP;
    
    // 停止机械臂
    Arm_Stop();
}

// 检查所有任务是否完成
int Mission_IsCompleted(void)
{
    return (current_mission_index >= mission_queue_size);
}

// 将任务类型转换为底盘动作
static void ConvertMissionToChassis(MissionType_t mission_type)
{
    switch (mission_type)
    {
        case MISSION_FOLLOW_LINE:
            Chassis.act = MOVE;
            break;
        case MISSION_TURN_LEFT:
        case MISSION_TURN_RIGHT:
            Chassis.act = TURN;
            // 设置转向方向
            if (mission_type == MISSION_TURN_LEFT)
                Chassis.rotate_speed = -300; // 负值表示左转
            else
                Chassis.rotate_speed = 300;  // 正值表示右转
            break;
        case MISSION_STOP:
            Chassis.act = STOP;
            break;
        case MISSION_GRAB:
            Chassis.act = GRAB;
            break;
        case MISSION_PUT:
            Chassis.act = PUT;
            break;
        // 机械臂相关任务不影响底盘
        case MISSION_ARM_MOVE_TO:
        case MISSION_ARM_GRAB:
        case MISSION_ARM_RELEASE:
        case MISSION_ARM_HOME:
        case MISSION_ARM_REST:
            // 这些任务不改变底盘状态
            break;
        default:
            Chassis.act = STOP;
            break;
    }
}

// 处理抓取状态机
static int HandleGrabState(void)
{
    static unsigned int grab_timeout = 0;
    
    // 读取距离传感器数据
    Sensor_ReadDistance();
    
    switch(grab_state)
    {
        case GRAB_STATE_INIT:
            // 初始化抓取超时计时
            grab_timeout = HAL_TICK();
            grab_state = GRAB_STATE_APPROACHING;
            Chassis.act = MOVE;  // 开始向前移动
            Chassis.moving_speed = 100;  // 使用较慢的速度
            return 0;
            
        case GRAB_STATE_APPROACHING:
            // 检查是否检测到物体
            if(Sensor_IsObjectDetected())
            {
                if(Sensor_IsGrabbable())
                {
                    // 物体在可抓取范围内
                    Chassis.act = STOP;
                    grab_state = GRAB_STATE_ADJUSTING;
                }
                else if(Sensor.distance_mm < DISTANCE_GRAB_MIN)
                {
                    // 太近了，需要后退
                    Chassis.act = MOVE;
                    Chassis.moving_speed = -50;
                }
            }
            
            // 检查超时
            if(HAL_TICK() - grab_timeout > 5000)  // 5秒超时
            {
                grab_state = GRAB_STATE_COMPLETE;
                return 1;  // 超时失败
            }
            return 0;
            
        case GRAB_STATE_ADJUSTING:
            // 调整位置，确保物体在最佳抓取范围内
            if(Sensor_IsGrabbable())
            {
                grab_state = GRAB_STATE_GRABBING;
                Chassis.act = GRAB;
                grab_timeout = HAL_TICK();
            }
            else
            {
                grab_state = GRAB_STATE_APPROACHING;
            }
            return 0;
            
        case GRAB_STATE_GRABBING:
            // 等待抓取动作完成
            if(HAL_TICK() - grab_timeout > 1000)  // 假设抓取需要1秒
            {
                grab_state = GRAB_STATE_COMPLETE;
                return 0;  // 抓取成功
            }
            return 0;
            
        case GRAB_STATE_COMPLETE:
            // 重置状态机
            grab_state = GRAB_STATE_INIT;
            return 1;
            
        default:
            grab_state = GRAB_STATE_INIT;
            return 1;
    }
}

// 处理任务状态机
static int HandleArmTask(Mission_t *mission)
{
    static unsigned int arm_task_start = 0;
    static int arm_task_initialized = 0;
    
    // 如果是新任务，初始化状态
    if (!arm_task_initialized)
    {
        arm_task_start = HAL_TICK();
        arm_task_initialized = 1;
        
        // 根据任务类型执行相应的机械臂操作
        switch (mission->type)
        {
            case MISSION_ARM_MOVE_TO:
                Arm_MoveTo(mission->param_x, mission->param_y);
                break;
                
            case MISSION_ARM_GRAB:
                Arm_MoveToGrabPosition();
                Arm_Grab();
                break;
                
            case MISSION_ARM_RELEASE:
                Arm_Release();
                break;
                
            case MISSION_ARM_HOME:
                Arm_MoveToHome();
                break;
                
            case MISSION_ARM_REST:
                Arm_MoveToRest();
                break;
                
            default:
                // 未知任务，认为已完成
                return 1;
        }
    }
    
    // 更新机械臂状态
    Arm_Task();
    Servo_Task();
    
    // 判断任务是否完成
    switch (mission->type)
    {
        case MISSION_ARM_MOVE_TO:
            // 如果不在移动状态，则认为已到达目标
            if (!Arm_IsMoving())
            {
                arm_task_initialized = 0;
                return 1;
            }
            break;
            
        case MISSION_ARM_GRAB:
            // 如果成功抓取到物体，则完成
            if (Arm_IsHolding())
            {
                arm_task_initialized = 0;
                return 1;
            }
            // 超时检查
            if (HAL_TICK() - arm_task_start > 10000)  // 10秒超时
            {
                arm_task_initialized = 0;
                return 1;  // 超时失败
            }
            break;
            
        case MISSION_ARM_RELEASE:
            // 如果没有持有物体，则释放完成
            if (!Arm_IsHolding())
            {
                arm_task_initialized = 0;
                return 1;
            }
            // 超时检查
            if (HAL_TICK() - arm_task_start > 5000)  // 5秒超时
            {
                arm_task_initialized = 0;
                return 1;  // 超时失败
            }
            break;
            
        case MISSION_ARM_HOME:
        case MISSION_ARM_REST:
            // 如果不在移动状态，则已到达目标位置
            if (!Arm_IsMoving())
            {
                arm_task_initialized = 0;
                return 1;
            }
            // 超时检查
            if (HAL_TICK() - arm_task_start > 5000)  // 5秒超时
            {
                arm_task_initialized = 0;
                return 1;  // 超时失败
            }
            break;
            
        default:
            arm_task_initialized = 0;
            return 1;
    }
    
    // 任务仍在执行中
    return 0;
}

// 任务执行主函数，应在主循环中调用
void Mission_Task(void)
{
    if (!mission_running || Mission_IsCompleted())
        return;
    
    // 更新传感器数据
    Sensor_Read();
    
    // 获取当前任务
    Mission_t *current_mission = &mission_queue[current_mission_index];
    
    // 判断是底盘任务还是机械臂任务
    int is_arm_task = (current_mission->type >= MISSION_ARM_MOVE_TO && 
                        current_mission->type <= MISSION_ARM_REST);
    
    if (is_arm_task)
    {
        // 处理机械臂任务
        if (HandleArmTask(current_mission))
        {
            // 机械臂任务完成，进入下一个任务
            current_mission->completed = 1;
            current_mission_index++;
        }
    }
    else
    {
        // 根据任务类型设置底盘动作
        ConvertMissionToChassis(current_mission->type);
        
        // 执行底盘任务
        Chassis_Task();
        
        // 检查当前任务是否完成
        switch (current_mission->type)
        {
            case MISSION_FOLLOW_LINE:
                // 如果到达目标距离或检测到特殊标记，则完成
                if (current_mission->param > 0)
                {
                    // TODO: 加入里程计功能检测行驶距离
                    // 暂时使用简单的循迹状态来判定
                    if (Sensor_GetTrackStatus() == TRACK_STATUS_CROSS)
                    {
                        current_mission->completed = 1;
                        current_mission_index++;
                    }
                }
                break;
                
            case MISSION_TURN_LEFT:
            case MISSION_TURN_RIGHT:
                // 如果转向足够角度，则完成
                // TODO: 加入陀螺仪判断转向角度
                // 暂时简单判断是否检测到新的直线
                if (Sensor_GetTrackStatus() == TRACK_STATUS_NORMAL)
                {
                    current_mission->completed = 1;
                    current_mission_index++;
                }
                break;
                
            case MISSION_STOP:
                // 停止任务立即完成
                current_mission->completed = 1;
                current_mission_index++;
                break;
                
            case MISSION_GRAB:
                // 处理抓取状态机
                if (HandleGrabState())
                {
                    current_mission->completed = 1;
                    current_mission_index++;
                }
                break;
                
            case MISSION_PUT:
                // 处理放置物体的逻辑
                // TODO: 实现放置物体的状态机
                current_mission->completed = 1;
                current_mission_index++;
                break;
                
            default:
                // 未知任务类型，直接标记为完成
                current_mission->completed = 1;
                current_mission_index++;
                break;
        }
    }
    
    // 检查是否所有任务都完成
    if (Mission_IsCompleted())
    {
        // 停止执行
        Chassis.act = STOP;
        mission_running = 0;
    }
}

// 示例：设置一个简单的任务序列
void Mission_SetDefaultSequence(void)
{
    // 清空任务队列
    mission_queue_size = 0;
    
    // 添加任务序列
    // 沿线前进1个交叉点
    Mission_Add(MISSION_FOLLOW_LINE, 1);
    
    // 在交叉点处左转
    Mission_Add(MISSION_TURN_LEFT, 90);
    
    // 沿线前进2个交叉点
    Mission_Add(MISSION_FOLLOW_LINE, 2);
    
    // 停止500毫秒
    Mission_Add(MISSION_STOP, 500);
    
    // 抓取物体
    Mission_Add(MISSION_GRAB, 0);
    
    // 右转
    Mission_Add(MISSION_TURN_RIGHT, 180);
    
    // 沿线返回
    Mission_Add(MISSION_FOLLOW_LINE, 2);
    
    // 放下物体
    Mission_Add(MISSION_PUT, 0);
    
    // 停止
    Mission_Add(MISSION_STOP, 0);
}