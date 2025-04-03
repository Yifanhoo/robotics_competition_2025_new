#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>  // 添加对uint32_t的支持

// 不需要包含具体的硬件抽象层头文件，在 sensor.c 中处理硬件抽象

// 定义网格大小
#define GRID_SIZE 4
#define GRID_POINTS 15  // 火种可能的放置点数量 (A-O)

// 定义场地尺寸（单位：mm）
#define FIELD_WIDTH  4200
#define FIELD_LENGTH 3200
#define GRID_SIZE_MM 600   // 每个网格的实际尺寸

// 火种相关定义
typedef enum {
    FLAME_NONE = 0,
    FLAME_RED = 1,
    FLAME_BLUE = 2
} FlameColor_t;

// 火种位置状态
typedef struct {
    char point_id;        // 火种位置标识(A-O)
    FlameColor_t color;   // 火种颜色
    int is_collected;     // 是否已被收集
} FlamePoint_t;

// 定义一些常用的方向和状态
typedef enum {
    DIRECTION_LEFT = 0,
    DIRECTION_RIGHT = 1,
    DIRECTION_CENTER = 2,
    DIRECTION_FORWARD = 3,
    DIRECTION_BACKWARD = 4
} Direction_t;

// 定义机器人朝向
typedef enum {
    HEADING_NORTH = 0,  // 向上
    HEADING_EAST = 1,   // 向右
    HEADING_SOUTH = 2,  // 向下
    HEADING_WEST = 3    // 向左
} Heading_t;

// 定义任务状态
typedef enum {
    TASK_IDLE = 0,        // 空闲状态
    TASK_GOTO_FLAME = 1,  // 前往火种位置
    TASK_GRAB_FLAME = 2,  // 抓取火种
    TASK_GOTO_WALL = 3,   // 前往城墙
    TASK_PLACE_FLAME = 4, // 放置火种
    TASK_RETURN = 5       // 返回起始位置
} TaskState_t;

typedef enum {
    TRACK_STATUS_NORMAL = 0,    // 正常循迹（在直线上）
    TRACK_STATUS_LOST = 1,      // 丢线
    TRACK_STATUS_CROSS = 2,     // 十字路口
    TRACK_STATUS_END = 3        // 终点（全白）
} TrackStatus_t;

// 网格位置结构体
typedef struct {
    int x;  // 0-3，从左到右
    int y;  // 0-3，从上到下
} GridPosition_t;

// 路径规划相关定义
typedef struct {
    GridPosition_t path[GRID_SIZE * GRID_SIZE];
    int length;
} Path_t;

// 错误状态定义
typedef enum {
    ERROR_NONE = 0,
    ERROR_LOST_LINE = 1,
    ERROR_OBSTACLE = 2,
    ERROR_INVALID_POSITION = 3,
    ERROR_GRAB_FAIL = 4,
    ERROR_TASK_TIMEOUT = 5,
    ERROR_EMERGENCY_STOP = 6,
    ERROR_CRITICAL = 7
} ErrorStatus_t;

// 任务执行结果
typedef enum {
    TASK_RESULT_SUCCESS = 0,
    TASK_RESULT_FAILED = 1,
    TASK_RESULT_IN_PROGRESS = 2
} TaskResult_t;

// 任务优先级定义
typedef enum {
    PRIORITY_LOW = 0,
    PRIORITY_NORMAL = 1,
    PRIORITY_HIGH = 2,
    PRIORITY_URGENT = 3
} TaskPriority_t;

// 任务信息结构体
typedef struct {
    TaskState_t state;          // 任务状态
    TaskPriority_t priority;    // 任务优先级
    uint32_t start_time;        // 任务开始时间
    uint32_t timeout;           // 任务超时时间（毫秒）
    int retry_limit;            // 重试次数限制
} TaskInfo_t;

// 传感器数据结构
typedef struct 
{
    char sensor[8];             // 8个循迹传感器的数据
    int distance_mm;            // 测距传感器读数，单位毫米
    int cross_count;           // 经过的十字路口计数
    Direction_t last_turn;     // 上一次转向方向
    GridPosition_t position;   // 当前网格位置
    Heading_t heading;        // 当前朝向
    TaskState_t task_state;   // 当前任务状态
    FlamePoint_t flames[GRID_POINTS];  // 火种位置信息
    int team_color;           // 队伍颜色(0:红队, 1:蓝队)
    int is_auto;             // 是否为自动模式
    Path_t current_path;         // 当前规划路径
    ErrorStatus_t error_status;  // 错误状态
    int retry_count;            // 重试计数
    TaskResult_t task_result;   // 任务执行结果
    TaskInfo_t task_info;       // 任务详细信息
    uint32_t last_error_time;   // 上次错误发生时间
    uint32_t operation_time;    // 系统运行时间
    int total_flames_collected; // 已收集的火种总数
    float battery_percentage;   // 电池电量百分比
} Sensor_t;

// 基础函数
void Sensor_Init(void);                // 初始化传感器
void Sensor_Read(void);                // 读取循迹传感器
void Sensor_ReadDistance(void);        // 读取距离传感器
int Sensor_GetLinePosition(void);      // 获取循迹位置
float Sensor_GetDeviation(void);       // 获取偏离中线的程度
int Sensor_IsObjectDetected(void);     // 检测是否有物体
int Sensor_IsGrabbable(void);         // 检查是否可以抓取

// 网格导航相关函数
TrackStatus_t Sensor_GetTrackStatus(void);     // 获取当前循迹状态
int Sensor_IsAtCross(void);                    // 检测是否在十字路口
int Sensor_IsAtEnd(void);                      // 检测是否到达终点
void Sensor_ResetCrossCount(void);             // 重置十字路口计数
int Sensor_GetCrossCount(void);                // 获取经过的十字路口数
void Sensor_SetLastTurn(Direction_t dir);      // 设置上一次转向方向
Direction_t Sensor_GetLastTurn(void);          // 获取上一次转向方向

// 位置和路径相关函数
void Sensor_SetPosition(int x, int y);         // 设置当前位置
void Sensor_SetHeading(Heading_t heading);     // 设置当前朝向
GridPosition_t Sensor_GetPosition(void);       // 获取当前位置
Heading_t Sensor_GetHeading(void);            // 获取当前朝向
void Sensor_UpdatePosition(Direction_t turn);  // 根据转向更新位置和朝向
int Sensor_IsValidPosition(int x, int y);      // 检查位置是否有效
Direction_t Sensor_GetTurnDirection(GridPosition_t target); // 计算到达目标位置需要的转向

// 抓取相关函数
int Sensor_IsObjectInRange(int min_dist, int max_dist); // 检查物体是否在指定范围内
int Sensor_GetOptimalGrabDistance(void);    // 获取最佳抓取距离
int Sensor_NeedAdjustPosition(void);        // 检查是否需要调整位置
Direction_t Sensor_GetAdjustDirection(void); // 获取位置调整方向

// 火种相关函数
void Sensor_InitFlamePoints(void);                    // 初始化火种位置信息
void Sensor_SetFlamePoint(char id, FlameColor_t color); // 设置火种位置
void Sensor_SetTeamColor(int is_blue);                // 设置队伍颜色
void Sensor_SetAutoMode(int is_auto);                 // 设置自动/手动模式
FlamePoint_t* Sensor_GetNearestFlame(void);          // 获取最近的未收集火种
int Sensor_IsValidFlamePoint(char point_id);         // 检查火种位置是否有效
void Sensor_MarkFlameCollected(char point_id);       // 标记火种已被收集
GridPosition_t Sensor_FlamePointToGrid(char point_id); // 将火种位置转换为网格坐标

// 路径规划相关函数
Path_t Sensor_PlanPath(GridPosition_t start, GridPosition_t target);
int Sensor_IsPathValid(Path_t path);
void Sensor_SetCurrentPath(Path_t path);
Path_t Sensor_GetCurrentPath(void);
Direction_t Sensor_GetNextPathDirection(void);

// 任务管理相关函数
TaskResult_t Sensor_ExecuteTask(void);
void Sensor_SetTaskState(TaskState_t state);
TaskState_t Sensor_GetTaskState(void);
int Sensor_IsTaskCompleted(void);

// 错误处理相关函数
void Sensor_SetErrorStatus(ErrorStatus_t error);
ErrorStatus_t Sensor_GetErrorStatus(void);
int Sensor_HandleError(void);
void Sensor_ResetError(void);

// 状态监控函数
float Sensor_GetBatteryVoltage(void);
int Sensor_IsBatteryLow(void);
int Sensor_GetMotorStatus(void);
int Sensor_GetGripperStatus(void);

// 全局传感器实例
extern Sensor_t Sensor;

#endif /* SENSOR_H */
