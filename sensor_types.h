#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>

// 网格大小定义
#define GRID_SIZE_X 8
#define GRID_SIZE_Y 8
#define GRID_POINTS (GRID_SIZE_X * GRID_SIZE_Y)

// 方向枚举
typedef enum {
    DIRECTION_NONE = 0,
    DIRECTION_FORWARD,
    DIRECTION_BACKWARD,
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
    DIRECTION_CENTER  // 添加中心方向
} Direction_t;

// 朝向枚举
typedef enum {
    HEADING_NORTH = 0,
    HEADING_EAST,
    HEADING_SOUTH,
    HEADING_WEST
} Heading_t;

// 网格位置结构体
typedef struct {
    int x;
    int y;
} GridPosition_t;

// 路径节点结构体
typedef struct PathNode {
    GridPosition_t position;
    struct PathNode* next;
} PathNode_t;

// 路径结构体
typedef struct {
    PathNode_t* head;
    int length;
} Path_t;

// 任务状态枚举
typedef enum {
    TASK_STATE_IDLE = 0,
    TASK_STATE_RUNNING,
    TASK_STATE_COMPLETED,
    TASK_STATE_ERROR,
    TASK_STATE_PAUSED
} TaskState_t;

// 错误状态枚举
typedef enum {
    ERROR_NONE = 0,
    ERROR_SENSOR_FAILURE,
    ERROR_PATH_BLOCKED,
    ERROR_POSITION_LOST,
    ERROR_TIMEOUT,
    ERROR_HARDWARE_FAILURE
} ErrorStatus_t;

// 任务信息结构体
typedef struct {
    int task_id;
    int priority;
    uint32_t start_time;
    uint32_t timeout;
} TaskInfo_t;

// 循迹状态枚举
typedef enum {
    TRACK_STATUS_NORMAL = 0,
    TRACK_STATUS_LOST,
    TRACK_STATUS_CROSS
} TrackStatus_t;

// 火种颜色枚举
typedef enum {
    FLAME_NONE = 0,
    FLAME_RED,
    FLAME_BLUE
} FlameColor_t;

// 火种点结构体
typedef struct {
    GridPosition_t position;
    FlameColor_t color;
    int is_collected;
} FlamePoint_t;

// 循迹传感器结构体
typedef struct {
    char sensors[8];        // 8个循迹传感器数据
    int cross_count;        // 十字路口计数
    Direction_t last_turn;  // 上一次转向方向
} LineTracker_t;

// 激光测距传感器结构体
typedef struct {
    int distance_mm;        // 距离值（毫米）
    int is_object_detected; // 是否检测到物体
} LaserRangeFinder_t;

// 位置和导航结构体
typedef struct {
    GridPosition_t position;  // 当前网格位置
    Heading_t heading;       // 当前朝向
    Path_t current_path;     // 当前规划路径
} Navigation_t;

// 任务管理结构体
typedef struct {
    TaskState_t task_state;     // 当前任务状态
    ErrorStatus_t error_status; // 错误状态
    TaskInfo_t task_info;       // 任务详细信息
    int retry_count;            // 重试计数
    uint32_t last_error_time;   // 上次错误发生时间
    uint32_t operation_time;    // 系统运行时间
} TaskManager_t;

// 火种管理结构体
typedef struct {
    FlamePoint_t flames[GRID_POINTS];  // 火种位置信息
    int team_color;                    // 队伍颜色(0:红队, 1:蓝队)
    int total_flames_collected;        // 已收集的火种总数
} FlameManager_t;

// 传感器系统总结构体
typedef struct {
    LineTracker_t line_tracker;       // 循迹传感器
    LaserRangeFinder_t range_finder;  // 激光测距传感器
    Navigation_t navigation;          // 导航系统
    TaskManager_t task_manager;       // 任务管理器
    FlameManager_t flame_manager;     // 火种管理器
} SensorSystem_t;

// 全局传感器系统实例
extern SensorSystem_t SensorSystem;

#endif /* SENSOR_TYPES_H */ 