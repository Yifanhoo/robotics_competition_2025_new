#ifndef MISSION_H
#define MISSION_H

// 火种位置最大数量
#define MAX_FIRE_POSITIONS 8

// 火种颜色定义
typedef enum {
    FIRE_COLOR_NONE = 0,
    FIRE_COLOR_RED,
    FIRE_COLOR_BLUE
} FireColor_t;

// 火种位置结构体
typedef struct {
    double x;
    double y;
    FireColor_t color;       // 火种颜色
    int is_occupied;         // 是否已被占用
    int is_collected;        // 是否已被收集
} FirePosition_t;

// 任务类型定义
typedef enum {
    MISSION_NONE = 0,
    MISSION_FOLLOW_LINE,
    MISSION_TURN_LEFT,
    MISSION_TURN_RIGHT,
    MISSION_STOP,
    MISSION_GRAB,
    MISSION_PUT,
    MISSION_ARM_MOVE_TO,     // 移动机械臂到指定位置
    MISSION_ARM_GRAB,        // 机械臂抓取
    MISSION_ARM_RELEASE,     // 机械臂释放
    MISSION_ARM_HOME,        // 机械臂回到初始位置
    MISSION_ARM_REST,        // 机械臂回到休息位置
    MISSION_GOTO_FIRE,      // 前往火种位置
    MISSION_PLACE_FIRE,     // 放置火种
    MISSION_MAX
} MissionType_t;

// 任务结构体定义
typedef struct {
    MissionType_t type;
    int param;      // 可选参数，例如要行驶的距离或转弯角度
    int completed;  // 任务完成标志
    double param_x; // 位置参数X（用于ARM_MOVE_TO任务）
    double param_y; // 位置参数Y（用于ARM_MOVE_TO任务）
    FireColor_t color; // 火种颜色参数
} Mission_t;

// 全局变量声明
extern int current_mission_index;
extern int mission_queue_size;

// 函数声明
void Mission_Init(void);
void Mission_Task(void);
int Mission_Add(MissionType_t type, int param);
void Mission_AddWithPosition(MissionType_t type, double x, double y, FireColor_t color);
void Mission_Start(void);
void Mission_Stop(void);
int Mission_IsCompleted(void);

// 火种相关函数
void Mission_SetFirePosition(int index, double x, double y, FireColor_t color);
void Mission_ClearFirePositions(void);
int Mission_GetNextAvailableFirePosition(double *x, double *y);
void Mission_MarkFirePositionCollected(int index);
FireColor_t Mission_GetFireColor(int index);
int Mission_GetCollectedFireCount(void);
int Mission_GetCollectedFireCountByColor(FireColor_t color);

#endif /* MISSION_H */
