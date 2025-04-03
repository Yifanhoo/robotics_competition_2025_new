#ifndef MISSION_H
#define MISSION_H

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
    MISSION_MAX
} MissionType_t;

// 任务结构体定义
typedef struct {
    MissionType_t type;
    int param;      // 可选参数，例如要行驶的距离或转弯角度
    int completed;  // 任务完成标志
    double param_x; // 位置参数X（用于ARM_MOVE_TO任务）
    double param_y; // 位置参数Y（用于ARM_MOVE_TO任务）
} Mission_t;

// 函数声明
void Mission_Init(void);
void Mission_Task(void);
void Mission_Add(MissionType_t type, int param);
void Mission_AddWithPosition(MissionType_t type, double x, double y);
void Mission_Start(void);
void Mission_Stop(void);
int Mission_IsCompleted(void);

#endif /* MISSION_H */
