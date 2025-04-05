#include "sensor.h"
#include <stdlib.h>
#include <string.h>

// 传感器系统全局实例
SensorSystem_t SensorSystem;

// 传感器状态定义
#define WHITE_LINE   1           // 循迹线 - 传感器位于白线上时的返回值
#define GREEN_BACKGROUND 0       // 背景 - 传感器位于绿色赛道背景时的返回值

// 超声波传感器阈值配置（单位：mm）
#define DISTANCE_DETECT_THRESHOLD  300  // 物体检测阈值
#define DISTANCE_GRAB_MIN          50   // 最小抓取距离
#define DISTANCE_GRAB_MAX          150  // 最大抓取距离

// 位置调整相关阈值
#define POSITION_ADJUST_THRESHOLD  20   // 位置调整阈值（mm）

// 传感器状态判定阈值
#define CROSS_SENSOR_THRESHOLD     6    // 判定为十字路口的最小传感器数

// 外部硬件接口函数声明
extern char Read_IO1(void);  // 最左
extern char Read_IO2(void);  // 左2
extern char Read_IO3(void);  // 左3
extern char Read_IO4(void);  // 中左
extern char Read_IO5(void);  // 中右
extern char Read_IO6(void);  // 右3
extern char Read_IO7(void);  // 右2
extern char Read_IO8(void);  // 最右
extern int Read_Ultrasonic(void);  // 超声波测距

/**
 * 传感器系统初始化
 */
void Sensor_Init(void)
{
    // 初始化循迹传感器
    memset(SensorSystem.line_tracker.sensors, 0, sizeof(SensorSystem.line_tracker.sensors));
    SensorSystem.line_tracker.cross_count = 0;
    SensorSystem.line_tracker.last_turn = DIRECTION_CENTER;
    
    // 初始化激光测距传感器
    SensorSystem.range_finder.distance_mm = 0;
    SensorSystem.range_finder.is_object_detected = 0;
    
    // 初始化导航系统
    SensorSystem.navigation.position.x = 0;
    SensorSystem.navigation.position.y = 0;
    SensorSystem.navigation.heading = HEADING_SOUTH;
    
    // 初始化任务管理器
    SensorSystem.task_manager.task_state = TASK_IDLE;
    SensorSystem.task_manager.error_status = ERROR_NONE;
    SensorSystem.task_manager.retry_count = 0;
    
    // 初始化火种管理器
    memset(SensorSystem.flame_manager.flames, 0, sizeof(SensorSystem.flame_manager.flames));
    SensorSystem.flame_manager.team_color = 0;
    SensorSystem.flame_manager.total_flames_collected = 0;
    
    // 初始化火种位置信息
    Sensor_InitFlamePoints();
}

/**
 * 读取循迹传感器
 */
void Sensor_ReadLineTracker(void)
{
    SensorSystem.line_tracker.sensors[0] = Read_IO1();
    SensorSystem.line_tracker.sensors[1] = Read_IO2();
    SensorSystem.line_tracker.sensors[2] = Read_IO3();
    SensorSystem.line_tracker.sensors[3] = Read_IO4();
    SensorSystem.line_tracker.sensors[4] = Read_IO5();
    SensorSystem.line_tracker.sensors[5] = Read_IO6();
    SensorSystem.line_tracker.sensors[6] = Read_IO7();
    SensorSystem.line_tracker.sensors[7] = Read_IO8();
}

/**
 * 读取距离传感器
 */
void Sensor_ReadDistance(void)
{
    SensorSystem.range_finder.distance_mm = Read_Ultrasonic();
    SensorSystem.range_finder.is_object_detected = 
        (SensorSystem.range_finder.distance_mm < DISTANCE_DETECT_THRESHOLD);
}

/**
 * 获取循迹位置
 */
int Sensor_GetLinePosition(void)
{
    int weighted_sum = 0;
    int count = 0;
    
    for(int i = 0; i < 8; i++) {
        if(SensorSystem.line_tracker.sensors[i] == WHITE_LINE) {
            weighted_sum += (i + 1);
            count++;
        }
    }
    
    return count ? weighted_sum / count : 0;
}

/**
 * 计算偏离中线程度
 */
float Sensor_GetDeviation(void)
{
    int position = Sensor_GetLinePosition();
    return position ? (float)(position - 4.5) : 0;
}

/**
 * 获取当前循迹状态
 */
TrackStatus_t Sensor_GetTrackStatus(void)
{
    int active_count = 0;
    
    for(int i = 0; i < 8; i++) {
        if(SensorSystem.line_tracker.sensors[i] == WHITE_LINE) {
            active_count++;
        }
    }
    
    if(active_count == 0) {
        return TRACK_STATUS_LOST;
    } else if(active_count >= CROSS_SENSOR_THRESHOLD) {
        return TRACK_STATUS_CROSS;
    }
    
    return TRACK_STATUS_NORMAL;
}

/**
 * 检测是否在十字路口
 */
int Sensor_IsAtCross(void)
{
    return (Sensor_GetTrackStatus() == TRACK_STATUS_CROSS);
}

/**
 * 重置十字路口计数
 */
void Sensor_ResetCrossCount(void)
{
    SensorSystem.line_tracker.cross_count = 0;
}

/**
 * 获取经过的十字路口数
 */
int Sensor_GetCrossCount(void)
{
    return SensorSystem.line_tracker.cross_count;
}

/**
 * 检测前方物体
 */
int Sensor_IsObjectDetected(void)
{
    return SensorSystem.range_finder.is_object_detected;
}

/**
 * 判断物体是否可抓取
 */
int Sensor_IsGrabbable(void)
{
    return Sensor_IsObjectInRange(DISTANCE_GRAB_MIN, DISTANCE_GRAB_MAX);
}

/**
 * 检查物体是否在指定距离范围内
 */
int Sensor_IsObjectInRange(int min_dist, int max_dist)
{
    return (SensorSystem.range_finder.distance_mm >= min_dist && 
            SensorSystem.range_finder.distance_mm <= max_dist);
}

/**
 * 获取最佳抓取距离
 */
int Sensor_GetOptimalGrabDistance(void)
{
    return (DISTANCE_GRAB_MIN + DISTANCE_GRAB_MAX) / 2;
}

/**
 * 检查是否需要调整位置
 */
int Sensor_NeedAdjustPosition(void)
{
    return (abs(SensorSystem.range_finder.distance_mm - Sensor_GetOptimalGrabDistance()) 
            > POSITION_ADJUST_THRESHOLD);
}

/**
 * 设置当前位置
 */
void Sensor_SetPosition(int x, int y)
{
    if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
        SensorSystem.navigation.position.x = x;
        SensorSystem.navigation.position.y = y;
    }
}

/**
 * 设置当前朝向
 */
void Sensor_SetHeading(Heading_t heading)
{
    if (heading >= HEADING_NORTH && heading <= HEADING_WEST) {
        SensorSystem.navigation.heading = heading;
    }
}

/**
 * 获取当前位置
 */
GridPosition_t Sensor_GetPosition(void)
{
    return SensorSystem.navigation.position;
}

/**
 * 获取当前朝向
 */
Heading_t Sensor_GetHeading(void)
{
    return SensorSystem.navigation.heading;
}

/**
 * 根据转向更新位置和朝向
 */
void Sensor_UpdatePosition(Direction_t turn)
{
    // 更新朝向
    Heading_t new_heading = SensorSystem.navigation.heading;
    if (turn == DIRECTION_LEFT) {
        new_heading = (SensorSystem.navigation.heading - 1) & 0x03;  // 循环减1
    } else if (turn == DIRECTION_RIGHT) {
        new_heading = (SensorSystem.navigation.heading + 1) & 0x03;  // 循环加1
    }
    SensorSystem.navigation.heading = new_heading;
    
    // 根据当前朝向更新位置
    int new_x = SensorSystem.navigation.position.x;
    int new_y = SensorSystem.navigation.position.y;
    
    switch (SensorSystem.navigation.heading) {
        case HEADING_NORTH:
            new_y--;
            break;
        case HEADING_EAST:
            new_x++;
            break;
        case HEADING_SOUTH:
            new_y++;
            break;
        case HEADING_WEST:
            new_x--;
            break;
    }
    
    // 检查新位置是否有效
    if (new_x >= 0 && new_x < GRID_SIZE && new_y >= 0 && new_y < GRID_SIZE) {
        SensorSystem.navigation.position.x = new_x;
        SensorSystem.navigation.position.y = new_y;
    }
}

/**
 * 计算到达目标位置需要的转向
 */
Direction_t Sensor_GetTurnDirection(GridPosition_t target)
{
    // 检查目标位置是否有效
    if (target.x < 0 || target.x >= GRID_SIZE || 
        target.y < 0 || target.y >= GRID_SIZE) {
        return DIRECTION_CENTER;
    }
    
    // 计算位置差
    int dx = target.x - SensorSystem.navigation.position.x;
    int dy = target.y - SensorSystem.navigation.position.y;
    
    // 如果已经到达目标位置
    if (dx == 0 && dy == 0) {
        return DIRECTION_CENTER;
    }
    
    // 根据当前朝向和目标位置计算需要的转向
    switch (SensorSystem.navigation.heading) {
        case HEADING_NORTH:
            if (dy < 0) return DIRECTION_CENTER;  // 已经朝向目标
            if (dx > 0) return DIRECTION_RIGHT;   // 需要向右转
            if (dx < 0) return DIRECTION_LEFT;    // 需要向左转
            return DIRECTION_RIGHT;               // 需要掉头，先右转
            
        case HEADING_EAST:
            if (dx > 0) return DIRECTION_CENTER;  // 已经朝向目标
            if (dy > 0) return DIRECTION_RIGHT;   // 需要向右转
            if (dy < 0) return DIRECTION_LEFT;    // 需要向左转
            return DIRECTION_RIGHT;               // 需要掉头，先右转
            
        case HEADING_SOUTH:
            if (dy > 0) return DIRECTION_CENTER;  // 已经朝向目标
            if (dx < 0) return DIRECTION_RIGHT;   // 需要向右转
            if (dx > 0) return DIRECTION_LEFT;    // 需要向左转
            return DIRECTION_RIGHT;               // 需要掉头，先右转
            
        case HEADING_WEST:
            if (dx < 0) return DIRECTION_CENTER;  // 已经朝向目标
            if (dy < 0) return DIRECTION_RIGHT;   // 需要向右转
            if (dy > 0) return DIRECTION_LEFT;    // 需要向左转
            return DIRECTION_RIGHT;               // 需要掉头，先右转
    }
    
    return DIRECTION_CENTER;  // 默认不转向
}

/**
 * 初始化火种位置信息
 */
void Sensor_InitFlamePoints(void)
{
    for(int i = 0; i < GRID_POINTS; i++) {
        SensorSystem.flame_manager.flames[i].point_id = 'A' + i;
        SensorSystem.flame_manager.flames[i].color = FLAME_NONE;
        SensorSystem.flame_manager.flames[i].is_collected = 0;
    }
}

/**
 * 设置火种位置
 */
void Sensor_SetFlamePoint(char id, FlameColor_t color)
{
    if(id >= 'A' && id <= 'O') {
        int index = id - 'A';
        SensorSystem.flame_manager.flames[index].color = color;
    }
}

/**
 * 设置队伍颜色
 */
void Sensor_SetTeamColor(int is_blue)
{
    SensorSystem.flame_manager.team_color = is_blue;
}

/**
 * 获取最近的未收集火种
 */
FlamePoint_t* Sensor_GetNearestFlame(void)
{
    FlamePoint_t* nearest = NULL;
    int min_distance = GRID_SIZE * 2;  // 最大可能距离
    
    for(int i = 0; i < GRID_POINTS; i++) {
        if(SensorSystem.flame_manager.flames[i].is_collected || 
           SensorSystem.flame_manager.flames[i].color == FLAME_NONE ||
           (SensorSystem.flame_manager.team_color && 
            SensorSystem.flame_manager.flames[i].color != FLAME_BLUE) ||
           (!SensorSystem.flame_manager.team_color && 
            SensorSystem.flame_manager.flames[i].color != FLAME_RED)) {
            continue;  // 跳过已收集、无火种或非本队颜色的位置
        }
        
        // 计算曼哈顿距离
        int flame_x = i % 3;  // 假设网格为3x5
        int flame_y = i / 3;
        int distance = abs(flame_x - SensorSystem.navigation.position.x) + 
                      abs(flame_y - SensorSystem.navigation.position.y);
        
        if(distance < min_distance) {
            min_distance = distance;
            nearest = &SensorSystem.flame_manager.flames[i];
        }
    }
    
    return nearest;
}

/**
 * 标记火种已被收集
 */
void Sensor_MarkFlameCollected(char point_id)
{
    if(point_id >= 'A' && point_id <= 'O') {
        int index = point_id - 'A';
        SensorSystem.flame_manager.flames[index].is_collected = 1;
        SensorSystem.flame_manager.total_flames_collected++;
    }
}

/**
 * 设置任务状态
 */
void Sensor_SetTaskState(TaskState_t state)
{
    SensorSystem.task_manager.task_state = state;
    SensorSystem.task_manager.retry_count = 0;  // 重置重试计数
}

/**
 * 获取任务状态
 */
TaskState_t Sensor_GetTaskState(void)
{
    return SensorSystem.task_manager.task_state;
}

/**
 * 设置错误状态
 */
void Sensor_SetErrorStatus(ErrorStatus_t error)
{
    SensorSystem.task_manager.error_status = error;
    if (error != ERROR_NONE) {
        SensorSystem.task_manager.retry_count++;
        SensorSystem.task_manager.last_error_time = HAL_GetTick();
    }
}

/**
 * 获取错误状态
 */
ErrorStatus_t Sensor_GetErrorStatus(void)
{
    return SensorSystem.task_manager.error_status;
}



/**
 * 重置错误状态
 */
void Sensor_ResetError(void)
{
    SensorSystem.task_manager.error_status = ERROR_NONE;
    SensorSystem.task_manager.retry_count = 0;
}

/**
 * 判断是否为十字路口
 * 通过检测白线传感器的数量来判断
 * @return 1:是十字路口 0:不是十字路口
 */
int Sensor_IsCrossing(void) {
    int white_count = 0;
    for(int i = 0; i < 8; i++) {
        if(SensorSystem.line_tracker.sensors[i] == WHITE_LINE) {
            white_count++;
        }
    }
    
    // 检测到足够多的白线即为十字路口
    return (white_count >= CROSS_SENSOR_THRESHOLD);
}

// ... 继续实现其他函数 ... 