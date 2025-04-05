#include "sensor.h"
#include <stdlib.h>  // 为abs()函数添加头文件
#include <string.h> // 为memset函数添加头文件

// 传感器全局实例
Sensor_t Sensor;

// 循迹传感器状态定义
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

// 系统时间相关定义
#define SYSTEM_TICK_MS 10        // 系统时钟周期（毫秒）
#define TASK_DEFAULT_TIMEOUT 30000 // 默认任务超时时间（30秒）
#define ERROR_RECOVERY_TIMEOUT 5000 // 错误恢复超时时间（5秒）

// 安全相关定义
#define SAFE_DISTANCE_MIN 50      // 安全距离最小值（毫米）
#define SAFE_DISTANCE_MAX 500     // 安全距离最大值（毫米）
#define MAX_SPEED_FACTOR 1.0      // 最大速度因子
#define MIN_SPEED_FACTOR 0.3      // 最小速度因子

// 传感器状态定义
#define TASK_IDLE 0
#define TASK_GOTO_FLAME 1
#define TASK_GRAB_FLAME 2
#define TASK_GOTO_WALL 3
#define TASK_RETURN_BASE 4
#define TASK_PLACE_FLAME 5
#define TASK_RETURN_FLAME 6
#define TASK_END 7

// 火种位置到网格坐标的映射表
static const GridPosition_t flame_positions[GRID_POINTS] = {
    {0, 0}, // A
    {1, 0}, // B
    {2, 0}, // C
    {0, 1}, // D
    {1, 1}, // E
    {2, 1}, // F
    {0, 2}, // G
    {1, 2}, // H
    {2, 2}, // I
    {0, 3}, // J
    {1, 3}, // K
    {2, 3}, // L
    {3, 1}, // M
    {3, 2}, // N
    {3, 3}  // O
};


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
extern float Read_Battery(void);    // 读取电池电压
extern int Read_MotorStatus(void);  // 读取电机状态
extern int Read_GripperStatus(void);// 读取机械爪状态

// 外部函数声明
extern uint32_t Get_SystemTime(void);  // 获取系统时间（毫秒）
extern void Stop_AllMotors(void);      // 停止所有电机

/**
 * 传感器系统初始化
 */
void Sensor_Init(void)
{
    // 初始化循迹传感器数组
    for(int i = 0; i < 8; i++) {
        Sensor.sensor[i] = 0;
    }
    
    // 初始化其他参数
    Sensor.distance_mm = 0;
    Sensor.cross_count = 0;
    Sensor.last_turn = DIRECTION_CENTER;
    
    // 初始化位置和朝向
    Sensor.position.x = 0;
    Sensor.position.y = 0;
    Sensor.heading = HEADING_SOUTH;  // 假设初始朝向为向下
    
    // 初始化任务状态和火种信息
    Sensor.task_state = TASK_IDLE;
    Sensor_InitFlamePoints();
    Sensor.team_color = 0;  // 默认为红队
}

/**
 * 更新循迹传感器状态
 */
void Sensor_Read(void)
{
    // 从左到右读取8个传感器
    Sensor.sensor[0] = Read_IO1();
    Sensor.sensor[1] = Read_IO2();
    Sensor.sensor[2] = Read_IO3();
    Sensor.sensor[3] = Read_IO4();
    Sensor.sensor[4] = Read_IO5();
    Sensor.sensor[5] = Read_IO6();
    Sensor.sensor[6] = Read_IO7();
    Sensor.sensor[7] = Read_IO8();
}

/**
 * 更新超声波传感器数据
 */
void Sensor_ReadDistance(void)
{
    Sensor.distance_mm = Read_Ultrasonic();
}

/**
 * 获取当前循迹状态
 */
TrackStatus_t Sensor_GetTrackStatus(void)
{
    int active_count = 0;
    
    // 统计检测到白线的传感器数量
    for(int i = 0; i < 8; i++) {
        if(Sensor.sensor[i] == WHITE_LINE) {
            active_count++;
        }
    }
    
    // 状态判断逻辑
    if(active_count == 0) {
        return TRACK_STATUS_LOST;  // 没有传感器检测到线
    } else if(Sensor_IsCrossing()) {
        return TRACK_STATUS_CROSS; // 多个传感器检测到线，可能是十字路口
    }
    
    return TRACK_STATUS_NORMAL;    // 正常循迹状态
}

/**
 * 检测是否在十字路口
 */
int Sensor_IsAtCross(void)
{
    return (Sensor_GetTrackStatus() == TRACK_STATUS_CROSS);
}

/**
 * 检测是否到达终点
 */
int Sensor_IsAtEnd(void)
{
    return (Sensor_GetTrackStatus() == TRACK_STATUS_END);
}

/**
 * 计算当前循迹位置
 * @return 0:未检测到线 1-8:对应传感器位置
 */
int Sensor_GetLinePosition(void)
{
    int weighted_sum = 0;
    int count = 0;
    
    // 计算加权平均位置
    for(int i = 0; i < 8; i++) {
        if(Sensor.sensor[i] == WHITE_LINE) {
            weighted_sum += (i + 1);
            count++;
        }
    }
    
    if(count == 0)
        return 0;
    
    return weighted_sum / count;
}

/**
 * 计算偏离中线程度
 * @return 偏离值（负值:左偏 正值:右偏）
 */
float Sensor_GetDeviation(void)
{
    int position = Sensor_GetLinePosition();
    
    if(position == 0)
        return 0;
    
    // 相对于中心位置(4.5)的偏差
    return (float)(position - 4.5);
}

/**
 * 检测前方物体
 */
int Sensor_IsObjectDetected(void)
{
    return (Sensor.distance_mm < DISTANCE_DETECT_THRESHOLD) ? 1 : 0;
}

/**
 * 判断物体是否可抓取
 */
int Sensor_IsGrabbable(void)
{
    return (Sensor.distance_mm >= DISTANCE_GRAB_MIN && 
            Sensor.distance_mm <= DISTANCE_GRAB_MAX) ? 1 : 0;
}

/**
 * 重置十字路口计数
 */
void Sensor_ResetCrossCount(void)
{
    Sensor.cross_count = 0;
}

/**
 * 获取经过的十字路口数
 */
int Sensor_GetCrossCount(void)
{
    return Sensor.cross_count;
}

/**
 * 设置上一次转向方向
 */
void Sensor_SetLastTurn(Direction_t dir)
{
    Sensor.last_turn = dir;
}

/**
 * 获取上一次转向方向
 */
Direction_t Sensor_GetLastTurn(void)
{
    return Sensor.last_turn;
}

/**
 * 检查物体是否在指定距离范围内
 */
int Sensor_IsObjectInRange(int min_dist, int max_dist)
{
    return (Sensor.distance_mm >= min_dist && 
            Sensor.distance_mm <= max_dist) ? 1 : 0;
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
    return (abs(Sensor.distance_mm - Sensor_GetOptimalGrabDistance()) > POSITION_ADJUST_THRESHOLD);
}

/**
 * 获取位置调整方向
 */
Direction_t Sensor_GetAdjustDirection(void)
{
    if(Sensor.distance_mm <= 0) {
        return DIRECTION_CENTER;
    }
    
    int optimal = Sensor_GetOptimalGrabDistance();
    int diff = Sensor.distance_mm - optimal;
    
    if(abs(diff) <= POSITION_ADJUST_THRESHOLD) {
        return DIRECTION_CENTER;
    }
    return (diff > 0) ? DIRECTION_BACKWARD : DIRECTION_FORWARD;
}

/**
 * 设置当前位置
 */
void Sensor_SetPosition(int x, int y)
{
    if (Sensor_IsValidPosition(x, y)) {
        Sensor.position.x = x;
        Sensor.position.y = y;
    }
}

/**
 * 设置当前朝向
 */
void Sensor_SetHeading(Heading_t heading)
{
    if (heading >= HEADING_NORTH && heading <= HEADING_WEST) {
        Sensor.heading = heading;
    }
}

/**
 * 获取当前位置
 */
GridPosition_t Sensor_GetPosition(void)
{
    return Sensor.position;
}

/**
 * 获取当前朝向
 */
Heading_t Sensor_GetHeading(void)
{
    return Sensor.heading;
}

/**
 * 检查位置是否有效
 */
int Sensor_IsValidPosition(int x, int y)
{
    return (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE);
}

/**
 * 根据转向更新位置和朝向
 */
void Sensor_UpdatePosition(Direction_t turn)
{
    // 更新朝向
    Heading_t new_heading = Sensor.heading;
    if (turn == DIRECTION_LEFT) {
        new_heading = (Sensor.heading - 1) & 0x03;  // 循环减1
    } else if (turn == DIRECTION_RIGHT) {
        new_heading = (Sensor.heading + 1) & 0x03;  // 循环加1
    }
    Sensor.heading = new_heading;
    
    // 根据当前朝向更新位置
    int new_x = Sensor.position.x;
    int new_y = Sensor.position.y;
    
    switch (Sensor.heading) {
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
    if (Sensor_IsValidPosition(new_x, new_y)) {
        Sensor.position.x = new_x;
        Sensor.position.y = new_y;
    }
}

/**
 * 计算到达目标位置需要的转向
 * 使用简单的贪婪算法：优先调整到正确的x或y坐标
 */
Direction_t Sensor_GetTurnDirection(GridPosition_t target)
{
    // 检查目标位置是否有效
    if (!Sensor_IsValidPosition(target.x, target.y)) {
        return DIRECTION_CENTER;
    }
    
    // 计算位置差
    int dx = target.x - Sensor.position.x;
    int dy = target.y - Sensor.position.y;
    
    // 如果已经到达目标位置
    if (dx == 0 && dy == 0) {
        return DIRECTION_CENTER;
    }
    
    // 根据当前朝向和目标位置计算需要的转向
    switch (Sensor.heading) {
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
    memset(Sensor.flames, 0, sizeof(Sensor.flames));
    for(int i = 0; i < GRID_POINTS; i++) {
        Sensor.flames[i].point_id = 'A' + i;
        Sensor.flames[i].color = FLAME_NONE;
        Sensor.flames[i].is_collected = 0;
    }
}

/**
 * 设置火种位置
 */
void Sensor_SetFlamePoint(char id, FlameColor_t color)
{
    if(id >= 'A' && id <= 'O') {
        int index = id - 'A';
        Sensor.flames[index].color = color;
    }
}

/**
 * 设置队伍颜色
 */
void Sensor_SetTeamColor(int is_blue)
{
    Sensor.team_color = is_blue;
}

/**
 * 检查火种位置是否有效
 */
int Sensor_IsValidFlamePoint(char point_id)
{
    return (point_id >= 'A' && point_id <= 'O');
}

/**
 * 标记火种已被收集
 */
void Sensor_MarkFlameCollected(char point_id)
{
    if(Sensor_IsValidFlamePoint(point_id)) {
        int index = point_id - 'A';
        Sensor.flames[index].is_collected = 1;
    }
}

/**
 * 将火种位置转换为网格坐标
 */
GridPosition_t Sensor_FlamePointToGrid(char point_id)
{
    GridPosition_t pos = {0, 0};
    if(Sensor_IsValidFlamePoint(point_id)) {
        int index = point_id - 'A';
        pos = flame_positions[index];
    }
    return pos;
}

/**
 * 获取最近的未收集火种
 * 使用曼哈顿距离计算最近的火种
 */
FlamePoint_t* Sensor_GetNearestFlame(void)
{
    FlamePoint_t* nearest = NULL;
    int min_distance = GRID_SIZE * 2;  // 最大可能距离
    
    for(int i = 0; i < GRID_POINTS; i++) {
        if(Sensor.flames[i].is_collected || 
           Sensor.flames[i].color == FLAME_NONE ||
           (Sensor.team_color && Sensor.flames[i].color != FLAME_BLUE) ||
           (!Sensor.team_color && Sensor.flames[i].color != FLAME_RED)) {
            continue;  // 跳过已收集、无火种或非本队颜色的位置
        }
        
        GridPosition_t flame_pos = flame_positions[i];
        int distance = abs(flame_pos.x - Sensor.position.x) + 
                      abs(flame_pos.y - Sensor.position.y);
        
        if(distance < min_distance) {
            min_distance = distance;
            nearest = &Sensor.flames[i];
        }
    }
    
    return nearest;
}

/**
 * 规划从起点到目标的路径
 * 使用简单的A*算法
 */
Path_t Sensor_PlanPath(GridPosition_t start, GridPosition_t target)
{
    Path_t path = {0};
    
    // 检查起点和终点的有效性
    if (!Sensor_IsValidPosition(start.x, start.y) || 
        !Sensor_IsValidPosition(target.x, target.y)) {
        return path;
    }
    
    // 简单路径规划：先在x方向移动，再在y方向移动
    int curr_x = start.x;
    int curr_y = start.y;
    int idx = 0;
    
    // 添加x方向的路径点
    while (curr_x != target.x && idx < GRID_SIZE * GRID_SIZE) {
        GridPosition_t pos = {curr_x, curr_y};
        path.path[idx++] = pos;
        curr_x += (target.x > curr_x) ? 1 : -1;
    }
    
    // 添加y方向的路径点
    while (curr_y != target.y && idx < GRID_SIZE * GRID_SIZE) {
        GridPosition_t pos = {curr_x, curr_y};
        path.path[idx++] = pos;
        curr_y += (target.y > curr_y) ? 1 : -1;
    }
    
    // 添加终点
    if (idx < GRID_SIZE * GRID_SIZE) {
        path.path[idx++] = target;
    }
    
    path.length = idx;
    return path;
}

/**
 * 检查路径是否有效
 */
int Sensor_IsPathValid(Path_t path)
{
    if (path.length <= 0 || path.length > GRID_SIZE * GRID_SIZE) {
        return 0;
    }
    
    // 检查路径中的每个点是否有效
    for (int i = 0; i < path.length; i++) {
        if (!Sensor_IsValidPosition(path.path[i].x, path.path[i].y)) {
            return 0;
        }
    }
    
    return 1;
}

/**
 * 设置当前路径
 */
void Sensor_SetCurrentPath(Path_t path)
{
    if (Sensor_IsPathValid(path)) {
        Sensor.current_path = path;
    }
}

/**
 * 获取当前路径
 */
Path_t Sensor_GetCurrentPath(void)
{
    return Sensor.current_path;
}

/**
 * 获取下一个移动方向
 */
Direction_t Sensor_GetNextPathDirection(void)
{
    if (Sensor.current_path.length <= 1) {
        return DIRECTION_CENTER;
    }
    
    // 获取当前位置在路径中的索引
    int current_idx = 0;
    for (int i = 0; i < Sensor.current_path.length; i++) {
        if (Sensor.current_path.path[i].x == Sensor.position.x &&
            Sensor.current_path.path[i].y == Sensor.position.y) {
            current_idx = i;
            break;
        }
    }
    
    // 如果已经是最后一个点，则返回中心
    if (current_idx >= Sensor.current_path.length - 1) {
        return DIRECTION_CENTER;
    }
    
    // 获取下一个点
    GridPosition_t next = Sensor.current_path.path[current_idx + 1];
    return Sensor_GetTurnDirection(next);
}

/**
 * 初始化任务信息
 */
void Sensor_InitTask(TaskState_t state, TaskPriority_t priority, uint32_t timeout)
{
    Sensor.task_info.state = state;
    Sensor.task_info.priority = priority;
    Sensor.task_info.start_time = Get_SystemTime();
    Sensor.task_info.timeout = timeout ? timeout : TASK_DEFAULT_TIMEOUT;
    Sensor.task_info.retry_limit = MAX_RETRY_COUNT;
}

/**
 * 检查任务是否超时
 */
int Sensor_IsTaskTimeout(void)
{
    uint32_t current_time = Get_SystemTime();
    return (current_time - Sensor.task_info.start_time) >= Sensor.task_info.timeout;
}

/**
 * 更新任务状态
 */
void Sensor_UpdateTaskStatus(void)
{
    // 检查任务超时
    if (Sensor_IsTaskTimeout()) {
        Sensor_SetErrorStatus(ERROR_TASK_TIMEOUT);
        return;
    }
    
    // 更新任务进度
    switch (Sensor.task_info.state) {
        case TASK_GOTO_FLAME:
            // 计算到目标的距离作为进度
            break;
            
        case TASK_GRAB_FLAME:
            // 根据抓取状态更新进度
            break;
            
        // ... 其他任务状态处理 ...
    }
}

/**
 * 获取任务剩余时间（毫秒）
 */
int Sensor_GetRemainingTime(void)
{
    uint32_t current_time = Get_SystemTime();
    uint32_t elapsed = current_time - Sensor.task_info.start_time;
    return (elapsed < Sensor.task_info.timeout) ? 
           (Sensor.task_info.timeout - elapsed) : 0;
}

/**
 * 获取任务进度（0-100）
 */
int Sensor_GetTaskProgress(void)
{
    switch (Sensor.task_info.state) {
        case TASK_IDLE:
            return 100;
            
        case TASK_GOTO_FLAME: {
            FlamePoint_t* target = Sensor_GetNearestFlame();
            if (!target) return 0;
            
            GridPosition_t target_pos = Sensor_FlamePointToGrid(target->point_id);
            int total_distance = GRID_SIZE * 2;  // 最大可能距离
            int current_distance = abs(target_pos.x - Sensor.position.x) + 
                                 abs(target_pos.y - Sensor.position.y);
            
            return ((total_distance - current_distance) * 100) / total_distance;
        }
        
        // ... 其他任务状态的进度计算 ...
        
        default:
            return 0;
    }
}

/**
 * 检查错误是否可恢复
 */
int Sensor_IsErrorRecoverable(ErrorStatus_t error)
{
    switch (error) {
        case ERROR_NONE:
        case ERROR_LOST_LINE:
        case ERROR_OBSTACLE:
            return 1;
            
        case ERROR_INVALID_POSITION:
        case ERROR_GRAB_FAIL:
            return (Sensor.retry_count <= Sensor.task_info.retry_limit);
            
        default:
            return 0;
    }
}

/**
 * 记录错误信息
 */
void Sensor_LogError(ErrorStatus_t error, const char* description)
{
    Sensor.last_error_time = Get_SystemTime();
    Sensor_SetErrorStatus(error);
    // 这里可以添加错误日志记录功能
}

/**
 * 获取自上次错误以来的时间（毫秒）
 */
uint32_t Sensor_GetTimeSinceLastError(void)
{
    return Get_SystemTime() - Sensor.last_error_time;
}

/**
 * 获取系统运行时间（毫秒）
 */
uint32_t Sensor_GetOperationTime(void)
{
    return Get_SystemTime();
}

/**
 * 重置统计信息
 */
void Sensor_ResetStatistics(void)
{
    Sensor.total_flames_collected = 0;
    Sensor.operation_time = 0;
    Sensor.last_error_time = 0;
    Sensor.retry_count = 0;
}

/**
 * 检查是否安全移动
 */
int Sensor_IsSafeToMove(void)
{
    // 检查障碍物
    if (Sensor.distance_mm < SAFE_DISTANCE_MIN) {
        return 0;
    }
    
    // 检查错误状态
    if (Sensor.error_status != ERROR_NONE && 
        !Sensor_IsErrorRecoverable(Sensor.error_status)) {
        return 0;
    }
    
    return 1;
}

/**
 * 检查是否安全抓取
 */
int Sensor_IsSafeToGrab(void)
{
    return (Sensor_IsGrabbable() && 
            Sensor.error_status == ERROR_NONE);
}

/**
 * 检查是否需要紧急停止
 */
int Sensor_IsEmergencyStop(void)
{
    return (Sensor.error_status >= ERROR_CRITICAL);
}

/**
 * 执行紧急停止
 */
void Sensor_EmergencyStop(void)
{
    Stop_AllMotors();
    Sensor_SetErrorStatus(ERROR_EMERGENCY_STOP);
    // 可以添加其他紧急停止操作
}

/**
 * 执行当前任务
 */
TaskResult_t Sensor_ExecuteTask(void)
{
    switch (Sensor.task_state) {
        case TASK_IDLE:
            return TASK_RESULT_SUCCESS;
            
        case TASK_GOTO_FLAME: {
            FlamePoint_t* target = Sensor_GetNearestFlame();
            if (!target) {
                return TASK_RESULT_FAILED;
            }
            
            GridPosition_t target_pos = Sensor_FlamePointToGrid(target->point_id);
            Direction_t turn = Sensor_GetTurnDirection(target_pos);
            
            if (turn == DIRECTION_CENTER) {
                Sensor.task_state = TASK_GRAB_FLAME;
            }
            return TASK_RESULT_IN_PROGRESS;
        }
        
        case TASK_GRAB_FLAME:
            if (Sensor_IsGrabbable()) {
                // 抓取逻辑由外部实现
                Sensor.task_state = TASK_GOTO_WALL;
                return TASK_RESULT_SUCCESS;
            }
            return TASK_RESULT_IN_PROGRESS;
            
        // ... 其他任务状态的处理 ...
        
        default:
            return TASK_RESULT_FAILED;
    }
}

/**
 * 设置任务状态
 */
void Sensor_SetTaskState(TaskState_t state)
{
    Sensor.task_state = state;
    Sensor.retry_count = 0;  // 重置重试计数
}

/**
 * 获取当前任务状态
 */
TaskState_t Sensor_GetTaskState(void)
{
    return Sensor.task_state;
}

/**
 * 检查任务是否完成
 */
int Sensor_IsTaskCompleted(void)
{
    return (Sensor.task_result == TASK_RESULT_SUCCESS);
}

/**
 * 设置错误状态
 */
void Sensor_SetErrorStatus(ErrorStatus_t error)
{
    Sensor.error_status = error;
    if (error != ERROR_NONE) {
        Sensor.retry_count++;
    }
}

/**
 * 获取错误状态
 */
ErrorStatus_t Sensor_GetErrorStatus(void)
{
    return Sensor.error_status;
}

/**
 * 处理错误
 * 返回1表示错误已处理，0表示需要人工干预
 */
int Sensor_HandleError(void)
{
    if (Sensor.error_status == ERROR_NONE) {
        return 1;
    }
    
    // 如果超过最大重试次数，需要人工干预
    if (Sensor.retry_count > MAX_RETRY_COUNT) {
        return 0;
    }
    
    switch (Sensor.error_status) {
        case ERROR_LOST_LINE:
            // 尝试找回线
            return 1;
            
        case ERROR_OBSTACLE:
            // 尝试绕过障碍物
            return 1;
            
        case ERROR_INVALID_POSITION:
            // 尝试重新定位
            return 1;
            
        case ERROR_GRAB_FAIL:
            // 尝试重新抓取
            return 1;
            
        default:
            return 0;
    }
}

/**
 * 重置错误状态
 */
void Sensor_ResetError(void)
{
    Sensor.error_status = ERROR_NONE;
    Sensor.retry_count = 0;
}

/**
 * 获取电池电压
 */
float Sensor_GetBatteryVoltage(void)
{
    return Read_Battery();
}

/**
 * 判断是否为十字路口
 */
int Sensor_IsCrossing(void) {
    int white_count = 0;
    for(int i = 0; i < 8; i++) {
        if(Sensor.sensor[i] == WHITE_LINE) {
            white_count++;
        }
    }
    
    // 检测到足够多的白线即为十字路口
    return (white_count >= CROSS_SENSOR_THRESHOLD);
}