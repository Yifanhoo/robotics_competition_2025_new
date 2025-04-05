#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include "sensor_types.h"

// 基础函数
void Sensor_Init(void);                // 初始化传感器系统

// 循迹传感器相关函数
void Sensor_ReadLineTracker(void);     // 读取循迹传感器
int Sensor_GetLinePosition(void);      // 获取循迹位置
float Sensor_GetDeviation(void);       // 获取偏离中线的程度
TrackStatus_t Sensor_GetTrackStatus(void); // 获取当前循迹状态
int Sensor_IsAtCross(void);           // 检测是否在十字路口
void Sensor_ResetCrossCount(void);     // 重置十字路口计数
int Sensor_GetCrossCount(void);        // 获取经过的十字路口数

// 激光测距传感器相关函数
void Sensor_ReadDistance(void);        // 读取距离传感器
int Sensor_IsObjectDetected(void);     // 检测是否有物体
int Sensor_IsGrabbable(void);         // 检查是否可以抓取
int Sensor_IsObjectInRange(int min_dist, int max_dist); // 检查物体是否在指定范围内
int Sensor_GetOptimalGrabDistance(void);    // 获取最佳抓取距离
int Sensor_NeedAdjustPosition(void);        // 检查是否需要调整位置

// 导航相关函数
void Sensor_SetPosition(int x, int y);         // 设置当前位置
void Sensor_SetHeading(Heading_t heading);     // 设置当前朝向
GridPosition_t Sensor_GetPosition(void);       // 获取当前位置
Heading_t Sensor_GetHeading(void);            // 获取当前朝向
void Sensor_UpdatePosition(Direction_t turn);  // 根据转向更新位置和朝向
Direction_t Sensor_GetTurnDirection(GridPosition_t target); // 计算到达目标位置需要的转向

// 火种相关函数
void Sensor_InitFlamePoints(void);                    // 初始化火种位置信息
void Sensor_SetFlamePoint(char id, FlameColor_t color); // 设置火种位置
void Sensor_SetTeamColor(int is_blue);                // 设置队伍颜色
FlamePoint_t* Sensor_GetNearestFlame(void);          // 获取最近的未收集火种
void Sensor_MarkFlameCollected(char point_id);       // 标记火种已被收集

// 任务管理相关函数
void Sensor_SetTaskState(TaskState_t state);    // 设置任务状态
TaskState_t Sensor_GetTaskState(void);          // 获取任务状态
void Sensor_SetErrorStatus(ErrorStatus_t error); // 设置错误状态
ErrorStatus_t Sensor_GetErrorStatus(void);       // 获取错误状态
int Sensor_HandleError(void);                    // 处理错误
void Sensor_ResetError(void);                    // 重置错误状态

// 全局传感器系统实例
extern SensorSystem_t SensorSystem;

#endif /* SENSOR_H */ 