#ifndef CHASSIS_H
#define CHASSIS_H

#include "sensor.h"
#include "alg.h"
#include "motor.h"

typedef enum
{
    STOP    = 0,
    MOVE    = 1,
    TURN_RIGHT    = 2,
    TURN_LEFT     = 3,
    GRAB    = 4,
    PUT     = 5,
    END     = 6,

}Chassis_Action;

typedef struct
{
    float left_right;    //左右
    float front_back; 	//前后
    float rotate; 		//旋转
}Chassis_Ref_t;


typedef struct
{
    Chassis_Action act;
    Chassis_Ref_t raw_ref;
    Chassis_Ref_t last_raw_ref;
    // Chassis_Ref_t raw_speed_ref;
    Motor_t motors[4];
    PID_PIDTypeDef rotate_angle_pid;
    PID_PIDTypeDef moving_distance_pid;
    PID_PIDParamTypeDef rotate_angle_pidparram;
    PID_PIDParamTypeDef moving_distace_pidparram;
    char control_state;
    char output_state;
    char pending_state;
    float moving_speed;     //move forward恒定速度
    float rotate_speed;
    int task_i;
    float last_time;

}Chassis_t;


extern Chassis_t Chassis;
extern Chassis_Action Task_list[100]; 

#endif /* CHASSIS_H */
