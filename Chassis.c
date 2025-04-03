#include "Chassis.h"
#include "alg.h"
#include "motor.h"
#include "sensor.h"

#define CONST_CHASSIS_FORWARDBACK_SPEED_MAX     1 //chassis 前后最快速度
#define CONST_CHASSIS_LEFTRIGHT_SPEED_MAX       1 //chassis 左右最快速度
#define CONST_CHASSIS_ROTATE_SPEED_MAX          1    //chassis 旋转最快速度

Chassis_t Chassis;

float gyro1 = 1.04f;
float gyro2 = 1.04f;
float gyro3 = 0.86f;
float gyro4 = 0.86f;

float chassis_anglePIDparram[5] = {0, 0, 0, 0, 0};
float chassis_speedPIDparram[5] = {0, 0, 0, 0, 0};


void Chassis_Init();
void Chassis_Task();
void Chassis_Control();
void Chassis_Output();

void Chassis_SetSpeedRef(Motor_t *motor, float ref);
void Chassis_SetAngleRef(Motor_t *motor, float ref);
void Chassis_SetOutput(Motor_t *motor, float output);
void Chassis_SetForwardBackRef(float ref);
void Chassis_SetLeftRightRef(float ref);
void Chassis_SetRotateRef(float ref);
void Chassis_SetStopRef();
void Chassis_ClearChassisRef(Chassis_Ref_t *pref);
void Chassis_CalcMecanumRef();
void Chassis_Move();
void Chassis_Turn();

// 如果没有定义 HAL_TICK 函数，添加一个简单实现
#ifndef HAL_TICK_DEFINED
#define HAL_TICK_DEFINED
unsigned int HAL_TICK(void)
{
    // 返回系统时钟，实际项目中应替换为实际的时钟函数
    // 例如，在 STM32 上可以使用 HAL_GetTick()
    return 0;
}
#endif

void Chassis_Init()
{
    Chassis_t *chassis = &Chassis;
    chassis->control_state  = 1;
    chassis->output_state   = 1;
    chassis->pending_state = 0;

    chassis->act    = STOP;
    chassis->raw_ref.front_back = 0;
    chassis->raw_ref.left_right = 0;
    chassis->raw_ref.rotate     = 0;
    chassis->last_raw_ref.front_back = 0;
    chassis->last_raw_ref.left_right = 0;
    chassis->last_raw_ref.rotate     = 0;
    chassis->moving_speed = 300;

    PID_ClearData(&chassis->angle_pid);
    PID_ClearData(&chassis->speed_pid);
    PID_ArrayParamInit(&chassis->angle_pidparram, chassis_anglePIDparram);
    PID_ArrayParamInit(&chassis->speed_pidparram, chassis_speedPIDparram);
}

void Chassis_Task()
{
    Chassis_t *chassis = &Chassis;
    chassis->pending_state = 1;
    Chassis_Control();
    Chassis_Output();
    chassis->pending_state = 0;
}

void Chassis_Control()
{
    Chassis_t *chassis = &Chassis;

    if(chassis->control_state != 1)
        return;

    switch (chassis->act)
    {
    case STOP:
        Chassis_SetStopRef();
        break;
    case MOVE:
        Chassis_Move();
        Chassis_SetForwardBackRef(chassis->moving_speed);
        Chassis_SetRotateRef(chassis->rotate_speed);
        break;
    case TURN:
        Chassis_Turn();
        Chassis_SetForwardBackRef(chassis->moving_speed);
        Chassis_SetRotateRef(chassis->rotate_speed);
        break;
    case GRAB:
        /* code */
        break;
    case PUT:
        /* code */
        break;
    
    default:
        break;
    }

    Chassis_CalcMecanumRef();
}

void Chassis_Output()
{
    Chassis_t *chassis = &Chassis;
    if(chassis->output_state != 1)
        return;
        
    //四个motor 的 pwm output
}

int sense_intersection_p();
void Follow_line();

void Follow_line()
{
    Chassis_t *chassis = &Chassis;
    
    if(sense_intersection_p())
        return;
        
    // 使用传感器模块提供的偏差值
    float deviation = Sensor_GetDeviation();
    
    // 设置 PID 控制器参数
    PID_SetRef(&chassis->angle_pid, 0);  // 目标是无偏差（居中）
    PID_SetFdb(&chassis->angle_pid, deviation);
    PID_Calc(&chassis->angle_pid, &chassis->angle_pidparram);
    
    // 将 PID 输出设置为旋转速度
    chassis->rotate_speed = chassis->angle_pid.output;
}

int sensor_count = 2;
int over_point = 0;
int intersection_point = 0;
int on_point_flag = 0;

int task_list[20];
int i = 0;
void Chassis_Move()
{
    Chassis_t *chassis = &Chassis;
    //走直线判断
        //交点判断

    //跟线走 用pid 算rotate的速度ref
    Follow_line();

    //when检测到在交点上的时候，
        //if 现在还要继续move forward，我给rotate的速度ref = 0，等过了这个交点再用Follow_line()的rotate ref
        //else if 已经要turn or stop or grab等等，i++代表要跳到下一个任务
    if(sense_intersection_p() && on_point_flag == 0 && HAL_TICK()-chassis->last_time >= 100)
    {   
        chassis->last_time = HAL_TICK();
        on_point_flag = 1;

        if(over_point != intersection_point)
        {
            over_point++;
            chassis->rotate_speed = 0;
        }
        else if(over_point == intersection_point)
        {
            over_point = 0;
            i++;
            chassis->act = task_list[i];
        }
    }
    else
    {
        on_point_flag = 0;
    }
        

}


void Chassis_SetSpeedRef(Motor_t *motor, float ref)
{
    motor->speed_ref = ref;
}
void Chassis_SetAngleRef(Motor_t *motor, float ref)
{
    motor->angle_ref = ref;
}
void Chassis_SetOutput(Motor_t *motor, float output)
{
    motor->output = output;
}

void Chassis_SetForwardBackRef(float ref)
{
	Chassis_t *chassis = &Chassis;
	chassis->last_raw_ref.front_back = chassis->raw_ref.front_back;

	chassis->raw_ref.front_back = limit(ref, CONST_CHASSIS_FORWARDBACK_SPEED_MAX, -CONST_CHASSIS_FORWARDBACK_SPEED_MAX);
}

void Chassis_SetLeftRightRef(float ref)
{
	Chassis_t *chassis = &Chassis;
	chassis->last_raw_ref.left_right = chassis->raw_ref.left_right;

	chassis->raw_ref.left_right = limit(ref, CONST_CHASSIS_LEFTRIGHT_SPEED_MAX, -CONST_CHASSIS_LEFTRIGHT_SPEED_MAX);
}

void Chassis_SetRotateRef(float ref)
{
	Chassis_t *chassis = &Chassis;
	chassis->last_raw_ref.rotate = chassis->raw_ref.rotate;
	chassis->raw_ref.rotate = limit(ref, CONST_CHASSIS_ROTATE_SPEED_MAX, -CONST_CHASSIS_ROTATE_SPEED_MAX);
}

void Chassis_ClearChassisRef(Chassis_Ref_t *pref)
{
	pref->front_back = 0;
	pref->left_right = 0;
	pref->rotate = 0;
}

void Chassis_SetStopRef()
{
	Chassis_t *chassis = &Chassis;
	Chassis_SetForwardBackRef(0);
	Chassis_SetLeftRightRef(0);
	Chassis_SetRotateRef(0);
	Chassis_ClearChassisRef(&(chassis->raw_ref));
}

void Chassis_CalcMecanumRef()
{
    Chassis_t *chassis = &Chassis;

    float motor_realspeed[4];

    //if motor 顺时转 为+ ， if not 要调整正负号
    //从上往下看，头超前，左前为0，逆时顺序计数
    //frontback front=+ ； leftright right=+ ； rotate 逆时=+
    motor_realspeed[0] = -chassis->raw_ref.front_back + chassis->raw_ref.left_right +chassis->raw_ref.rotate;
    motor_realspeed[1] = -chassis->raw_ref.front_back - chassis->raw_ref.left_right +chassis->raw_ref.rotate;
    motor_realspeed[2] = +chassis->raw_ref.front_back - chassis->raw_ref.left_right +chassis->raw_ref.rotate;
    motor_realspeed[3] = +chassis->raw_ref.front_back + chassis->raw_ref.left_right +chassis->raw_ref.rotate;

    chassis->motors[0].speed_ref = motor_realspeed[0];
    chassis->motors[1].speed_ref = motor_realspeed[1];
    chassis->motors[2].speed_ref = motor_realspeed[2];
    chassis->motors[3].speed_ref = motor_realspeed[3];
}
