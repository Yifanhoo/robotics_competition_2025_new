#include <stdio.h>

#include "alg.h"

float limit( float input , float upper_limit_value , float lower_limit_value )
{
	if(input > upper_limit_value)
		return upper_limit_value ;
	else if(input < lower_limit_value)
		return lower_limit_value ;
	else
		return input ;
}

/**
* @brief      斜坡控制
* @param      无
* @retval     无
*/
float slope_ctrl(float rawref, float targetref, float acc, float dec)
{    
    float newref;
    if (rawref > 0) {
        if (rawref < targetref - acc)
            newref = rawref + acc;
        else if (rawref > targetref + dec) 
            newref = rawref - dec;
        else 
            newref = targetref;
    }
    else {
        if (rawref > targetref + acc) 
            newref = rawref - acc;
        else if (rawref < targetref - dec) 
            newref = rawref + dec;
        else 
            newref = targetref;
    }
    return newref;
}

/**
 * @brief 	PID setup function
 * @param 	None
 * @retval	None
 * @note	None
 */
void PID_ParamInit(PID_PIDParamTypeDef* pparam, float kp, float ki, float kd, float sum_max, float output_max) {
    pparam->kp = kp;
    pparam->ki = ki;
    pparam->kd = kd;
    pparam->sum_max = sum_max;
    pparam->output_max = output_max;
}

void PID_ArrayParamInit(PID_PIDParamTypeDef* pparam,float param[5]){
	PID_ParamInit(pparam,param[0],param[1],param[2],param[3],param[4]);
}
	
float PID_GetRef(PID_PIDTypeDef* pid) {
    return pid->ref;
}

void PID_SetRef(PID_PIDTypeDef* pid, float ref) {
    pid->ref = ref;
}

void PID_AddRef(PID_PIDTypeDef* pid, float inc) {
    pid->ref += inc;
}


float PID_GetFdb(PID_PIDTypeDef* pid) {
    return pid->fdb;
}

void PID_SetFdb(PID_PIDTypeDef* pid, float fdb) {
    pid->fdb = fdb;
}

float PID_GetOutput(PID_PIDTypeDef* pid) {
    return pid->output;
}

void PID_SetOutput(PID_PIDTypeDef* pid,float value){
	pid->output = value;
}

void PID_ClearData(PID_PIDTypeDef* pid) {
	pid->ref    = 0;
	pid->fdb    = 0;
	pid->err 	= 0;
	pid->err_last = 0;
	pid->sum    = 0;
	pid->output = 0;
}

void PID_Calc(PID_PIDTypeDef* pid, PID_PIDParamTypeDef* pparam) {
	float dError,Error;
	
	Error = pid->ref - pid->fdb;		//计算误差
	pid->sum = pid->sum + Error;		//累积误差
	pid->err_last = pid->err;			//更新结构体历史误差
	pid->err = Error;					//更新结构体误差
	dError = pid->err - pid->err_last;	//计算微分
	
	//积分限幅
	pid->sum = limit(pid->sum, pparam->sum_max,-pparam->sum_max);

	//计算输出
	pid->output = pparam->kp * Error + pparam->ki * pid->sum + pparam->kd * dError;            

	//输出限幅
	pid->output = limit(pid->output, pparam->output_max, -pparam->output_max);
}