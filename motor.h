#ifndef MOTOR_H
#define MOTOR_H

typedef struct
{
    float angle_ref;
    float angle_fdb;
    float speed_ref;
    float speed_fdb;
    float output;
}Motor_t;

extern Motor_t ChassisMotors[4];

#endif /* MOTOR_H */
