
/* pid.h */

#ifndef INC_PID_H_
#define INC_PID_H_

// Globális PID példány minden motorhoz
//PIDController pid_m1, pid_m2, pid_m3, pid_m4;

// PID controller struktúra
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
    float output;
    float min_output;
    float max_output;
} PIDController;

void PID_Init(PIDController* pid, float Kp, float Ki, float Kd, float min, float max);
float PID_Compute(PIDController* pid, float setpoint, float measurement, float dt);


#endif /* INC_PID_H_ */
