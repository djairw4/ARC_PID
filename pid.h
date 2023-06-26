#ifndef PID_H_
#define PID_H_
#include <stdint.h>
typedef struct {
float Kp;
float Ki;
float Kd;
int32_t e_last;
int32_t e_sum;
int32_t dt_ms;
}PID_t;
void pid_init(PID_t * pid, float p, float i, float d, int32_t dt_ms);
int32_t pid_calc(PID_t * pid, int32_t mv, int32_t dv);
#endif /* PID_H_ */
