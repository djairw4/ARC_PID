#include "pid.h"
void pid_init(PID_t * pid, float p, float i, float d, int32_t dt_ms) {
pid->Kp = p;
pid->Ki = i;
pid->Kd = d;
pid->e_last = 0;
pid->e_sum = 0;
pid->dt_ms = dt_ms;
}
int32_t pid_calc(PID_t * pid, int32_t mv, int32_t dv) {
	int32_t p, i, d, e, u, e_diff;
    e = dv - mv;

    p = pid->Kp * e;
    if (p > 4095) p = 4095;
    else if (p < -4095) p = -4095;

    i = pid->dt_ms * pid->Ki * pid->e_sum / 1000;
    if (i > 4095) i = 4095;
    else if (i < -4095) i = -4095;
    pid->e_sum += e;

    e_diff = e - pid->e_last;
    d = pid->Kd * e_diff * 1000 / pid->dt_ms;
    if (d > 4095) d = 4095;
    else if (d < -4095) d = -4095;

    u = p + i + d;
    if (u > 4095) u = 4095;
    else if (u < 0) u = 0;

    pid->e_last = e;

    return u;
}
