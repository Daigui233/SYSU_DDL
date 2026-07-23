#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    bool valid;
} Attitude;

bool attitude_init(void);
void attitude_update(float dt_seconds);
const Attitude *attitude_get(void);

#endif
