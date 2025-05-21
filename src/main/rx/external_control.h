#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct externalControlCommand_s {
    float roll_rate;    // Desired roll rate (e.g., deg/s)
    float pitch_rate;   // Desired pitch rate (e.g., deg/s)
    float yaw_rate;     // Desired yaw rate (e.g., deg/s)
    float thrust;       // Desired thrust (e.g., 0.0 to 1.0, or raw PWM value)
} externalControlCommand_t;

struct rxConfig_s; // Forward declaration
struct rxRuntimeState_s; // Forward declaration

bool externalControlInit(const struct rxConfig_s *rxConfig, struct rxRuntimeState_s *rxRuntimeState);
const externalControlCommand_t *getExternalControlCommand(void);
void externalControlSetCommand(const externalControlCommand_t* newCmd);
