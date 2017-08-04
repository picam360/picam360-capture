#pragma once
#include "picam360_capture_plugin.h"

void create_manual_mpu(MPU_T **mpu);
void manual_mpu_set(MPU_T *mpu, float x_deg, float y_deg, float z_deg);
