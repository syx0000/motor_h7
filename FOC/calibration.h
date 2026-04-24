#ifndef __CALIBRATION_H
#define __CALIBRATION_H

#include "stdint.h"



extern int32_t offset_lut[128];

void OrderPhases(void);
void Calibrate(void);

void PositionSensor_WriteLUT(int32_t new_lut[128]);

#endif
