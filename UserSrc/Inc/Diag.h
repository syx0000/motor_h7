#ifndef _DIAG_H
#define _DIAG_H
#include "stdint.h"

extern float a_cur[3],b_cur[3],c_cur[3],DC_bus[3],temperature_MOS[3],temperature_MOTOR[3];
extern float a_cur_record,b_cur_record,c_cur_record,DC_bus_record,temperature_MOS_record,temperature_MOTOR_record;
extern uint8_t Report_HWOvercur;
extern float TEMPERATURE_MOSFET,TEMPERATURE_MOTOR;
void errorDiag(void);
void errorReport(void);

#endif

