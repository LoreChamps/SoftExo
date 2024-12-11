#ifndef Controls_header
#define Controls_header

#include "Arduino.h"
#include "Definitions.h"

// controls
extern int control_mode;
extern int control_mode_old;
extern float pref;
extern float pmeas;
extern float perr_th;
extern float perr;
extern uint32_t time_start;
extern uint32_t start_discharge;
extern bool press_autocontrol;
extern bool flag_control;
extern bool start_fatigue;
extern int current_cycle;
extern int needed_cycles;
extern int step_fatigue;
extern int counter_idle;
extern bool discharging;

// low-level pressure control
extern bool asymmetric_margin;
extern float Pressure_old;

// system states
extern uint8_t micro_state;
extern uint8_t micro_state_old;
extern uint8_t macro_state;
extern uint8_t macro_state_old;


void SetState(uint8_t state2set);
void ChangeState();
void PressureControl();
void Reset_Control_Var();
int IdleControl(int c);
void Inflate(int PUMP);
void Deflate(int VALVE);
void Seal();
void DischargePumps(uint32_t Tstart);

#endif