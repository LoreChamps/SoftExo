#ifndef IO_header
#define IO_header

#include "Arduino.h"
#include "Definitions.h"
#include "Sensors.h"
#include "Controls.h"

// user input vars
extern String user_input;
extern int tmp, tmp1, tmp2; // user_input to int
extern int kilos;

void Print2Serial(struct data_sensors d_up_general);
void UserInput();

#endif