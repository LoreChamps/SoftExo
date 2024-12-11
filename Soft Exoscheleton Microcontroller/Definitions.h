#ifndef Definitions_header
#define Definitions_header


// --------- PINOUT --------- //

// sensors pins
#define PRESSURE_SENSOR1 A2
#define PRESSURE_SENSOR2 A3
#define EMG1_OUTPUT_PIN A4
#define EMG2_OUTPUT_PIN A5

// pinout
#define PUMP_1 9
#define PUMP_2 5
#define VALVE_1 12
#define VALVE_2 6
#define VALVE_DISCHARGE 10


// --------- CALIBRATION --------- //

// sensors plugged
#define SENSORS_INCLUDED true // true if included | false if not
#define IMU_PLUGGED true // true if included | false if not

// low level control
#define ASYMMETRIC_GAIN 1.15 // --> commento

// fatigue cycle
#define TIME_REST_HP 3000 // [ms] time @ high pressure in the cycle
#define TIME_REST_LP 5000 // [ms] time @ low pressure in the cycle
#define STOP_FATIGUE 15000 // [ms] stop criterion based on bursting detection
#define ERROR_FATIGUE 0.5 // [psi] pressure error band for fatigue cycle

// idle control
#define COUNTER_IDLE_LIM 300 // counter to vent when turning in idle mode
#define PRESS_IDLE_LIM 0 // psi  --> commento

// others
#define FREQ 100 // sampling frequency in Hz (su Feather M4 Exrpess sono riuscito a fare max 200Hz, ho settato 100Hz per avere margine)

// --------- SYSTEM STATES --------- //

// system states macro-states
#define MACRO_INFLATING 11
#define MACRO_DEFLATING 12
#define MACRO_SEALING 13

// system states micro-states
#define MICRO_INFLATE_P1 21
#define MICRO_INFLATE_P2 22
#define MICRO_INFLATE_PP 23
#define MICRO_DEFLATE_V1 24
#define MICRO_DEFLATE_V2 25
#define MICRO_DEFLATE_VV 26
#define MICRO_SEAL 27

#endif