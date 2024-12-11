#include "EMGFilters.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MadgwickAHRS.h>
#include <math.h>
#include <String.h>
#include "DFRobot_BMI160.h"
#include "MathFunctions.h"  

#include "Definitions.h"
#include "IO.h"
#include "Sensors.h"
#include "Controls.h"

// BASIC RULES.
// in .h include all the libraries needed
// in .cpp only include the associated .h file
// ifndef should be in .h files to avoid multiple inclusions 
// extern variables in .h files, to be used in .cpp files
// struct definitions in .h files plus extern struct in .h, to be used in .cpp files

// sample frequency
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 1000/FREQ; // milliseconds 

void SetupPins(){
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(PUMP_1, OUTPUT);
  digitalWrite(PUMP_1, LOW);
  pinMode(PUMP_2, OUTPUT);
  digitalWrite(PUMP_2, LOW);
  pinMode(VALVE_1, OUTPUT);
  digitalWrite(VALVE_1, LOW);
  pinMode(VALVE_2, OUTPUT);
  digitalWrite(VALVE_2, LOW);
  pinMode(VALVE_DISCHARGE, OUTPUT);
  digitalWrite(VALVE_DISCHARGE, LOW);
}


// --------------------------------------------------- //

void setup(){
  Serial.setTimeout(5); // ms, remove delay in serial communication between feather and pyqt (default 1000ms)
  Serial.begin(9600);
  while(!Serial);
  SetupPins();
  if(SENSORS_INCLUDED){
    SetupSensors();
    SetupArmEstimation(RM1,RM2,V1,V2,arm);
  }
  startMillis = millis();  //initial start time
}



// --------------------------------------------------- //

void loop(){
  currentMillis = millis();
  if (currentMillis - startMillis >= period)  // test running frequency
  {
    UserInput(); // execute command line user inputs
    GetPressure(d_up); // reading pressure value
    if(SENSORS_INCLUDED){
      GetSensorsData(d_up); // retreives data from all the sensors
      ArmEstimation(arm, d_up); // computes arm flexion and elevation
    }

    // idle control if in TAB:"Automatic" but no options selected (Pressure Control, Fatigue Test, ...)
    if(control_mode==0 && counter_idle<COUNTER_IDLE_LIM){counter_idle=IdleControl(counter_idle);}

    // // Pressure filtering
    // // different pressure filter for inflation and deflation 
    // if(macro_state == MACRO_INFLATING){
    //   pmeas = ALPHA_PRESSURE_IN*d_up.pressure1 + (1-ALPHA_PRESSURE_IN)*Pressure_old;
    // }
    // else{
    //   pmeas = ALPHA_PRESSURE_OUT*d_up.pressure1 + (1-ALPHA_PRESSURE_OUT)*Pressure_old;
    // }
    // same pressure filter for inflation and deflation 
    pmeas = ALPHA_PRESSURE*d_up.pressure1 + (1-ALPHA_PRESSURE)*Pressure_old;

    // Automatic control
    if(flag_control){PressureControl();}

    // changing the state if changes have happened
    if(micro_state != micro_state_old){ChangeState();}

    // changing in MACRO state
    if(macro_state_old == MACRO_INFLATING && macro_state == MACRO_SEALING){discharging = true; start_discharge = millis();}

    // pumps discharging at each inflation
    if(discharging){DischargePumps(start_discharge);}

    // update the state history
    macro_state_old = macro_state;
    micro_state_old = micro_state;
    Pressure_old = pmeas;
    
    // Serial communication
    Print2Serial(d_up);

    startMillis = currentMillis; // update timing variables
  }
}