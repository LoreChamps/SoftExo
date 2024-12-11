#include "Controls.h"

// system states
uint8_t micro_state = MICRO_SEAL;
uint8_t micro_state_old = MICRO_SEAL;
uint8_t macro_state = MACRO_SEALING;
uint8_t macro_state_old = MACRO_SEALING;

// controls
int control_mode = 0; // 0 = idle, 1 = manual, 2 = automatic, 3 = fatigue test
float pref = 15;
float pmeas = 0;
float perr_th = 0.5;
float perr = 0;
uint32_t time_start = 0;
uint32_t start_discharge = 0;
bool press_autocontrol = false;
bool flag_control = false;
bool start_fatigue = true;
int step_fatigue = 1;
int current_cycle = 0;
int needed_cycles = 0;
int counter_idle = 0;
bool discharging = false;

// low-level pressure control
bool asymmetric_margin = false;
float Pressure_old = 0;




// --------- HIGH LEVEL CONTROLS --------- //

void PressureControl(){
  perr=pmeas-pref;
  switch (control_mode){
    case 2: // automatic control
      if(asymmetric_margin){perr=pmeas-(pref+ASYMMETRIC_GAIN*perr_th);} // if asymmetric margin True -> update the measured error ----> Perché un prodotto fra gain e offset e non solo un offset?
      if(abs(perr)<perr_th){SetState(MICRO_SEAL); asymmetric_margin=false;} // perché setti i micro state dal controllo? mi aspetterei i macro state...
      else if(perr>0){SetState(MICRO_DEFLATE_VV);}
      else{SetState(MICRO_INFLATE_PP);asymmetric_margin=true;}
      break;
    case 3: // fatigue test
      if(abs(perr)<ERROR_FATIGUE & step_fatigue == 1){start_fatigue = false; SetState(MICRO_SEAL); time_start = millis(); step_fatigue = 2; current_cycle++;}
      else if(step_fatigue == 2 & (millis()-time_start)>=TIME_REST_HP){SetState(MICRO_DEFLATE_VV); time_start = millis(); step_fatigue = 3;}
      else if(step_fatigue == 3 & (millis()-time_start)>=TIME_REST_LP){SetState(MICRO_SEAL); start_fatigue = true; time_start = millis(); step_fatigue = 1;}
      else if(start_fatigue){SetState(MICRO_INFLATE_PP);}

      if((macro_state==MACRO_INFLATING && (millis()-time_start)>=STOP_FATIGUE) || (step_fatigue==3 && current_cycle==needed_cycles)){Reset_Control_Var(); control_mode = 0;}
      break;
  }
}


// --------- LOW LEVEL CONTROLS --------- //

int IdleControl(int c){
  c=c+1;
  if(pmeas>PRESS_IDLE_LIM && c<COUNTER_IDLE_LIM){SetState(MICRO_DEFLATE_VV);}
  else{SetState(MICRO_SEAL);}
  return c;
}

void Inflate(int PUMP){
  digitalWrite(VALVE_1, LOW);
  digitalWrite(VALVE_2, LOW);
  if(PUMP==PUMP_1){digitalWrite(PUMP_1, HIGH);digitalWrite(PUMP_2, LOW);}
  else if(PUMP==PUMP_2){digitalWrite(PUMP_2, HIGH);digitalWrite(PUMP_1, LOW);}
  else{digitalWrite(PUMP_1, HIGH);digitalWrite(PUMP_2, HIGH);}
}

void Deflate(int VALVE){
  digitalWrite(PUMP_1, LOW);
  digitalWrite(PUMP_2, LOW);
  if(VALVE==VALVE_1){digitalWrite(VALVE_1, HIGH);digitalWrite(VALVE_2, LOW);}
  else if(VALVE==VALVE_2){digitalWrite(VALVE_2, HIGH);digitalWrite(VALVE_1, LOW);}
  else{digitalWrite(VALVE_1, HIGH);digitalWrite(VALVE_2, HIGH);}
}

void Seal(){
  digitalWrite(PUMP_1, LOW);
  digitalWrite(PUMP_2, LOW);
  digitalWrite(VALVE_1, LOW);
  digitalWrite(VALVE_2, LOW);
}

void DischargePumps(uint32_t Tstart){
  if((millis()-Tstart)<75){digitalWrite(VALVE_DISCHARGE, HIGH);}
  else{digitalWrite(VALVE_DISCHARGE, LOW); discharging = false;}
}

void Reset_Control_Var(){
  flag_control = false;
  step_fatigue = 1;
  start_fatigue = true;
  if(control_mode!=0){counter_idle = 0;}
}

// --------- STATE MACHINE --------- //

void SetState(uint8_t state2set){
  // set new micro state only if it is different that the previous
  if(state2set != micro_state){
    micro_state = state2set;
    // set new macro states
    if(micro_state == MICRO_INFLATE_P1 || micro_state == MICRO_INFLATE_P2 || micro_state == MICRO_INFLATE_PP){macro_state = MACRO_INFLATING;}
    else if(micro_state == MICRO_DEFLATE_V1 || micro_state == MICRO_DEFLATE_V2 || micro_state == MICRO_DEFLATE_VV){macro_state = MACRO_DEFLATING;}
    else{macro_state = MACRO_SEALING;}
  }
}

void ChangeState(){
  if(micro_state == MICRO_INFLATE_P1){Inflate(PUMP_1);}
  else if(micro_state == MICRO_INFLATE_P2){Inflate(PUMP_2);}
  else if(micro_state == MICRO_INFLATE_PP){Inflate(0);}
  else if(micro_state == MICRO_DEFLATE_V1){Deflate(VALVE_1);}
  else if(micro_state == MICRO_DEFLATE_V2){Deflate(VALVE_2);}
  else if(micro_state == MICRO_DEFLATE_VV){Deflate(VALVE_1);} // substitute Deflate(VALVE_1); with Deflate(0); if valve 2 is installed in the robot
  else{Seal();}
}