#include "IO.h"

// user input vars
String user_input;
int tmp, tmp1, tmp2; // user_input to int
int kilos = 0;



void Print2Serial(struct data_sensors d_up_general){
  // data 0-1 -> first EMG: filtered + raw | FLEXION MOVEMENT
  Serial.print(d_up_general.emg1.EMG_data);
  Serial.print(",");
  Serial.print(d_up_general.emg1.EMG_raw);
  Serial.print(",");
  // data 2-3 -> second EMG: filtered + raw | ABDUCTION MOVEMENT
  Serial.print(d_up_general.emg2.EMG_data);
  Serial.print(",");
  Serial.print(d_up_general.emg2.EMG_raw);
  Serial.print(",");
  // data 4 -> EMG baseline
  Serial.print(EMGbaseline);
  Serial.print(",");
  // data 5-6-7 -> first IMU | Euler angles
  Serial.print(d_up_general.imu1.eul1,2);
  Serial.print(",");
  Serial.print(d_up_general.imu1.eul2,2);
  Serial.print(",");
  Serial.print(d_up_general.imu1.eul3,2);
  Serial.print(",");
  // data 8-9 -> first IMU | AngSpeed - LinAcc
  Serial.print(d_up_general.imu1.angSpeed_norm,2);
  Serial.print(",");
  Serial.print(d_up_general.imu1.linAcc_norm,2);
  Serial.print(",");
  // data 10-11-12 -> second IMU | Euler angles
  Serial.print(d_up_general.imu2.eul1,2);
  Serial.print(",");
  Serial.print(d_up_general.imu2.eul2,2);
  Serial.print(",");
  Serial.print(d_up_general.imu2.eul3,2);
  Serial.print(",");
  // data 13-14 -> second IMU | AngSpeed - LinAcc
  Serial.print(d_up_general.imu2.angSpeed_norm,2);
  Serial.print(",");
  Serial.print(d_up_general.imu2.linAcc_norm,2);
  Serial.print(",");
  // data 15-16 -> ArmElevation calibration
  Serial.print(arm.calibrated);
  Serial.print(",");
  Serial.print(arm.flex_calibrated);
  Serial.print(",");
  // data 17-18-19 -> torso | elev | flex angle estimations
  Serial.print(arm.torso,2);
  Serial.print(",");
  Serial.print(arm.elev_comp,2);
  Serial.print(",");
  Serial.print(arm.flex,2);
  Serial.print(",");
  // data 20-21-22 -> pressure sensors + reference pressure
  Serial.print(d_up_general.pressure1,1);
  Serial.print(",");
  Serial.print(pref);
  Serial.print(",");
  Serial.print(perr);
  Serial.print(",");
  // data 23-24-25-26 -> pumps & valves state
  Serial.print(digitalRead(PUMP_1));
  Serial.print(",");
  Serial.print(digitalRead(PUMP_2));
  Serial.print(",");
  Serial.print(digitalRead(VALVE_1));
  Serial.print(",");
  Serial.print(digitalRead(VALVE_2));
  Serial.print(",");
  // data 27-28 -> fatigue_cycle parameters
  Serial.print(current_cycle);
  Serial.print(",");
  Serial.print(needed_cycles);
  Serial.print(",");
  // data 29-30-31-32 -> system states
  Serial.print(micro_state_old);
  Serial.print(",");
  Serial.print(micro_state);
  Serial.print(",");
  Serial.print(macro_state_old);
  Serial.print(",");
  Serial.print(macro_state);
  Serial.print(",");
  // data 33-34-35-36 -> control mode + timeStamp + userInput + kilograms
  Serial.print(control_mode);
  Serial.print(",");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(user_input);
  Serial.print(",");
  Serial.print(kilos);
  Serial.println(",");
}

void UserInput(){
  if(Serial.available()){
    // read user input and make integer out of it
    user_input = Serial.readStringUntil('|');  // \r\n "passo e chiudo"
    tmp = user_input.toInt();
    tmp1 = user_input.charAt(0) - '0'; // first digit
    tmp2 = tmp - 1000*tmp1; // rest of number

    switch (tmp1) {
      case 0: // SW reset
        switch (tmp2) {
          case 0: // tab widget changes
            Reset_Control_Var(); // if starting -> nothing changes | if stopping -> reset
            control_mode = 0;
            break;
          case 1: // gui starts or closes
            Reset_Control_Var(); // if starting -> nothing changes | if stopping -> reset
            Reset_Estimation_Var(); // reset of all the variables in ARM estimation
            control_mode = 0;
            break;
        }
        break;
      case 1: // manual control
        Reset_Control_Var(); // if starting -> nothing changes | if stopping -> reset
        control_mode = 1;
        switch (tmp2) {
          case 1: // inflate with P1 selected
            if(micro_state == MICRO_INFLATE_P1){SetState(MICRO_SEAL);}
            else if(micro_state == MICRO_INFLATE_P2){SetState(MICRO_INFLATE_PP);}
            else if(micro_state == MICRO_INFLATE_PP){SetState(MICRO_INFLATE_P2);}
            else{SetState(MICRO_INFLATE_P1);}
            break;
          case 2: // inflate with P2 selected
            if(micro_state == MICRO_INFLATE_P2){SetState(MICRO_SEAL);}
            else if(micro_state == MICRO_INFLATE_P1){SetState(MICRO_INFLATE_PP);}
            else if(micro_state == MICRO_INFLATE_PP){SetState(MICRO_INFLATE_P1);}
            else{SetState(MICRO_INFLATE_P2);}
            break;
          case 3: // deflate with V1 selected
            if(micro_state == MICRO_DEFLATE_V1){SetState(MICRO_SEAL);}
            else if(micro_state == MICRO_DEFLATE_V2){SetState(MICRO_DEFLATE_VV);}
            else if(micro_state == MICRO_DEFLATE_VV){SetState(MICRO_DEFLATE_V2);}
            else{SetState(MICRO_DEFLATE_V1);}
            break;
          case 4: // deflate with V2 selected
            if(micro_state == MICRO_DEFLATE_V2){SetState(MICRO_SEAL);}
            else if(micro_state == MICRO_DEFLATE_V1){SetState(MICRO_DEFLATE_VV);}
            else if(micro_state == MICRO_DEFLATE_VV){SetState(MICRO_DEFLATE_V1);}
            else{SetState(MICRO_DEFLATE_V2);}
            break;
        }
        break;
      case 2: // control laws
        counter_idle = 0;
        flag_control = !flag_control;
        switch (tmp2) {
          case 1: // pressure setpoint
            control_mode = 2*(int)flag_control;
            break;
          case 2: // fatigue test
            if(control_mode!=3){current_cycle=0; time_start=millis(); step_fatigue = 1; start_fatigue = true;} // check if starting from a different modality
            control_mode = 3*(int)flag_control; // updating the control_mode to the one of the fatigue
            break;
        }
        break;
      case 3: // set pressure
        pref = tmp2;
        break;
      case 4: // set error threshold
        perr_th = tmp2/10.0;
        break;
      case 5: // IMU parameters
        switch (tmp2) {
          case 1: // re-calibrate
            Reset_Estimation_Var();
            break;
        }
      case 6: // set EMG baseline
        EMGbaseline = tmp2;
        break;
      case 7: // number of fatigue cycles to perform
        needed_cycles = tmp2*10;
        break;
      case 8: // kilos raised in elevation
        kilos = tmp2;
        break;
      default:
        break;
    }
  }
  else{
    user_input = "";
  }
}