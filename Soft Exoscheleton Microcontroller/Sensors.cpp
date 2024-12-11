#include "Sensors.h"


// imu raw data
int rslt_1, rslt_2;
int16_t accelGyro_1[6] = {0};
int16_t accelGyro_2[6] = {0};

// Madgwick filter for IMU processing
Madgwick filter1, filter2;

// IMU shoulder angles estimation
Matrix RM1, RM2; // rotation matrices 
Matrix RW1, RW2; // rotation matrices w/ respect to World Frame
Matrix V1, V2; // vectors

// BNO055
Adafruit_BNO055 bno_first = Adafruit_BNO055(55, 0x28, &Wire); // number 1 -> shouder
Adafruit_BNO055 bno_second = Adafruit_BNO055(55, 0x29, &Wire); // number 2 -> torso

// BMI160
const int8_t i2c_addr_1 = 0x68;
const int8_t i2c_addr_2 = 0x69;

// EMG characteristic values
int EMGbaseline = 0;
EMGFilters myFilter;  // filter
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;  // input frequency.
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;  // power supply frequency

struct data_sensors d_up;
struct ArmEstimator arm;

// other variables
float compensation_sign_detector = 0;

void SetupArmEstimation(Matrix &RM1, Matrix &RM2, Matrix &V1, Matrix &V2, struct ArmEstimator &A){
  RM1.eye();
  RM2.eye();
  RW1.eye();
  RW2.eye();
  V1.zeros(1);
  V2.zeros(1);
  A.RM1_cal.eye();
  A.RM2_cal.eye();
}

void SetupSensors(){
  // -------------------- EMG -------------------- //
  myFilter.init(sampleRate, humFreq, true, true, true);

  if (IMU_PLUGGED){
    // -------------------- BNO055 -------------------- //
    // first IMU BNO055 -> initialization
    if (!bno_first.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (!bno_first.begin());
      bno_first.begin();
      delay(100);
    }

    // second IMU BNO055 -> initialization
    if (!bno_second.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (!bno_second.begin());
      bno_second.begin();
      delay(100);
    }

    // -------------------- BMI160 -------------------- //
    // first IMU BMI160 -> initialization
    // if (bmi160_1.softReset() != BMI160_OK){
    //   Serial.println("reset1 false");
    //   while(1);
    // }
    // //set and init the bmi160 i2c address
    // if (bmi160_1.I2cInit(i2c_addr_1) != BMI160_OK){
    //   Serial.println("init1 false");
    //   while(1);
    // }
    // // second IMU BMI160 -> initialization
    // if (bmi160_2.softReset() != BMI160_OK){
    //   Serial.println("reset2 false");
    //   while(1);
    // }
    // if (bmi160_2.I2cInit(i2c_addr_2) != BMI160_OK){
    //   Serial.println("init2 false");
    //   while(1);
    // }

    // Serial.println("IMU Setup completed correctly");
  }

  // Variables initialization
  Reset_Estimation_Var();
}

void Reset_Estimation_Var(){
  arm.calibrated = false;
  arm.flex_calibrated = false;
  arm.flex_zero = 0;
  arm.flex_compensation = false;
}

void GetSensorsData(struct data_sensors &d_up_general){
  // ---------- EMG ---------- //
  // ---> FIRST EMG
  d_up_general.emg1.EMG_raw = analogRead(EMG1_OUTPUT_PIN);
  // filter processing
  d_up_general.emg1.EMG_data = myFilter.update(d_up_general.emg1.EMG_raw,1);
  // Get envelope by squaring the input
  d_up_general.emg1.EMG_data = sq(d_up_general.emg1.EMG_data);
  // ---> SECOND EMG
  d_up_general.emg2.EMG_raw = analogRead(EMG2_OUTPUT_PIN);
  // filter processing
  d_up_general.emg2.EMG_data = myFilter.update(d_up_general.emg2.EMG_raw,2);
  // Get envelope by squaring the input
  d_up_general.emg2.EMG_data = sq(d_up_general.emg2.EMG_data);
  
  // root-mean-square filtering
  if(EMG_RMS_FILT){
    // ---> FIRST EMG <---
    // Get envelope via RMS
    d_up_general.emg1.EMG_data = sqrt(0.33*(sq(d_up_general.emg1.EMG_data) + sq(d_up_general.emg1.EMG_rms_old1) + sq(d_up_general.emg1.EMG_rms_old2)));
    // EMG RMS values update
    d_up_general.emg1.EMG_rms_old2 = d_up_general.emg1.EMG_rms_old1;
    d_up_general.emg1.EMG_rms_old1 = d_up_general.emg1.EMG_data;
    // ---> SECOND EMG <---
    // Get envelope via RMS
    d_up_general.emg2.EMG_data = sqrt(0.33*(sq(d_up_general.emg2.EMG_data) + sq(d_up_general.emg2.EMG_rms_old1) + sq(d_up_general.emg2.EMG_rms_old2)));
    // EMG RMS values update
    d_up_general.emg2.EMG_rms_old2 = d_up_general.emg2.EMG_rms_old1;
    d_up_general.emg2.EMG_rms_old1 = d_up_general.emg2.EMG_data;
  }

  // exponential filtering
  if(EMG_EXP_FILT){
    // ---> FIRST EMG <---
    d_up_general.emg1.EMG_data = ALPHA_EMG*d_up_general.emg1.EMG_data + (1-ALPHA_EMG)*d_up_general.emg1.EMG_data_old;
    // EMG old value update
    d_up_general.emg1.EMG_data_old = d_up_general.emg1.EMG_data;
    // ---> SECOND EMG <---
    d_up_general.emg2.EMG_data = ALPHA_EMG*d_up_general.emg2.EMG_data + (1-ALPHA_EMG)*d_up_general.emg2.EMG_data_old;
    // EMG old value update
    d_up_general.emg2.EMG_data_old = d_up_general.emg2.EMG_data;
  }
  // Any value below the `baseline` value will be treated as zero
  if(d_up_general.emg1.EMG_data < EMGbaseline){d_up_general.emg1.EMG_data = 0;}
  if(d_up_general.emg2.EMG_data < EMGbaseline){d_up_general.emg2.EMG_data = 0;}

  // ---------- IMU BNO055 ---------- //

  //   PCB     |      EULER       |    STRUCT
  // X axis   ->   roll          ->   eul3
  // Y axis   ->   pitch         ->   eul2
  // Z axis   ->   yaw           ->   eul1

  // Collecting data from first IMU
  sensors_event_t orientationData , angVelocityData , linearAccelData;
  bno_first.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER); // Euler angles [deg]
  bno_first.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE); // Angular speed [deg/s]
  bno_first.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL); // Linear accelerations [m/s^2]
  // Extraction of data from the struct
  readIMU1(&orientationData, d_up_general);
  readIMU1(&angVelocityData, d_up_general);
  readIMU1(&linearAccelData, d_up_general);
  // Collecting data from second IMU
  bno_second.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER); // Euler angles [deg]
  bno_second.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE); // Angular speed [deg/s]
  bno_second.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL); // Linear accelerations [m/s^2]
  // Extraction of data from the struct
  readIMU2(&orientationData, d_up_general);
  readIMU2(&angVelocityData, d_up_general);
  readIMU2(&linearAccelData, d_up_general);
  // Real time calibration
  bno_first.getCalibration(&d_up_general.imu1.system, &d_up_general.imu1.gyro, &d_up_general.imu1.accel, &d_up_general.imu1.mag);
  bno_second.getCalibration(&d_up_general.imu2.system, &d_up_general.imu2.gyro, &d_up_general.imu2.accel, &d_up_general.imu2.mag);

  // ---------- IMU BMI160 ---------- //
  // // read raw data from IMU
  // bmi160_1.getAccelGyroData(accelGyro_1);
  // bmi160_2.getAccelGyroData(accelGyro_2);
  // // update the filter, which computes orientation (gyro in deg/s, accel in gravity)
  // filter1.updateIMU(accelGyro_1[0], accelGyro_1[1], accelGyro_1[2], accelGyro_1[3]/32768.0, accelGyro_1[4]/32768.0, accelGyro_1[5]/32768.0); // +/-32768 is the range for +/-1G and 2^16 bit resolution 
  // filter2.updateIMU(accelGyro_2[0], accelGyro_2[1], accelGyro_2[2], accelGyro_2[3]/32768.0, accelGyro_2[4]/32768.0, accelGyro_2[5]/32768.0);  
  // // euler angles | angular speed | linear acceleration
  // d_up_general.imu1.eul1 = filter1.getRoll();
  // d_up_general.imu1.eul2 = filter1.getPitch();
  // d_up_general.imu1.eul3 = filter1.getYaw();
  // d_up_general.imu2.eul1 = filter2.getRoll();
  // d_up_general.imu2.eul2 = filter2.getPitch();
  // d_up_general.imu2.eul3 = filter2.getYaw();
  // d_up_general.imu1.angSpeed_norm = sqrt(pow(accelGyro_1[1],2) + pow(accelGyro_1[2],2) + pow(accelGyro_1[3],2));
  // d_up_general.imu2.angSpeed_norm = sqrt(pow(accelGyro_2[1],2) + pow(accelGyro_2[2],2) + pow(accelGyro_2[3],2));
  // d_up_general.imu1.linAcc_norm = sqrt(pow(accelGyro_1[3]/32768.0,2) + pow(accelGyro_1[4]/32768.0,2) + pow(accelGyro_1[5]/32768.0,2));
  // d_up_general.imu2.linAcc_norm = sqrt(pow(accelGyro_2[3]/32768.0,2) + pow(accelGyro_2[4]/32768.0,2) + pow(accelGyro_2[5]/32768.0,2));
}

void GetPressure(struct data_sensors &d_up_general){
  // ---------- PRESSURE SENSORS ---------- //
  d_up_general.pressure1 = (float)analogRead(PRESSURE_SENSOR1);
  d_up_general.pressure2 = (float)analogRead(PRESSURE_SENSOR2);
  d_up_general.pressure1 = (d_up_general.pressure1-OFFSET_P1)*GAIN_P1;
  d_up_general.pressure2 = (d_up_general.pressure2-OFFSET_P2)*GAIN_P2;
  if(d_up_general.pressure1<0){d_up_general.pressure1=0;}
  if(d_up_general.pressure2<0){d_up_general.pressure2=0;}
}

// Function to extract data from the struct - FIRST IMU
void readIMU1(sensors_event_t* event, struct data_sensors &d_up_general) {
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    // Angular orientation
    d_up_general.imu1.eul1 = event->orientation.z;
    d_up_general.imu1.eul2 = event->orientation.y;
    d_up_general.imu1.eul3 = event->orientation.x;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    // Angular speed
    d_up_general.imu1.angSpeed_norm = sqrt(pow(event->gyro.x,2) + pow(event->gyro.y,2) + pow(event->gyro.z,2));
    d_up_general.imu1.angSpeed_norm = ALPHA_IMU_SPEED * d_up_general.imu1.angSpeed_norm + (1-ALPHA_IMU_SPEED) * d_up_general.imu1.angSpeed_norm_old;
    d_up_general.imu1.angSpeed_norm_old = d_up_general.imu1.angSpeed_norm;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    // Linear acceleration
    d_up_general.imu1.linAcc_norm = sqrt(pow(event->acceleration.x,2) + pow(event->acceleration.y,2) + pow(event->acceleration.z,2));
    d_up_general.imu1.linAcc_norm = ALPHA_IMU_ACCEL * d_up_general.imu1.linAcc_norm + (1-ALPHA_IMU_ACCEL) * d_up_general.imu1.linAcc_norm_old;
    d_up_general.imu1.linAcc_norm_old = d_up_general.imu1.linAcc_norm;
  }
}

// Function to extract data from the struct - SECOND IMU
void readIMU2(sensors_event_t* event, struct data_sensors &d_up_general) {
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    // Angular orientation
    d_up_general.imu2.eul1 = event->orientation.z;
    d_up_general.imu2.eul2 = event->orientation.y;
    d_up_general.imu2.eul3 = event->orientation.x;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    // Angular speed
    d_up_general.imu2.angSpeed_norm = sqrt(pow(event->gyro.x,2) + pow(event->gyro.y,2) + pow(event->gyro.z,2));
    d_up_general.imu2.angSpeed_norm = ALPHA_IMU_SPEED * d_up_general.imu2.angSpeed_norm + (1-ALPHA_IMU_SPEED) * d_up_general.imu2.angSpeed_norm_old;
    d_up_general.imu2.angSpeed_norm_old = d_up_general.imu2.angSpeed_norm;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    // Linear acceleration
    d_up_general.imu2.linAcc_norm = sqrt(pow(event->acceleration.x,2) + pow(event->acceleration.y,2) + pow(event->acceleration.z,2));
    d_up_general.imu2.linAcc_norm = ALPHA_IMU_ACCEL * d_up_general.imu2.linAcc_norm + (1-ALPHA_IMU_ACCEL) * d_up_general.imu2.linAcc_norm_old;
    d_up_general.imu2.linAcc_norm_old = d_up_general.imu2.linAcc_norm;
  }
}

// Sholuder Flexion/Extension Estimation
void ArmEstimation(struct ArmEstimator &arm, struct data_sensors &d_up_general){
  // x --> EAST
  // y --> NORTH
  // z --> UP

  // eul3 = roll
  // eul2 = pitch
  // eul1 = yaw

  // WF = World Frame
  // LF = Local Frame

  // 1 = shoulder, 2 = torso

  // arm re-calibration 
  if(!arm.calibrated){
    // calibrate rotation matrix
    arm.RM1_cal = eul2rotm(D2R(d_up_general.imu1.eul3),D2R(d_up_general.imu1.eul2),D2R(d_up_general.imu1.eul1));
    arm.RM2_cal = eul2rotm(D2R(d_up_general.imu2.eul3),D2R(d_up_general.imu2.eul2),D2R(d_up_general.imu2.eul1));
    // rotate w.r.t. world frame (align Z LF to Z WF to account for IMU on-body placement and IMU local orientation)
    arm.RM1_cal = dot_product(arm.RM1_cal.transpose(),Matrix(1,-90));
    arm.RM2_cal = dot_product(arm.RM2_cal.transpose(),Matrix(1,90));
    // getting the vertical reference for torso
    arm.torso_zero = d_up_general.imu2.eul2;
    arm.calibrated = true;
  }
  if(!arm.flex_calibrated && abs(arm.elev-90)<5){
    arm.flex_zero = arm.flex - 90;
    arm.flex_calibrated = true;
  }
  // Euler Angles to  Rotation Matrix
  RM1 = eul2rotm(D2R(d_up_general.imu1.eul3),D2R(d_up_general.imu1.eul2),D2R(d_up_general.imu1.eul1));
  RM2 = eul2rotm(D2R(d_up_general.imu2.eul3),D2R(d_up_general.imu2.eul2),D2R(d_up_general.imu2.eul1));
  // Calibrated Rotation Matrix
  RW1 = dot_product(RM1,arm.RM1_cal);
  RW2 = dot_product(RM2,arm.RM2_cal);

  // Torso Bending
  V1 = Matrix(0,0,1); // WF Z axis (vertical)
  V2 = dot_product(RW2, Matrix(0,1,0)); // IMU2 Y axis vs Z WF
  arm.torso = R2D(within_angle(V1,V2));

  // Shoulder Elevation
  V1 = dot_product(RW1, Matrix(0,-1,0)); // IMU1 Y axis vs Z WFV1 = dot_product(RW2, Matrix(0,1,0)); // IMU2 Y axis vs Z WF
  V2 = Matrix(0,0,1); // WF Z axis (vertical)
  arm.elev = R2D(within_angle(V1,V2));
  
  // sum and subtraction of the torso inclination depending on the sgn of the euler angle on y
  compensation_sign_detector = d_up_general.imu2.eul2-arm.torso_zero;
  if(abs(compensation_sign_detector)<0.1){arm.elev_comp = arm.elev;}
  else{arm.elev_comp = arm.elev + arm.torso*compensation_sign_detector/abs(compensation_sign_detector);}
  
  arm.elev_comp = ALPHA_ELEV*arm.elev_comp + (1-ALPHA_ELEV)*arm.elev_comp_old;
  arm.elev_comp_old = arm.elev_comp;

  // Shoulder Horizontal Flexion
  V1 = dot_product(RW1,Matrix(0,-1,0)); // for shoulder flexion computation, torso frontal vector
  V2 = Matrix(1,0,0);
  arm.flex_shoulder = R2D(within_angle(V1,V2)) * V1.v31[1]/abs(V1.v31[1]); // correction with sgn given by Y axis
  V1 = Matrix(1,0,0);
  V2 = dot_product(RW2,Matrix(1,0,0));
  arm.flex_torso = R2D(within_angle(V1,V2)) * V2.v31[1]/abs(V2.v31[1]); // correction with sgn given by Y axis;
  arm.flex = arm.flex_shoulder - arm.flex_torso - arm.flex_zero;

  // showing a meaningful curve for arm horizontal flexion
  if(arm.elev_comp < 70){arm.flex=0;}
}