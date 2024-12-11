#ifndef Sensors_header
#define Sensors_header

#include "Arduino.h"
#include "Definitions.h"
#include "MathFunctions.h"
#include <MadgwickAHRS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "EMGFilters.h"

// -------------------- IMU -------------------- //

// BNO055
extern Adafruit_BNO055 bno_first;
extern Adafruit_BNO055 bno_second;

// BMI160
// extern DFRobot_BMI160 bmi160_1;
// extern DFRobot_BMI160 bmi160_2;

// imu raw data
extern int rslt_1, rslt_2;
extern int16_t accelGyro_1[6];
extern int16_t accelGyro_2[6];

// Madgwick filter for IMU processing
extern Madgwick filter1, filter2;

// IMU shoulder angles estimation
extern Matrix RM1, RM2; // rotation matrices 
extern Matrix RW1, RW2; // rotation matrices w/ respect to World Frame
extern Matrix V1, V2; // vectors

struct ArmEstimator{
  bool calibrated; // calibration flag
  bool flex_calibrated;
  float flex_zero;
  float torso_zero;
  float flex_shoulder, flex_torso;
  float elev_comp;
  bool flex_compensation;
  Matrix RM1_cal, RM2_cal; // calibration matrices
  float torso, elev, flex; // output vars
  float elev_comp_old; // last loop variables for exponential filtering
};

// IMU data struct
struct imu_data{
  float eul1, eul2, eul3, angSpeed_norm, linAcc_norm;
  float angSpeed_norm_old, linAcc_norm_old;
  uint8_t system, gyro, accel, mag;
};

// EMG data struct
struct emg_data{
  float EMG_data, EMG_data_old;
  float EMG_rms_old1, EMG_rms_old2;
  float EMG_raw;
};

extern struct imu_data imu1;
extern struct imu_data imu2;
extern struct emg_data emg1;
extern struct emg_data emg2;

#define ALPHA_IMU_SPEED 0.3 // exponential filtering on speed
#define ALPHA_IMU_ACCEL 0.3 // exponential filtering on acceleration
#define ALPHA_ELEV 0.7 // exponential filtering on elevation angle

// -------------------- EMG -------------------- //

// EMG characteristic values
extern EMGFilters myFilter;  // filter
extern SAMPLE_FREQUENCY sampleRate;  // input frequency.
extern NOTCH_FREQUENCY humFreq;  // power supply frequency

// EMG baseline
extern int EMGbaseline;
#define EMG_RMS_FILT false
#define EMG_EXP_FILT true
#define ALPHA_EMG 0.03 // exponential filtering on EMG signal

// -------------------- Pressure -------------------- //

// pressure sensor
#define ALPHA_PRESSURE_IN 0.1
#define ALPHA_PRESSURE_OUT 0.02
#define ALPHA_PRESSURE 0.04
#define OFFSET_P1 155.0
#define GAIN_P1 25.0/(770-OFFSET_P1)
#define OFFSET_P2 155.0
#define GAIN_P2 25.0/(770-OFFSET_P2)


// -------------------- General Structure -------------------- //

// all sensors data struct
struct data_sensors{
  emg_data emg1; // data for EMG1
  emg_data emg2; // data for EMG2
  imu_data imu1; // data for IMU1
  imu_data imu2; // data for IMU2
  float pressure1, pressure2; // pressure sensors
};

extern struct data_sensors d_up;
extern struct ArmEstimator arm;

// other variables
extern float compensation_sign_detector;

void SetupArmEstimation(Matrix &RM1, Matrix &RM2, Matrix &V1, Matrix &V2, struct ArmEstimator &A);
void SetupSensors();
void Reset_Estimation_Var();
void ArmEstimation(struct ArmEstimator &arm, struct data_sensors &d_up_general);
void readIMU1(sensors_event_t* event, struct data_sensors &d_up_general);
void readIMU2(sensors_event_t* event, struct data_sensors &d_up_general);
void GetSensorsData(struct data_sensors &d_up_general);
void GetPressure(struct data_sensors &d_up_general);

#endif