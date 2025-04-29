#include <Arduino.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include <ODriveTeensyCAN.h>
#include "ODriveFlexCAN.hpp"
#include "imu_helpers.h"
#include <SimpleFOC.h>

int sign = 1;

int DELAY_TIME = 60;

// indecies for odrive controlls
const int HIP_LEFT = 0;
const int HIP_RIGHT = 1;
const int WHEEL_LEFT = 2;
const int WHEEL_RIGHT = 3;

const float HIP_START_POS_LEFT = 0.5; // in revolutions
const float HIP_START_POS_RIGHT = -0.5; // in revolutions

const float HIP_HOME = 0.5;
float HIP_MAX = 1.4;
const float HIP_MIN = 0;

float dir [4] = {1, -1, 1, -1}; // [hip_left, hip_right, wheel_left, wheel_right]

float hip_pos_left = 0;
float hip_pos_right = 0;
float hip_vel_left = 0;
float hip_vel_right = 0;

float wheel_vel_left = 0;
float wheel_vel_right = 0;

IqMsg_t iqData;         
EncoderEstimatesMsg_t end_pos_msg;
EncoderEstimatesMsg_t encoder_msg;

ODriveTeensyCAN odriveCAN(250000);

// control algorithm parameters
// stabilisation pid
// PIDController pid_stb(0.4, 0, 0.04, 100000, 0.39); low
// PIDController pid_stb(0.7, 0.8, 0.05, 100000, 0.39); high

PIDController pid_stb(0.7, 0.8, 0.05, 100000, 0.39); // PIDController
// velocity pid
PIDController pid_vel(0.01, 0.055, 0, 10000, 0.39);
// leg height pid
PIDController pid_hip(1, 0, 0, 10000, HIP_MAX); // position controller
// velocity control filtering
LowPassFilter lpf_pitch_cmd(0.07);
// low pass filters for user commands - throttle and steering
LowPassFilter lpf_throttle(0.5);
LowPassFilter lpf_steering(0.1);

// Bluetooth app variables
float steering = 0;
float throttle = 0;
float max_throttle = 20; // 20 rad/s
float max_steering = 1; // 1 V
int state = 1; // 1 on / 0 off

bool call_once = false;

// Modes
const int IDLE = 0;
const int BALANCE = 1;
const int TEST = 2;
const int HEIGHT = 3;

int mode = 1;  //an int 0 to 3

struct Controls {
  float wheel_controls = 0.0; // velocity of wheels
  float hip_controls = 0.0; // position of hip
};

// motion control tunning using commander
Commander commander = Commander(Serial);
void cntStab(char* cmd)
{ 
  Serial.println("Tuning Stabilizer PID... Use:");
  Serial.println("AP10  -> Set P gain to 10");
  Serial.println("AI0.5 -> Set I gain to 0.5");
  Serial.println("AD2   -> Set D gain to 2");
  commander.pid(&pid_stb, cmd);
}
void cntMove(char* cmd) {  commander.pid(&pid_vel, cmd);}
void lpfPitch(char* cmd) {  commander.lpf(&lpf_pitch_cmd, cmd);}
void lpfSteering(char* cmd) {  commander.lpf(&lpf_steering, cmd);}
void lpfThrottle(char* cmd) {  commander.lpf(&lpf_throttle, cmd);}

void set_idle(char* cmd) {  mode = 0;}
void set_balance(char* cmd) {  mode = 1;}
void set_switch_dir(char* cmd) {  sign = -1;}
void set_height(char* cmd) {  mode = 3;}


/*void homeMotor() {
  Serial.println("Homing motor...");

  odriveCAN.RunState(0, 8);

  delay(100);

  odriveCAN.SetLimits(0, 3, 18);

  odriveCAN.SetPosition(0, 2);

  float currentThreshold = 2.2;
  bool motorHomed = false;
  
  while (!motorHomed) {

    if (odriveCAN.ReadMsg(inMsg)) {
      odriveCAN.GetIqResponse(iqData, inMsg);
      odriveCAN.GetPositionVelocity(0);
      odriveCAN.GetPositionVelocityResponse(end_pos_msg, inMsg);
    
      Serial.print("Iq Measured: ");
      Serial.println(iqData.iqMeasured);
      Serial.print("Motor Position: ");
      Serial.println(end_pos_msg.posEstimate);
    
      if (abs(iqData.iqMeasured) > currentThreshold) {
        HIP_MAX = end_pos_msg.posEstimate;
        Serial.print("Motor end Position: ");
        Serial.println(HIP_MAX);

        odriveCAN.SetPosition(0, 0.5);

        delay(1500);

        odriveCAN.SetPosition(0, HIP_MAX - 0.1);
  
        motorHomed = true;
        Serial.println("Motor has hit the barrier! Homed.");
      }
    }
    delay(50);
  }
}*/

void setup() {
  Serial.begin(115200);  //start Serial monitor
  
  // add the configuration commands
  commander.add('A', cntStab, "pid stab");
  commander.add('B', cntMove, "pid vel");
  commander.add('C', lpfThrottle, "lpf vel command");
  commander.add('D', lpfPitch, "lpf throttle");
  commander.add('E', lpfSteering, "lpf steering");
  commander.add('i', set_idle, "idle");
  commander.add('M', set_balance, "balance");
  commander.add('s', set_switch_dir, "switch_dir");
  commander.add('h', set_height, "height");

  delay(1000);
  // imu init and configure
  if ( !initIMU() ) {
    Serial.println(F("IMU connection problem... Disabling!"));
    return;
  }
  delay(1000);

  //odriveCAN.SetLimits(HIP_LEFT, 5, 18);
  //odriveCAN.SetLimits(HIP_RIGHT, 5, 18);
  //odriveCAN.SetLimits(2, 10, 3);
  //odriveCAN.SetLimits(3, 10, 3);
  
  //odriveCAN.SetPositionGain(HIP_LEFT, 70);  //axisID, position gain
  //odriveCAN.SetVelocityGains(HIP_LEFT, 0.167, 0.333);  //axisID, velocity gain, velocity integrator gain

  //odriveCAN.SetPositionGain(HIP_RIGHT, 70);  //axisID, position gain
  //odriveCAN.SetVelocityGains(HIP_RIGHT, 0.167, 0.333);  //axisID, velocity gain, velocity integrator gain

  //odriveCAN.SetPositionGain(2, 15);  //axisID, position gain
  //odriveCAN.SetVelocityGains(2, 0.167, 0.333);  //axisID, velocity gain, velocity integrator gain

  //odriveCAN.SetPositionGain(3, 20);  //axisID, position gain
  //odriveCAN.SetVelocityGains(3, 0.167, 0.333);  //axisID, velocity gain, velocity integrator gain

  // Calibration state
  //odriveCAN.RunState(WHEEL_LEFT, 3);

  // Set odrives in close loop control mode --> 8
  odriveCAN.RunState(WHEEL_LEFT, 8);
  odriveCAN.RunState(WHEEL_RIGHT, 8);
  odriveCAN.RunState(HIP_LEFT, 8);
  odriveCAN.RunState(HIP_RIGHT, 8);

  delay(DELAY_TIME);

  odriveCAN.SetPosition(HIP_LEFT, -dir[HIP_LEFT] * 0.2);
  odriveCAN.SetPosition(HIP_RIGHT, -dir[HIP_RIGHT] * 0.2);

  //odriveCAN.SetPosition(HIP_LEFT, HIP_START_POS_LEFT);

  //delay(2000);

  //homeMotor();
}

void idle()
{
  odriveCAN.RunState(WHEEL_LEFT, 1);
  odriveCAN.RunState(WHEEL_RIGHT, 1);
  odriveCAN.RunState(HIP_LEFT, 1);
  odriveCAN.RunState(HIP_RIGHT, 1);
}

void balance(Controls controls)
{
  /*if (call_once == true)
  {
    call_once = false;
    odriveCAN.RunState(WHEEL_LEFT, 8);
    odriveCAN.RunState(HIP_LEFT, 8);
  }*/

  Serial.print("Wheel controls left: ");
  Serial.print(dir[WHEEL_LEFT] * controls.wheel_controls);
  Serial.print("  Wheel controls right: ");
  Serial.println(dir[WHEEL_RIGHT] * controls.wheel_controls);

  odriveCAN.SetTorque(WHEEL_LEFT, -dir[WHEEL_LEFT] * controls.wheel_controls);
  odriveCAN.SetTorque(WHEEL_RIGHT, -dir[WHEEL_RIGHT] * controls.wheel_controls);
}

void adjust_height()
{
  odriveCAN.SetPosition(HIP_LEFT, -dir[HIP_LEFT] * 0.1);
  odriveCAN.SetPosition(HIP_RIGHT, -dir[HIP_RIGHT] * 0.12);

  // Send "force re-run closed-loop control" to both
  odriveCAN.RunState(HIP_LEFT, ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL);
  odriveCAN.RunState(HIP_RIGHT, ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL);

  commander.pid(&pid_stb, "AP70");
  commander.pid(&pid_stb, "AI10");
  commander.pid(&pid_stb, "AD10");


  mode = BALANCE;
}

Controls compute_controls()
{
  Controls controls;

  if ( hasDataIMU() )
  {
      
    // read pitch from the IMU
    float pitch = getPitchIMU();
    float roll = getRollIMU();

    
    // wheel controls
    float target_pitch = 0; // lpf_pitch_cmd(pid_vel((wheel_vel_left + wheel_vel_right) / 2 - lpf_throttle(throttle)));
    float wheel_velocity = pid_stb(target_pitch - pitch);
    controls.wheel_controls = wheel_velocity;
      
    // compute hip controls
    /*if (!std::isnan(hip_pos_left))
    {
      float target_roll = 0;
      float hip_controls = pid_hip(0 - roll);
      controls.hip_controls = hip_pos_left * hip_controls;
    }*/
  }
  return controls;
}

void get_joint_data()
{
  uint32_t start_time = millis();
  while (millis() - start_time < 10) {
    CAN_message_t inMsg;
    if (odriveCAN.ReadMsg(inMsg))
    {
      uint32_t id_hip_left = ODriveTeensyCAN::CMD_ID_GET_ENCODER_ESTIMATES | (HIP_LEFT << 5);
      uint32_t id_hip_right = ODriveTeensyCAN::CMD_ID_GET_ENCODER_ESTIMATES | (HIP_RIGHT << 5);
      uint32_t id_wheel_left = ODriveTeensyCAN::CMD_ID_GET_ENCODER_ESTIMATES | (WHEEL_LEFT << 5);
      uint32_t id_wheel_right = ODriveTeensyCAN::CMD_ID_GET_ENCODER_ESTIMATES | (WHEEL_RIGHT << 5);

      EncoderEstimatesMsg_t posVel;

      if (inMsg.id == id_hip_left) {
        posVel.parseMessage(inMsg);
        hip_pos_left = posVel.posEstimate;
        hip_vel_left = posVel.velEstimate;
      } 
      else if (inMsg.id == id_hip_right) {
        posVel.parseMessage(inMsg);
        hip_pos_right = (-1) * posVel.posEstimate; 
        hip_vel_right = (-1) * posVel.velEstimate;
      }
      else if (inMsg.id == id_wheel_left) {
        posVel.parseMessage(inMsg);
        wheel_vel_left = posVel.velEstimate;
      }
      else if (inMsg.id == id_wheel_right) {
        posVel.parseMessage(inMsg);
        wheel_vel_right = (-1) * posVel.velEstimate;
      }
    }
  }
}

void test_wheels()
{
  float torque = sign * 0.05;
  odriveCAN.SetTorque(WHEEL_LEFT, dir[WHEEL_LEFT] * torque);
  odriveCAN.SetTorque(WHEEL_RIGHT, dir[WHEEL_RIGHT] * torque);
}

void loop() {

  get_joint_data();

  switch (mode)
  {
  case IDLE:
    call_once = true;
    idle();
    break;
  case BALANCE:
    balance(compute_controls());
    break;
  case HEIGHT:
    adjust_height();
  case TEST:
    test_wheels();
    break;
  default:
    break;
  }

  commander.run();
  //delay(500);
}  