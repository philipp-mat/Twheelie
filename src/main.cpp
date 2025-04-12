#include <Arduino.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include <ODriveTeensyCAN.h>
#include "ODriveFlexCAN.hpp"
#include "imu_helpers.h"
#include <SimpleFOC.h>

int DELAY_TIME = 60;

// indecies for odrive controlls
const int HIP_LEFT = 0;
const int HIP_RIGHT = 1;
const int WHEEL_LEFT = 2;
const int WHEEL_RIGHT = 3;

const float HIP_START_POS_LEFT = 0.5; // in revolutions
const float HIP_START_POS_RIGHT = -0.5; // in revolutions

// direction of increasing position according to the Odrive (CW = +1, CCW = -1) {hip, wheel}
const int DIR[2][2] = { { 1, 1},     // left leg
                        { -1, -1}};  // right leg
const float HIP_HOME = 0.5;
float HIP_MAX = 1.4;
const float HIP_MIN = 0;

float hip_pos_left = 0;
float hip_pos_right = 0;
float wheel_vel_left = 0;
float wheel_vel_right = 0;

IqMsg_t iqData;         
CAN_message_t inMsg;
EncoderEstimatesMsg_t end_pos_msg;
EncoderEstimatesMsg_t encoder_msg;

ODriveTeensyCAN odriveCAN(250000);

// control algorithm parameters
// stabilisation pid
PIDController pid_stb(30, 100, 1, 100000, 0.39);
// velocity pid
PIDController pid_vel(0.01, 0.03, 0, 10000, 0.39);
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

bool call_once = true;

// Modes
const int IDLE = 0;
const int MOVE = 1;

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
void set_move(char* cmd) {  mode = 1;}


void homeMotor() {
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
}

void setup() {
  Serial.begin(115200);  //start Serial monitor
  
  // add the configuration commands
  commander.add('A', cntStab, "pid stab");
  commander.add('B', cntMove, "pid vel");
  commander.add('C', lpfThrottle, "lpf vel command");
  commander.add('D', lpfPitch, "lpf throttle");
  commander.add('E', lpfSteering, "lpf steering");
  commander.add('I', set_idle, "idle");
  commander.add('M', set_move, "move");

  delay(1000);
  // imu init and configure
  if ( !initIMU() ) {
    Serial.println(F("IMU connection problem... Disabling!"));
    return;
  }
  delay(1000);

  odriveCAN.SetLimits(HIP_LEFT, 5, 18);
  odriveCAN.SetLimits(HIP_RIGHT, 5, 18);
  //odriveCAN.SetLimits(2, 10, 3);
  //odriveCAN.SetLimits(3, 10, 3);
  
  odriveCAN.SetPositionGain(HIP_LEFT, 70);  //axisID, position gain
  odriveCAN.SetVelocityGains(HIP_LEFT, 0.167, 0.333);  //axisID, velocity gain, velocity integrator gain

  odriveCAN.SetPositionGain(HIP_RIGHT, 70);  //axisID, position gain
  odriveCAN.SetVelocityGains(HIP_RIGHT, 0.167, 0.333);  //axisID, velocity gain, velocity integrator gain

  //odriveCAN.SetPositionGain(2, 15);  //axisID, position gain
  //odriveCAN.SetVelocityGains(2, 0.167, 0.333);  //axisID, velocity gain, velocity integrator gain

  //odriveCAN.SetPositionGain(3, 20);  //axisID, position gain
  //odriveCAN.SetVelocityGains(3, 0.167, 0.333);  //axisID, velocity gain, velocity integrator gain

  // Calibration state
  //odriveCAN.RunState(WHEEL_LEFT, 3);

  //delay(15000);

  //odriveCAN.RunState(HIP_LEFT, 8); // run closed loop control
  //odriveCAN.RunState(HIP_RIGHT, 8);
  //odriveCAN.RunState(WHEEL_LEFT, 8);
  //odriveCAN.RunState(3, 8);

  odriveCAN.RunState(WHEEL_LEFT, 8);
  odriveCAN.RunState(HIP_LEFT, 8);

  delay(DELAY_TIME);

  odriveCAN.SetPosition(HIP_LEFT, HIP_START_POS_LEFT);

  delay(2000);

  //homeMotor();
}

void idle()
{
  odriveCAN.RunState(HIP_LEFT, 1);
  odriveCAN.RunState(WHEEL_LEFT, 1);
}

void move(Controls controls)
{
  if (call_once == true)
  {
    call_once = false;
    odriveCAN.RunState(WHEEL_LEFT, 8);
    odriveCAN.RunState(HIP_LEFT, 8);
  }
  //Serial.println(controls.wheel_controls);

  //Serial.print("hip control pos: ");
  // Serial.println(controls.hip_controls);
  //odriveCAN.SetTorque(WHEEL_LEFT, controls.wheel_controls);
}

Controls compute_controls()
{
  Controls controls;

    
  if ( hasDataIMU() )
  {
      
    // read pitch from the IMU
    float pitch = getPitchIMU();
    float roll = getRollIMU();
      
    // compute wheel controls
    if (!std::isnan(wheel_vel_left))
    {
      // wheel controls
      float target_pitch = lpf_pitch_cmd(pid_vel(wheel_vel_left - lpf_throttle(throttle)));
      // calculate the target voltage
      float wheel_velocity = pid_stb(target_pitch - pitch);
      controls.wheel_controls = wheel_velocity;
    }
      
    // compute hip controls
    if (!std::isnan(hip_pos_left))
    {
      Serial.print("hip_pos_left: ");
      Serial.println(hip_pos_left);
      float target_roll = 0;
      float hip_controls = pid_hip(0 - roll);
      controls.hip_controls = hip_pos_left * hip_controls;
    }
  }
  return controls;
}

void get_joint_data()
{
  /*if (odriveCAN.ReadMsg(inMsg))
  {
    odriveCAN.GetPositionVelocity(WHEEL_LEFT);
    odriveCAN.GetPositionVelocityResponse(encoder_msg, inMsg);
    wheel_vel_left = encoder_msg.velEstimate;
  }*/

  if (odriveCAN.ReadMsg(inMsg))
  {
    odriveCAN.GetPositionVelocity(HIP_LEFT);
    odriveCAN.GetPositionVelocityResponse(encoder_msg, inMsg);
    hip_pos_left = encoder_msg.posEstimate;
  }
}

void loop() {

  get_joint_data();

  switch (mode)
  {
  case IDLE:
    call_once = true;
    idle();
    break;
  case MOVE:
    move(compute_controls());
    break;
  default:
    break;
  }

  commander.run();
}  