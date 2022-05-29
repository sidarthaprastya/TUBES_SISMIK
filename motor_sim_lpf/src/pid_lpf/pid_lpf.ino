#include <ros.h>
#include <PID_v1.h>
#include <motor_sim_lpf/motor.h>
#include <motor_sim_lpf/pid_motor_arduino.h>
#include "TimerOne.h"

#define KP 1
#define KI 0
#define KD 0.6

#define potentio_pin A0

double setpoint = 0;
double input, output;

int potentio_value;

ros::NodeHandle nh;

PID myPID(&input, &output, &setpoint, KP, KI, KD, DIRECT); 

void getMotorFeedback(const motor_sim_lpf::motor& motor_msg){
  input = (double) motor_msg.output;
}

motor_sim_lpf::pid_motor_arduino pid_msg;
motor_sim_lpf::motor motor_msg;

ros::Subscriber<motor_sim_lpf::motor> sub_motor("motor_output", &getMotorFeedback);
ros::Publisher pub("pid_output", &pid_msg);

void timer1Callback(){
  // Compute PID per 1 ms
  myPID.Compute();

  
}

void setup() {
  nh.initNode();
  nh.subscribe(sub_motor);
  nh.advertise(pub);
  pid_msg.setpoint = setpoint;
  pid_msg.kp = KP;
  pid_msg.ki = KI;
  pid_msg.kd = KD;

  Timer1.initialize(1000); // Set interrupt for 1 ms
  Timer1.attachInterrupt(timer1Callback);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-10, 10); // Max PID output between -10 and 10
  myPID.SetSampleTime(1); // Set PID sampling for 1 ms

  Serial.begin(115200);
}

void loop() {
  // Get input for position setpoint using potentiometer
  potentio_value = analogRead(potentio_pin);
  setpoint = (float) map(potentio_value, 0, 1023, 0, 500);

  // Then, publish to "pid_output" topic
  pid_msg.output = output;
  pid_msg.setpoint = setpoint;
  pub.publish(&pid_msg);
  nh.spinOnce();
}
