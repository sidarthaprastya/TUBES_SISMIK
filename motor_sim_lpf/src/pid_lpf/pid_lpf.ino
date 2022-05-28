#include <ros.h>
#include <PID_v1.h>
#include <motor_sim_lpf/motor.h>
#include <motor_sim_lpf/pid_motor_arduino.h>
//#include <motor_sim_lpf/setpoint.h>

#define KP 1
#define KI 4
#define KD 0.1

#define potentio_pin A0

double setpoint = 0;
double input, output;

int potentio_value;

ros::NodeHandle nh;

PID myPID(&input, &output, &setpoint, KP, KI, KD, DIRECT); 

void getMotorFeedback(const motor_sim_lpf::motor& motor_msg){
  input = (double) motor_msg.output;
  // setpoint = (double) motor_msg.setpoint;
}

//void getSetpointFeedback(const motor_sim_lpf::motor& setpoint_msg){
//  setpoint = (double) setpoint_msg.setpoint;
//}

motor_sim_lpf::pid_motor_arduino pid_msg;
motor_sim_lpf::motor motor_msg;
//motor_sim_lpf::setpoint setpoint_msg;

ros::Subscriber<motor_sim_lpf::motor> sub_motor("motor_output", &getMotorFeedback);
//ros::Subscriber<motor_sim_lpf::motor> sub_set("set_setpoint", &getSetpointFeedback); 
ros::Publisher pub("pid_output", &pid_msg);

void setup() {
  nh.initNode();
  nh.subscribe(sub_motor);
//  nh.subscribe(sub_set);
  nh.advertise(pub);
  pid_msg.setpoint = setpoint;
  pid_msg.kp = KP;
  pid_msg.ki = KI;
  pid_msg.kd = KD;

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-10, 10);
  myPID.SetSampleTime(1);

  Serial.begin(115200);
}

void loop() {
  potentio_value = analogRead(potentio_pin);
  setpoint = (float) map(potentio_value, 0, 1023, 0, 500);
  myPID.Compute();
  pid_msg.output = output;
  pid_msg.setpoint = setpoint;
  pub.publish(&pid_msg);
  nh.spinOnce();
  delay(1);
}
