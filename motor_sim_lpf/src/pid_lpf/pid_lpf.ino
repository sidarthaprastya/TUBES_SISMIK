#include <ros.h>
#include <PID_v1.h>
#include <motor_sim_lpf/motor.h>
#include <motor_sim_lpf/pid_motor_arduino.h>

#define KP 0.6
#define KI 1.5
#define KD 0

double setpoint = 100.0;
double input, output;

ros::NodeHandle nh;

PID myPID(&input, &output, &setpoint, KP, KI, KD, DIRECT); 

void getFeedback(const motor_sim_lpf::motor& motor_msg){
  input = (double) motor_msg.output;
}

motor_sim_lpf::pid_motor_arduino pid_msg;
motor_sim_lpf::motor motor_msg;

ros::Subscriber<motor_sim_lpf::motor> sub("motor_output", &getFeedback); 
ros::Publisher pub("pid_output", &pid_msg);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  pid_msg.setpoint = setpoint;
  pid_msg.kp = KP;
  pid_msg.ki = KI;
  pid_msg.kd = KD;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0.0, 200);
  myPID.SetSampleTime(1);

  Serial.begin(115200);
}

void loop() {
  myPID.Compute();
  pid_msg.output = output;
  pub.publish(&pid_msg);
  nh.spinOnce();
  delay(1);
}
