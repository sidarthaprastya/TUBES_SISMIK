#include <ros.h>
#include <PID_v1.h>
#include <motor_sim_lpf/motor.h>
#include <motor_sim_lpf/pid_motor_arduino.h>
#include <motor_sim_lpf/constant.h>
#include "TimerOne.h"
#include "EEPROM.h"


#define potentio_pin A0

double setpoint = 0;
double input, output;

int potentio_value;
int address[3] = {0,1,2}; // EEPROM address for Kp, Ki, and Kd
double kp, ki, kd;

// Variables for getting serial input
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
bool newData = false;

ros::NodeHandle nh;

PID myPID(&input, &output, &setpoint, 0, 0, 0, DIRECT); 

void getMotorFeedback(const motor_sim_lpf::motor& motor_msg){
  input = (double) motor_msg.output;
}

void getConsFeedback(const motor_sim_lpf::constant& cons_msg){
  if(cons_msg.newData == true){
     kp = (double) cons_msg.kp;
     ki = (double) cons_msg.ki;
     kd = (double) cons_msg.kd;

      EEPROM.update(address[0],kp);
      EEPROM.update(address[1],ki);
      EEPROM.update(address[2],kd);
  }
}

motor_sim_lpf::pid_motor_arduino pid_msg;
motor_sim_lpf::motor motor_msg;

ros::Subscriber<motor_sim_lpf::motor> sub_motor("motor_output", &getMotorFeedback);
ros::Subscriber<motor_sim_lpf::constant> sub_cons("set_pid", &getConsFeedback);
ros::Publisher pub("pid_output", &pid_msg);

void timer1Callback(){
  // Compute PID per 10 ms
  myPID.Compute();
}


void setup() {
  nh.initNode();
  nh.subscribe(sub_motor);
  nh.subscribe(sub_cons);
  nh.advertise(pub);

  Timer1.initialize(10000); // Set interrupt for 10 ms
  Timer1.attachInterrupt(timer1Callback);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-5, 5); // Max PID output between -1 and 1
  myPID.SetSampleTime(10); // Set PID sampling for 10 ms

  Serial.begin(115200);
  Serial.println("Berikan input dengan format <kp,ki,kd>:");
}

void loop() {
  
  // Load EEPROM Value
  // EEPROM in arduino takes 3.3ms
  kp = EEPROM.read(address[0]);
  ki = EEPROM.read(address[1]);
  kd = EEPROM.read(address[2]);

  myPID.SetTunings(kp,ki,kd);

  // Get input for position setpoint using potentiometer
  potentio_value = analogRead(potentio_pin);
  setpoint = (float) map(potentio_value, 0, 1023, 0, 500);

  // Then, publish to "pid_output" topic
  pid_msg.kp = kp;
  pid_msg.ki = ki;
  pid_msg.kd = kd;
  pid_msg.output = output;
  pid_msg.setpoint = setpoint;
  pub.publish(&pid_msg);
  nh.spinOnce();
}
