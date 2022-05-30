#include <ros.h>
#include <PID_v1.h>
#include <motor_sim_lpf/motor.h>
#include <motor_sim_lpf/pid_motor_arduino.h>
#include <motor_sim_lpf/constant.h>
#include "TimerOne.h"
#include "EEPROM.h"

// #define KP 1
// #define KI 0
// #define KD 0.6

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



//void showParsedData() {
//    Serial.print("konstanta: ");
//    Serial.print(kp);
//    Serial.print(", ");
//    Serial.print(ki);
//    Serial.print(", ");
//    Serial.println(kd);
//}
//
//void recvWithStartEndMarkers() {
//    static boolean recvInProgress = false;
//    static byte ndx = 0;
//    char startMarker = '<';
//    char endMarker = '>';
//    char rc;
//
//    while (Serial.available() > 0 && newData == false) {
//        rc = Serial.read();
//
//        if (recvInProgress == true) {
//            if (rc != endMarker) {
//                receivedChars[ndx] = rc;
//                ndx++;
//                if (ndx >= numChars) {
//                    ndx = numChars - 1;
//                }
//            }
//            else {
//                receivedChars[ndx] = '\0'; // terminate the string
//                recvInProgress = false;
//                ndx = 0;
//                newData = true;
//            }
//        }
//
//        else if (rc == startMarker) {
//            recvInProgress = true;
//        }
//    }
//}
//
//void parseData() {      // split the data into its parts
//
//    char * strtokIndx; // this is used by strtok() as an index
//
//    strtokIndx = strtok(tempChars,",");      // get the first part - the string
//    kp = atof(strtokIndx); // copy it to messageFromPC
// 
//    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
//    ki = atof(strtokIndx);     // convert this part to an integer
//
//    strtokIndx = strtok(NULL, ",");
//    kd = atof(strtokIndx);     // convert this part to a float
//
//}


void setup() {
  nh.initNode();
  nh.subscribe(sub_motor);
  nh.subscribe(sub_cons);
  nh.advertise(pub);
//   pid_msg.setpoint = setpoint;
//   pid_msg.kp = KP;
//   pid_msg.ki = KI;
//   pid_msg.kd = KD;

  Timer1.initialize(10000); // Set interrupt for 10 ms
  Timer1.attachInterrupt(timer1Callback);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-5, 5); // Max PID output between -1 and 1
  myPID.SetSampleTime(10); // Set PID sampling for 10 ms

  Serial.begin(115200);
  Serial.println("Berikan input dengan format <kp,ki,kd>:");
}

void loop() {
//  recvWithStartEndMarkers();
//  if (newData == true) {
//      strcpy(tempChars, receivedChars);
//          // this temporary copy is necessary to protect the original data
//          //   because strtok() used in parseData() replaces the commas with \0
//      parseData();
//      
//      newData = false;
//

////      showParsedData();
//  }
  
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
