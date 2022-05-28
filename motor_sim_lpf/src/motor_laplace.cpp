// Nama     : Sidartha Prastya. P & Kelvin Sutirta
// NIM      : 13219033 & 13219069

#include "ros/ros.h"
#include <motor_sim_lpf/pid_motor_arduino.h>
#include <motor_sim_lpf/motor.h>

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <math.h>

float x[2] = {0.0, 0.0};
float y[2] = {0.0, 0.0};
float curr_position = 0;

// motor_sim_lpf::pid_motor_arduino pid_msg;
motor_sim_lpf::motor motor_msg;
motor_sim_lpf::pid_motor_arduino pid_msg;


void pidCallback(const motor_sim_lpf::pid_motor_arduino& pid_msg){
    x[1] = x[0]; 
    x[0] = pid_msg.output;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motor_laplace");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("pid_output",1000,pidCallback);
    ros::Publisher pub_motor = nh.advertise<motor_sim_lpf::motor>("motor_output",1000);
    ros::Rate loop_rate(100);

    while(ros::ok()){
        y[1] = y[0];
        y[0] = 0.0476 * x[0] + 0.0476 * x[1] + 0.9048 * y[1];
        // motor_msg.output = y;

        // Add Integrator for count current position
        curr_position += y[0];
        motor_msg.output = curr_position;

        pub_motor.publish(motor_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}