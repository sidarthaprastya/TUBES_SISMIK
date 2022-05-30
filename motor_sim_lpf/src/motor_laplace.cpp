// Nama     : Sidartha Prastya. P & Kelvin Sutirta
// NIM      : 13219033 & 13219069

#include "ros/ros.h"
#include <ros/package.h>
#include <motor_sim_lpf/pid_motor_arduino.h>
#include <motor_sim_lpf/motor.h>


#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string>
#include <math.h>

using namespace std;

float x[2] = {0.0, 0.0};
float y[2] = {0.0, 0.0};
float curr_position = 0;

std::string path = ros::package::getPath("motor_sim_lpf");

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

    ofstream fptr;

    string path_log = path.append("/src/motor_laplace_log.txt");
    // Save file in the same directory
    fptr.open(path_log);

    while(ros::ok()){
        y[1] = y[0];

        // INsert to Laplace equation (10/s+10)
        y[0] = 0.0476 * x[0] + 0.0476 * x[1] + 0.9048 * y[1];

        // Add Integrator for count current position
        curr_position += y[0];
        // fprintf(fptr, "%.2f\n", curr_position);
        fptr << (float)curr_position << endl;

        motor_msg.output = curr_position;


        pub_motor.publish(motor_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}