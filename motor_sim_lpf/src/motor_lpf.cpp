// Nama     : Sidartha Prastya. P & Kelvin Sutirta
// NIM      : 13219033 & 13219069

#include "ros/ros.h"
#include <motor_sim_lpf/pid_motor_arduino.h>
#include <motor_sim_lpf/motor.h>

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <math.h>

using namespace std;

// panjang tap filter
#define BUFFERLENGTH 33
#define LENGTH 100
#define PI 3.14159265


float koef_filter[BUFFERLENGTH] = {0.0013,0.0018,0.0029,0.0047,0.0074,0.0110,0.0157,0.0214,0.0278,0.0347,0.0418,
0.0487,0.0550,0.0604,0.0645,0.0671,0.0680,0.0671,0.0645,0.0604,0.0550,0.0487,
0.0418,0.0347,0.0278,0.0214,0.0157,0.0110,0.0074,0.0047,0.0029,0.0018,0.0013};

// Variabel untuk mengubah koef ke dalam frac 32 bit
int koef_frac[BUFFERLENGTH];

float buffer[BUFFERLENGTH];
// float x[LENGTH];
// float y[LENGTH];

float x;
float y;
float curr_position = 0;

int i, j;

void conv_frac(){
    for(int i = 0; i<BUFFERLENGTH; i++){
        koef_frac[i] = round(pow(2,31) * koef_filter[i]);
    }

}

float filter(float input) {
    float hasil_fract;
    //buffering
    for(j = BUFFERLENGTH - 1; j > 0; j--) {
        buffer[j] = buffer[j-1];
    }
    //input disimpan di buffer 0
    buffer[0] = input;
    // perhitungan filter
    hasil_fract = 0;
    for(j = 0; j < BUFFERLENGTH; j++) {
        hasil_fract = hasil_fract + buffer[j] * koef_frac[j];
    }

    // Hasil dalam frac 32 bit, dikonversi kembali ke float
    float hasil = hasil_fract/pow(2,31);
    // kembalikan hasil pemfilteran
    return hasil;
}   


// motor_sim_lpf::pid_motor_arduino pid_msg;
motor_sim_lpf::motor motor_msg;
motor_sim_lpf::pid_motor_arduino pid_msg;


void pidCallback(const motor_sim_lpf::pid_motor_arduino& pid_msg){
    x = pid_msg.output;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motor_lpf");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("pid_output",1000,pidCallback);
    ros::Publisher pub_motor = nh.advertise<motor_sim_lpf::motor>("motor_output",1000);
    ros::Rate loop_rate(100);

    conv_frac(); // Mengonversi koefisien ke dalam frac 32 bit

    //inisialisasi buffer dengan nol
    for(i=0; i < BUFFERLENGTH; i++) {
        buffer[i] = 0;
    }

    while(ros::ok()){
        y = filter(x);
        // motor_msg.output = y;

        // Add Integrator for count current position
        curr_position += y;
        motor_msg.output = curr_position;

        pub_motor.publish(motor_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}