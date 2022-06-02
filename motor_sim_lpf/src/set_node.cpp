#include "ros/ros.h"
#include <thread>
#include <motor_sim_lpf/constant.h>

// This file is for updating Kp, Ki, Kd of the Arduino

motor_sim_lpf::constant con_msg;
float kp,ki,kd;
bool change = false;

int main(int argc, char *argv[]){
    ros::init(argc, argv, "set_node");
    ros::NodeHandle nh;

    ros::Publisher pub_set = nh.advertise<motor_sim_lpf::constant>("set_pid",1000);

    while(ros::ok()){
        printf("Kp: ");
        scanf("%f", &kp);
        printf("Ki: ");
        scanf("%f", &ki);
        printf("Kd: ");
        scanf("%f", &kd);
        printf("\n");
        con_msg.kp = kp;
        con_msg.ki = ki;
        con_msg.kd = kd;
        change = true;
        con_msg.newData = change;
        pub_set.publish(con_msg);

        ros::spinOnce();
    }

}