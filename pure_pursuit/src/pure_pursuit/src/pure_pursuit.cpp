#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/console.h>
#include "pure_pursuit.hpp"
#include <iostream>

int main(int argc, char **argv)
{
    float cx [51] = {};
    float cy [51] = {}; 
    float target_speed = 6.0 / 3.6;
    float T = 100.0;
    float time = 0.0;
    float ai = 0.0;
    int LastIndex = 50;
    control target_ind;

    ros::init(argc,argv,"pure_pursuit");
    ros::NodeHandle nh;
    ROS_INFO("begin");
    ros::Publisher my_pub_cx = nh.advertise<std_msgs::Float32MultiArray>("cx",1000 );
    ros::Publisher my_pub_cy = nh.advertise<std_msgs::Float32MultiArray>("cy",1000 );
    ros::Publisher my_pub_x = nh.advertise<std_msgs::Float32MultiArray>("x",1000 );
    ros::Publisher my_pub_y = nh.advertise<std_msgs::Float32MultiArray>("y",1000 );
    ros::Publisher my_pub_yaw = nh.advertise<std_msgs::Float32MultiArray>("yaw",1000 );
    ros::Publisher my_pub_v = nh.advertise<std_msgs::Float32MultiArray>("v",1000 );
    ros::Publisher my_pub_plot_time = nh.advertise<std_msgs::Float32MultiArray>("time",1000 );

    ros::Rate rate(10);
    std_msgs::Float32MultiArray msg_cx, msg_cy;
    std_msgs::Float32MultiArray x, y, yaw, v ,plot_time ;
    for(int i = 0; i<51; i++)
        cx[i] = i;
    for(int i = 0; i<51; i++)
        {
        cy[i] = sinf(i / 5.0) * i / 2.0;
        std::cout <<cy[i] <<std::endl;  

        }
    
    VehicleState car (-0.0, -3.0, 0.0, 0.0);
    // std::cout<<"out:car return"<<car.state_return.state_x<<"  "<<
    //     car.state_return.state_y<<"  "<<car.state_return.state_yaw<<"  "<<car.state_return.state_v<<std::endl;

    x.data.push_back(car.state_return.state_x);
    y.data.push_back(car.state_return.state_y);
    yaw.data.push_back(car.state_return.state_yaw);
    v.data.push_back(car.state_return.state_v);
    target_ind.ind = car.calc_target_index(cx, cy);
    // std::cout<<"target_ind.ind = "<<target_ind.ind<<std::endl;;

    for(int i=0; i<51; i++)
    {
    msg_cx.data.push_back(cx[i]);
    msg_cy.data.push_back(cy[i]);
    }
        while (T >= time && LastIndex > target_ind.ind)
        {
        ai = car.PControl(target_speed, car.state_return.state_v);
        // std::cout<<"ai " <<ai<<std::endl;
        target_ind = car.pure_pursuit_control(cx, cy, target_ind.ind, 51);//di有问题
        // std::cout<<"target_ind.ind = " <<target_ind.ind<<std::endl;
        // std::cout<<"target_ind.di = " <<target_ind.di<<std::endl;
        // std::cout<<"target_ind.delta = " <<target_ind.delta<<std::endl;
        car.Update(ai,target_ind.delta);
        // std::cout<<"car.state_return x= " <<car.state_return.state_x<<std::endl;

        time = time +dt;
        // std::cout<<"time = " << time <<std::endl;

        x.data.push_back(car.state_return.state_x);
        y.data.push_back(car.state_return.state_y);
        // std::cout <<car.state_return.state_y <<std::endl;  
        // std::cout <<car.state_return.state_x <<std::endl;  
        yaw.data.push_back(car.state_return.state_yaw);
        v.data.push_back(car.state_return.state_v);
        plot_time.data.push_back(time);
        // std::cout<<"     " <<std::endl;  

        }
    std::cout<<"cout : x" <<std::endl;

    for(auto i = x.data.begin();i< x.data.end() ;i++)
    {
        std::cout<< *i <<std::endl;

    }
    std::cout<<"cout : y" <<std::endl;

    for(auto i = y.data.begin();i< y.data.end() ;i++)
    {
        std::cout<< *i <<std::endl;

    }
    while (ros::ok())
    {
        my_pub_cx.publish(msg_cx);
        my_pub_cy.publish(msg_cy);    
        my_pub_x.publish(x);
        my_pub_y.publish(y);
        my_pub_yaw.publish(yaw);
        my_pub_v.publish(v);
        my_pub_plot_time.publish(plot_time);
        rate.sleep();

    }


    ros::spin();
     return 0;  
}