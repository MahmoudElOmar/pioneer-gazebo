#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <sstream>
#include <iostream>
#include <typeinfo>


int main(int argc, char **argv)
{
    double leftWheelVelocity = 0.0;
    double rightWheelVelocity = 0.0;

    if (argc == 0)
    {
        ROS_INFO("Wrong usage. rosnode run publisgher <left_vel> <right_vel>");
        return -1;
    }
    else
    {
        //std::cout << typeid(argv[1][0]).name() << std::endl;
        //std::cout << typeid(argv[2][0]).name() << std::endl;
        leftWheelVelocity = atof(argv[1]);
        rightWheelVelocity = atof(argv[2]);
    }

    std::cout << "left wheel : " << leftWheelVelocity << " right wheel : " << rightWheelVelocity << std::endl;

    ros::init(argc, argv, "vel_publisher");



    ros::NodeHandle n;

    ros::Publisher vel_pub = n.advertise<std_msgs::Float32MultiArray>("/pioneer_robot/vel_cmd", 1);

    ros::Rate loop_rate(10);


    while (ros::ok())
    {

        std_msgs::Float32MultiArray msg;
        msg.data.push_back(leftWheelVelocity);
        msg.data.push_back(rightWheelVelocity);

        vel_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();



    }


    return 0;
}
