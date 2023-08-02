#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose2D.h>

#include <sstream>
#include <iostream>
#include <typeinfo>


geometry_msgs::Pose2D pioneer_pose;

void pose_callback(geometry_msgs::Pose2DPtr &msg)
{
    pioneer_pose.x = msg->x;
    pioneer_pose.y = msg->y;
    pioneer_pose.theta = msg->theta;
    ROS_INFO("Read World Pose\n");
}

int main(int argc, char **argv)
{
    double leftWheelVelocity = 10.0;
    double rightWheelVelocity = 10.0;

    if (argc == 0)
    {
        ROS_INFO("Wrong usage. rosnode run publisgher <left_vel> <right_vel>");
        return -1;
    }
    else
    {
        leftWheelVelocity = atof(argv[1]);
        rightWheelVelocity = atof(argv[2]);
    }
    std::cout << "left wheel : " << leftWheelVelocity << " right wheel : " << rightWheelVelocity << std::endl;
    ros::init(argc, argv, "vel_publisher");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<std_msgs::Float32MultiArray>("/pioneer_robot/vel_cmd", 1);
    ros::Subscriber sub = n.subscribe("/pioneer_robot/pose", 1, pose_callback);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std_msgs::Float32MultiArray msg;
        msg.data.push_back(leftWheelVelocity);
        msg.data.push_back(rightWheelVelocity);

        vel_pub.publish(msg);

        std::cout << "X: "<< pioneer_pose.x << std::endl;
        std::cout << "Y: "<< pioneer_pose.y << std::endl;
        std::cout << "Theta: "<< pioneer_pose.theta << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
