#ifndef _PIONEER_PLUGIN_
#define _PIONEER_PLUGIN_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <iostream>

#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose2D.h>

using namespace std;


namespace gazebo
{
    class PioneerPlugin: public ModelPlugin
    {
        public:
            PioneerPlugin() {}
            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
            {
                double left_velocity = 0.0, right_velocity = 0.0;
                double Kp = 0.0,Ki = 0.0,Kd = 0.0;
                cerr << "The Control Plugin is attached to [" << _model->GetName() << "]" << endl;

                if (_model->GetJointCount() == 0)
                {
                    cerr << "Invalid Joint Count. Plugin not loaded correctly\n";
                }
                this->model = _model;
                this->jointCount = this->model->GetJointCount();

                if (_sdf->HasElement("left_velocity"))
                    left_velocity = _sdf->Get<double>("left_velocity");
                if (_sdf->HasElement("right_velocity"))
                    right_velocity = _sdf->Get<double>("right_velocity");
                if (_sdf->HasElement("controller"))
                {
                    sdf::ElementPtr _child = _sdf->GetElement("controller");

                    if (_child->HasElement("Kp"))
                        Kp = _child->Get<double>("Kp");
                    if (_child->HasElement("Ki"))
                        Ki = _child->Get<double>("Ki");
                    if (_child->HasElement("Kd"))
                        Kd = _child->Get<double>("Kd");
                }


                cout << "Kp: " << Kp << " Kd: " << Kd << " Ki: " << Ki << endl;
                cout << "Left Velocity : " << left_velocity << " Right Velocity : " << right_velocity << endl;
                cout << "Number of Joints : " << this->jointCount << endl;
                for (unsigned int i = 0; i < this->jointCount; i++)
                {
                    this->joints.push_back(this->model->GetJoints()[i]);
                    this->controllers.push_back(common::PID(Kp, Ki, Kd));

                    this->model->GetJointController()->SetVelocityPID(
                          this->joints[i]->GetScopedName(), this->controllers[i]);

                }
                this->SetWheelVelocities(left_velocity,right_velocity);

                // Interfacing this plugin with ROS.
                if(!ros::isInitialized())
                {
                    int argc = 0;
                    char **argv = NULL;

                    ros::init(argc,argv, "pioneer_plugin",ros::init_options::NoSigintHandler);
                }

                // create ros node
                this->rosNode.reset(new ros::NodeHandle("pioneer_plugin"));

                // create a named topic an subscribed to it
                ros::SubscribeOptions so =
                        ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
                            "/"+ this->model->GetName() + "/vel_cmd",
                            1,
                            boost::bind(&PioneerPlugin::OnMsg,this,_1),
                            ros::VoidPtr(),&this->rosQueue);
                // subscribe for velocity commands on left and right wheel
                this->rosSub = this->rosNode->subscribe(so);

                // subscribe to the state publisher of the robot
                this->rosPub = this->rosNode->advertise<geometry_msgs::Pose2D>("/" + this->model->GetName() + "/pose",1);

                // sping the thread
                this->rosQueueThread = thread(bind(&PioneerPlugin::QueueThread,this));



            }
            void SetWheelVelocities(const double& _leftWheelVelocity, const double& _rightWheelVelocity)
            {
                this->model->GetJointController()->SetVelocityTarget(this->joints[0]->GetScopedName(),_leftWheelVelocity);
                this->model->GetJointController()->SetVelocityTarget(this->joints[1]->GetScopedName(),_rightWheelVelocity);
            }
            void OnMsg(const std_msgs::Float32MultiArrayConstPtr &_msg)
            {
                this->SetWheelVelocities(_msg->data[0],_msg->data[1]);
                //cout << this->model->WorldPose() << endl;
            }

        private:
            void QueueThread()
            {
                static const double timeout = 0.01;
                while(this->rosNode->ok())
                {
                    this->rosQueue.callAvailable(ros::WallDuration(timeout));
                    worldPose.x = this->model->WorldPose().X();
                    worldPose.y = this->model->WorldPose().Y();
                    worldPose.theta = this->model->WorldPose().Yaw();
                    this->rosPub.publish(worldPose);
                }
            }

        private:

            physics::ModelPtr model;

            vector<physics::JointPtr> joints;
            vector<common::PID> controllers;

            geometry_msgs::Pose2D worldPose;
            unsigned int jointCount;

            unique_ptr<ros::NodeHandle> rosNode;

            ros::Subscriber rosSub;
            ros::Publisher rosPub;

            ros::CallbackQueue rosQueue;
            thread rosQueueThread;


    };
    GZ_REGISTER_MODEL_PLUGIN(PioneerPlugin)
}
#endif

