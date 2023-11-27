#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"  // Include the appropriate message type for your controller
#include "std_msgs/Float64.h"  // Include the appropriate message type for your controller

// Callback function to print joint positions
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){
    ROS_INFO("Received joint states:");
    for(size_t i = 0; i < msg->name.size(); i++){
        ROS_INFO("%s: %f", msg->name[i].c_str(), msg->position[i]);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joint_states_sub_pub");
    ros::NodeHandle nh;

    // Create a subscriber for the "joint_states" topic
    ros::Subscriber sub = nh.subscribe("/arm/joint_states", 1, jointStatesCallback);

    // Create a publisher for the /command topic
    ros::Publisher pub0 = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J0_controller/command", 1000);
    ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J1_controller/command", 1000);
    ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J2_controller/command", 1000);
    ros::Publisher pub3 = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J3_controller/command", 1000);

    // Rate at which to publish commands (adjust as needed)
    ros::Rate loop_rate(10);  // 10 Hz
    
    std_msgs::Float64 msg;

    while(ros::ok()){
        msg.data=0;
        char r='0';

        std::cout<<"Vuoi inserire un comando per i joint? (Y/N): "<<std::endl;
        std::cin>>r;
    
        if(r=='Y' || r=='y'){
            std::cout <<"Inserisci j0 : " <<std::endl;
            std::cin>> msg.data;
            pub0.publish(msg);

            std::cout <<"Inserisci j1: " <<std::endl;
            std::cin>> msg.data;
            pub1.publish(msg);

            std::cout <<"Inserisci j2: " <<std::endl;
            std::cin>> msg.data;
            pub2.publish(msg);

            std::cout <<"Inserisci j3: " <<std::endl;
            std::cin>> msg.data;
            pub3.publish(msg);
        }
        
        // Spin once to process callbacks and maintain communication
        ros::spinOnce();

        // Sleep to control the publishing rate
        loop_rate.sleep();
    }

    ros::spin();

    return 0;
}

