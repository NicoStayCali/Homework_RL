#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include "Eigen/Dense"
#include <tf/transform_broadcaster.h>


#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Global variables
//std::vector<double> aruco_pose(7,0.0);
geometry_msgs::PoseStamped aruco_pose;
bool aruco_pose_available = false;

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)
{
  aruco_pose_available = true;
  aruco_pose.header.stamp=ros::Time::now();
  aruco_pose.header.frame_id="aruco_pose_our";
  aruco_pose.pose.position.x=(msg.pose.position.x);
  aruco_pose.pose.position.y=(msg.pose.position.y);
  aruco_pose.pose.position.z=(msg.pose.position.z);          
  aruco_pose.pose.orientation.x=(msg.pose.orientation.x);
  aruco_pose.pose.orientation.y=(msg.pose.orientation.y);
  aruco_pose.pose.orientation.z=(msg.pose.orientation.z);
  aruco_pose.pose.orientation.w=(msg.pose.orientation.w);

}

int main(int argc, char** argv){
  ros::init(argc, argv, "aruco_navigation_goals");
  ros::NodeHandle n;

  // Rate
  ros::Rate loop_rate(500);

  // Subscribers
  ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  tf::TransformListener listener;
  tf::StampedTransform transform0,transform1,transform2, transform3,transform4,transform5, transform6, transform7, transform8;
  tf::Transform Optical_to_base, Optical_to_map;

  try{
    // listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
    // listener.lookupTransform( "map", "base_footprint", ros::Time(0), transform0 );

    listener.waitForTransform( "base_footprint", "base_link", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform( "base_footprint", "base_link", ros::Time(0), transform1 );

    listener.waitForTransform( "base_link", "d435_link", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform( "base_link", "d435_link", ros::Time(0), transform2 );

    listener.waitForTransform( "d435_link", "camera_bottom_screw_frame", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform( "d435_link", "camera_bottom_screw_frame", ros::Time(0), transform3 );

    listener.waitForTransform( "camera_bottom_screw_frame", "camera_link", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform( "camera_bottom_screw_frame", "camera_link", ros::Time(0), transform4 );

    listener.waitForTransform( "camera_link", "camera_depth_frame", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform( "camera_link", "camera_depth_frame", ros::Time(0), transform5 );

    listener.waitForTransform( "camera_depth_frame", "camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform( "camera_depth_frame", "camera_depth_optical_frame", ros::Time(0), transform6 );

    // listener.waitForTransform( "camera_depth_optical_frame", "aruco_marker_frame", ros::Time(0), ros::Duration(10.0) );
    // listener.lookupTransform( "camera_depth_optical_frame", "aruco_marker_frame", ros::Time(0), transform7 );

  }catch(tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  
  Optical_to_base = transform1 * transform2 * transform3* transform4 * transform5 * transform6;
  
  Optical_to_map = transform0 * Optical_to_base;
  // ROS_INFO("Camera pose in the global pose: x=%f, y=%f, <=%f",
  //           cam_to_map.getOrigin().x(),cam_to_map.getOrigin().y(),cam_to_map.getOrigin().z());

  move_base_msgs::MoveBaseGoal goal;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
 
  goal.target_pose.pose.position.x = -14.0;
  goal.target_pose.pose.position.y = 7.0;
  goal.target_pose.pose.orientation.z = 1.0;
  goal.target_pose.pose.orientation.w = 0.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // while(ros::ok()){
    while(!aruco_pose_available){
      ROS_INFO("Aruco non disponibile");
      ros::spinOnce();
    }
    
    // //reset se non lo vede
    // aruco_pose_available=false;
    
    tf::Transform msgTransform;
    msgTransform.setOrigin(tf::Vector3(aruco_pose.pose.position.x,aruco_pose.pose.position.y,aruco_pose.pose.position.z));
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(aruco_pose.pose.orientation,quaternion);
    msgTransform.setRotation(quaternion);
    
    try{
      listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
      listener.lookupTransform( "map", "base_footprint", ros::Time(0), transform0 );

    }catch(tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    
    msgTransform = transform0 * Optical_to_base * msgTransform;
  
    ROS_INFO("Aruco Marker pose in the global pose: x=%f, y=%f, z=%f",
                msgTransform.getOrigin().x(),msgTransform.getOrigin().y(),msgTransform.getOrigin().z());
    
    // aruco goal
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
 
    goal.target_pose.pose.position.x = msgTransform.getOrigin().x() + 1;
    goal.target_pose.pose.position.y = msgTransform.getOrigin().y();
    goal.target_pose.pose.orientation.z = 1.0;
    goal.target_pose.pose.orientation.w = 0.0;

    ROS_INFO("Sending goal");
    ROS_INFO("Goal: x=%f, y=%f",
                msgTransform.getOrigin().x()+1,msgTransform.getOrigin().y());
    ac.sendGoal(goal);

    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The mobile robot arrived in the TF goal");
    else
      ROS_INFO("The base failed to move for some reason");

    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(msgTransform,ros::Time::now(),"map","aruco_marker_frame"));

    ROS_INFO("TF published");

    //ros::spinOnce();
  
  // }

  return 0;
 }