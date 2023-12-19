#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include "boost/thread.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 
int main(int argc, char** argv){
  ros::init(argc, argv, "tf_debug");

  tf::TransformListener listener;
  tf::StampedTransform transform1,transform2,transform3,transform4;
  ros::Rate r( 1 );

  try{
    listener.waitForTransform( "map", "goal1", ros::Time( 0 ), ros::Duration( 10.0 ) );
    listener.lookupTransform( "map", "goal1", ros::Time( 0 ), transform1 );

    listener.waitForTransform( "map", "goal2", ros::Time( 0 ), ros::Duration( 10.0 ) );
    listener.lookupTransform( "map", "goal2", ros::Time( 0 ), transform2 );

    listener.waitForTransform( "map", "goal3", ros::Time( 0 ), ros::Duration( 10.0 ) );
    listener.lookupTransform( "map", "goal3", ros::Time( 0 ), transform3 );

    listener.waitForTransform( "map", "goal4", ros::Time( 0 ), ros::Duration( 10.0 ) );
    listener.lookupTransform( "map", "goal4", ros::Time( 0 ), transform4 );
  }
  catch( tf::TransformException &ex )
  {
      ROS_ERROR("%s", ex.what());
      r.sleep();
  }
  
  ROS_INFO("Goal 1 ---- \n Position --> x: %f  y: %f z: %f, \n Orientation --> x.or: %f, y.or: %f, z.or: %f, w.or: %f ",
          transform1.getOrigin().x(), transform1.getOrigin().y(), transform1.getOrigin().z(),
          transform1.getRotation().x(), transform1.getRotation().y(), transform1.getRotation().z(), transform1.getRotation().w());
  ROS_INFO("Goal 2 ---- \n Position --> x: %f  y: %f z: %f, \n Orientation --> x.or: %f, y.or: %f, z.or: %f, w.or: %f ",
          transform2.getOrigin().x(), transform2.getOrigin().y(), transform2.getOrigin().z(),
          transform2.getRotation().x(), transform2.getRotation().y(), transform2.getRotation().z(), transform2.getRotation().w());
  ROS_INFO("Goal 3 ---- \n Position --> x: %f  y: %f z: %f, \n Orientation --> x.or: %f, y.or: %f, z.or: %f, w.or: %f ",
          transform3.getOrigin().x(), transform3.getOrigin().y(), transform3.getOrigin().z(),
          transform3.getRotation().x(), transform3.getRotation().y(), transform3.getRotation().z(), transform3.getRotation().w());
  ROS_INFO("Goal 4 ---- \n Position --> x: %f  y: %f z: %f, \n Orientation --> x.or: %f, y.or: %f, z.or: %f, w.or: %f ",
          transform4.getOrigin().x(), transform4.getOrigin().y(), transform4.getOrigin().z(),
          transform4.getRotation().x(), transform4.getRotation().y(), transform4.getRotation().z(), transform4.getRotation().w());
    
    

  return 0;
 }