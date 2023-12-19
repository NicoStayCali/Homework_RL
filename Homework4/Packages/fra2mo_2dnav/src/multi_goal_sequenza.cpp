#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }


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
  
  // ROS_INFO("Goal 1 ---- \n Position --> x: %f  y: %f z: %f, \n Orientation --> x.or: %f, y.or: %f, z.or: %f, w.or: %f ",
  //         transform1.getOrigin().x(), transform1.getOrigin().y(), transform1.getOrigin().z(),
  //         transform1.getRotation().x(), transform1.getRotation().y(), transform1.getRotation().z(), transform1.getRotation().w());
  // ROS_INFO("Goal 2 ---- \n Position --> x: %f  y: %f z: %f, \n Orientation --> x.or: %f, y.or: %f, z.or: %f, w.or: %f ",
  //         transform2.getOrigin().x(), transform2.getOrigin().y(), transform2.getOrigin().z(),
  //         transform2.getRotation().x(), transform2.getRotation().y(), transform2.getRotation().z(), transform2.getRotation().w());
  // ROS_INFO("Goal 3 ---- \n Position --> x: %f  y: %f z: %f, \n Orientation --> x.or: %f, y.or: %f, z.or: %f, w.or: %f ",
  //         transform3.getOrigin().x(), transform3.getOrigin().y(), transform3.getOrigin().z(),
  //         transform3.getRotation().x(), transform3.getRotation().y(), transform3.getRotation().z(), transform3.getRotation().w());
  // ROS_INFO("Goal 4 ---- \n Position --> x: %f  y: %f z: %f, \n Orientation --> x.or: %f, y.or: %f, z.or: %f, w.or: %f ",
  //         transform4.getOrigin().x(), transform4.getOrigin().y(), transform4.getOrigin().z(),
  //         transform4.getRotation().x(), transform4.getRotation().y(), transform4.getRotation().z(), transform4.getRotation().w());
  
  // DEFINISCO I GOAL
  move_base_msgs::MoveBaseGoal goal1, goal2, goal3, goal4;
  
  //goal1
  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.header.stamp = ros::Time::now();
  
  goal1.target_pose.pose.position.x = transform1.getOrigin().x();
  goal1.target_pose.pose.position.y = transform1.getOrigin().y();
  goal1.target_pose.pose.position.z = transform1.getOrigin().z();
  goal1.target_pose.pose.orientation.x = transform1.getRotation().x();
  goal1.target_pose.pose.orientation.y = transform1.getRotation().y();
  goal1.target_pose.pose.orientation.z = transform1.getRotation().z();
  goal1.target_pose.pose.orientation.w = transform1.getRotation().w();
  
  //goal2
  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.header.stamp = ros::Time::now();
  
  goal2.target_pose.pose.position.x = transform2.getOrigin().x();
  goal2.target_pose.pose.position.y = transform2.getOrigin().y();
  goal2.target_pose.pose.position.z = transform2.getOrigin().z();
  goal2.target_pose.pose.orientation.x = transform2.getRotation().x();
  goal2.target_pose.pose.orientation.y = transform2.getRotation().y();
  goal2.target_pose.pose.orientation.z = transform2.getRotation().z();
  goal2.target_pose.pose.orientation.w = transform2.getRotation().w();
  
  //goal3
  goal3.target_pose.header.frame_id = "map";
  goal3.target_pose.header.stamp = ros::Time::now();
  
  goal3.target_pose.pose.position.x = transform3.getOrigin().x();
  goal3.target_pose.pose.position.y = transform3.getOrigin().y();
  goal3.target_pose.pose.position.z = transform3.getOrigin().z();
  goal3.target_pose.pose.orientation.x = transform3.getRotation().x();
  goal3.target_pose.pose.orientation.y = transform3.getRotation().y();
  goal3.target_pose.pose.orientation.z = transform3.getRotation().z();
  goal3.target_pose.pose.orientation.w = transform3.getRotation().w();
  
  //goal4
  goal4.target_pose.header.frame_id = "map";
  goal4.target_pose.header.stamp = ros::Time::now();
  
  goal4.target_pose.pose.position.x = transform4.getOrigin().x();
  goal4.target_pose.pose.position.y = transform4.getOrigin().y();
  goal4.target_pose.pose.position.z = transform4.getOrigin().z();
  goal4.target_pose.pose.orientation.x = transform4.getRotation().x();
  goal4.target_pose.pose.orientation.y = transform4.getRotation().y();
  goal4.target_pose.pose.orientation.z = transform4.getRotation().z();
  goal4.target_pose.pose.orientation.w = transform4.getRotation().w();

  int scelta=0;

  //Invio i goal --> Sequenza 3,4,2,1 SIUM
  ROS_INFO("Sending goal3");
  ac.sendGoal(goal3);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("goal3 confirmed");
  else
    ROS_INFO("goal3 failed to move for some reason");
  
  //
  ROS_INFO("Sending goal4");
  ac.sendGoal(goal4);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("goal4 confirmed");
  else
    ROS_INFO("goal4 failed to move for some reason");
  
  //
  ROS_INFO("Sending goal2");
  ac.sendGoal(goal2);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("goal2 confirmed");
  else
    ROS_INFO("goal2 failed to move for some reason");
  
  //
  ROS_INFO("Sending goal1");
  ac.sendGoal(goal1);
  
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("goal1 confirmed");
  else
    ROS_INFO("goal1 failed to move for some reason");

  return 0;
 }