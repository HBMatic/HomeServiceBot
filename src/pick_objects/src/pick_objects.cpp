#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Mbc;

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_and_delivery");

  // Create a SimpleActionClient for the move_base action server
  Mbc a("move_base", true);

  // Wait for the move_base action server to start
  while(!a.waitForServer(ros::Duration(3.0))){
    ROS_INFO("Waiting for the move_base action server");
  }

  // Define the goal for picking up the object
  move_base_msgs::MoveBaseGoal pg;

  // Set the header frame ID and timestamp for the goal
  pg.target_pose.header.frame_id = "map";
  pg.target_pose.header.stamp = ros::Time::now();

  // Define the pickup goal position and orientation
  pg.target_pose.pose.position.x = 2.0;
  pg.target_pose.pose.position.y = 2.0;
  pg.target_pose.pose.orientation.w = 1.057;

  ROS_INFO("Pickup goal feeding");
  
  // Send the pickup goal to the move_base action server
  a.sendGoal(pg);

  // Wait for the robot to reach the pickup zone
  a.waitForResult();

  // Check the status of the goal
  if(a.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot reached the pickup zone");
    
  else
    ROS_INFO("Robot failed to reach the pickup zone");

  // Pause for 3 seconds
  ros::Duration(3).sleep();

  // Define the goal for dropping off the object
  move_base_msgs::MoveBaseGoal dg;

  // Set the header frame ID and timestamp for the goal
  dg.target_pose.header.frame_id = "map";
  dg.target_pose.header.stamp = ros::Time::now();

  // Define the drop off goal position and orientation
  dg.target_pose.pose.position.x = -2.0;
  dg.target_pose.pose.position.y = -2.0;
  dg.target_pose.pose.orientation.w = 1.057;

  ROS_INFO("Drop off goal feeding");
  
  // Send the drop off goal to the move_base action server
  a.sendGoal(dg);

  // Wait for the robot to reach the drop off zone
  a.waitForResult();

  // Check the status of the goal
  if(a.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot reached the drop off zone");
  else
    ROS_INFO("Robot failed to reach the drop off zone");

  return 0;
}

