#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Mbc;

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_and_delivery");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher dmarker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::CUBE;
 
 while(ros::ok)
 {
  // Create a SimpleActionClient for the move_base action server
  Mbc a("move_base", true);
  visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "object";
    marker.id = 0;

    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 2;
    marker.pose.position.y = 2;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0f;

    visualization_msgs::Marker dmarker;
    dmarker.header.frame_id = "map";
    dmarker.header.stamp = ros::Time::now();

    dmarker.ns = "object";
    dmarker.id = 1;

    dmarker.type = shape;
    dmarker.action = visualization_msgs::Marker::ADD;

    dmarker.pose.position.x = -2;
    dmarker.pose.position.y = -2;
    dmarker.pose.position.z = 0;
    dmarker.pose.orientation.x = 0.0;
    dmarker.pose.orientation.y = 0.0;
    dmarker.pose.orientation.z = 0.0;
    dmarker.pose.orientation.w = 1.0;

    dmarker.scale.x = 0.2;
    dmarker.scale.y = 0.2;
    dmarker.scale.z = 0.2;

    dmarker.color.r = 0.5f;
    dmarker.color.g = 0.5f;
    dmarker.color.b = 0.5f;
    dmarker.color.a = 1.0f;

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

  ROS_INFO("Task Begin");
  ROS_INFO("Object identified by TurtleBot");
  marker_pub.publish(marker);
  
  // Send the pickup goal to the move_base action server
  a.sendGoal(pg);

  // Wait for the robot to reach the pickup zone
  a.waitForResult();

  // Check the status of the goal
  if(a.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
   { ROS_INFO("Robot reached the Object");
    marker.action = visualization_msgs::Marker::DELETE;
    ros::Duration(5).sleep();
    marker_pub.publish(marker);
    ROS_INFO("Robot picked up the Object");
   }
  else
    ROS_INFO("Robot failed to reach the pickup zone");

  // Pause for 3 seconds
  ros::Duration(1).sleep();

  // Define the goal for dropping off the object
  move_base_msgs::MoveBaseGoal dg;

  // Set the header frame ID and timestamp for the goal
  dg.target_pose.header.frame_id = "map";
  dg.target_pose.header.stamp = ros::Time::now();

  // Define the drop off goal position and orientation
  dg.target_pose.pose.position.x = -2.0;
  dg.target_pose.pose.position.y = -2.0;
  dg.target_pose.pose.orientation.w = -1.057;

  ROS_INFO("Drop off goal feeding");
  
  // Send the drop off goal to the move_base action server
  a.sendGoal(dg);

  // Wait for the robot to reach the drop off zone
  a.waitForResult();

  // Check the status of the goal
  if(a.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
     ROS_INFO("Robot reached the drop off zone");
     ros::Duration(2).sleep();
     dmarker_pub.publish(dmarker);
     ROS_INFO("Robot dropped off the object");
    }
  else
    ROS_INFO("Robot failed to reach the drop off zone");
   ros::Duration(2).sleep();
   ROS_INFO("Task Over");
  return 0;
}
}
