#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher dmarker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
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

    
    
    ROS_INFO("Start");
    ros::Duration(2).sleep();
    marker_pub.publish(marker);
    ROS_INFO("Marker Placed at Pickup Zone");
    ros::Duration(5).sleep();
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ROS_INFO("Marker Removed from Pickup Zone");
    ros::Duration(5).sleep();
    dmarker_pub.publish(dmarker);
    ROS_INFO("Marker Placed at Dropoff Zone");
    ros::Duration(5).sleep();
    dmarker.action = visualization_msgs::Marker::DELETE;
    dmarker_pub.publish(dmarker);
    ROS_INFO("Marker Removed from Dropoff Zone");
    ros::Duration(2).sleep();
    ROS_INFO("Stop");
    ros::shutdown();
    r.sleep();
  }
}

