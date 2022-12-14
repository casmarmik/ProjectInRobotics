#include <ros/ros.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <resource_retriever/retriever.h>
#include <ros/package.h>

void sim_plug()
{
  ros::NodeHandle nh;
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Rate loop_rate(10);
  for (int i = 0; i < 10; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    // marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.5;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.5;
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;
    marker.color.a = 0;  // Don't forget to set the alpha!
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0;
    // only if using a MESH_RESOURCE marker type:
    marker.mesh_use_embedded_materials = true;
    marker.mesh_resource = "package://" ROS_PACKAGE_NAME "/data/templates/plug_color.dae";
    vis_pub.publish(marker);
    loop_rate.sleep();
  }
}

void sim_plate()
{
  ros::NodeHandle nh;
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Rate loop_rate(10);
  for (int i = 0; i < 10; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    // marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 0;  // Don't forget to set the alpha!
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0;
    // only if using a MESH_RESOURCE marker type:
    marker.mesh_use_embedded_materials = true;
    marker.mesh_resource = "package://" ROS_PACKAGE_NAME "/data/templates/wood_plate.dae";
    vis_pub.publish(marker);
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_sim");

  sim_plate();

  return 0;
}
