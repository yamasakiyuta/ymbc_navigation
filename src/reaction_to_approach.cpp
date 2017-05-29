#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <people_tracking_filter/tracker_kalman.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/rgb.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>

#include <algorithm>
#include "tf/transform_listener.h"

tf::TransformListener tflistener;
geometry_msgs::Twist cmd_vel;
people_msgs::PositionMeasurementArray people;
geometry_msgs::PointStamped peopleRobot;
geometry_msgs::PointStamped peopleGlobal;

struct velocity
{ 
    double x;
    double w;
};

velocity people_vel;
double dist=0;
double pre_dist=0;

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

void peopleCallback (const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{
    people = *msg;
    if(people.people.empty()){
        ROS_INFO("empty\n");
        cmd_vel.linear.x=0;
    }
    else{
        for(int i=0;i<people.people.size();i++){
            double x = people.people[i].pos.x;
            double y = people.people[i].pos.y;
            peopleGlobal.point.x = x;
            peopleGlobal.point.y = y;
            peopleGlobal.point.z = 0;
            if(x>-10 && x<10 && y>-2.0 && y<-0.3){
                tflistener.transformPoint("odom",people.people[i].header.stamp,peopleGlobal,"base_link",peopleRobot);
                ROS_INFO("x=%g, y=%g\n",peopleRobot.point.x,peopleRobot.point.y);
                dist = distance(0,0,peopleRobot.point.x,peopleRobot.point.y);
                if((dist-pre_dist)>1.0){
                    ROS_INFO("go\n");
                    cmd_vel.linear.x=0.2;
                }
                else if((dist-pre_dist)<=1.0){
                    ROS_INFO("go\n");
                    cmd_vel.linear.x=0.1;
                }
                else{
                    ROS_INFO("stop\n");
                    cmd_vel.linear.x=0;
                }
                pre_dist = dist;
            }
            else cmd_vel.linear.x=0;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reaction_to_approach");
    ros::NodeHandle private_node_handle("~");
    ros::NodeHandle public_node_handle;

    ros::Publisher vel_pub = public_node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::Subscriber laser_sub = public_node_handle.subscribe<people_msgs::PositionMeasurementArray>("people_tracker_measurements", 1, peopleCallback);
  
    ros::Rate rate(10);

    while (ros::ok())
    {
        vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
} 

