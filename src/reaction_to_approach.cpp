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
double pre_x=0;
const static double x_thresh=0.005;
const static double vel_x=0.6;

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

void peopleCallback (const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{
    people = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reaction_to_approach");
    ros::NodeHandle private_node_handle("~");
    ros::NodeHandle public_node_handle;

    ros::Publisher vel_pub = public_node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::Subscriber laser_sub = public_node_handle.subscribe<people_msgs::PositionMeasurementArray>("people_tracker_measurements", 1, peopleCallback);
    
    tf::TransformListener listener;
  
    ros::Rate rate(10);

    while (ros::ok())
    {
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        //ROS_INFO("x=%g\n",transform.getOrigin().x());
    if(people.people.empty()){
        ROS_INFO("stop(empty)\n");
        cmd_vel.linear.x=0;
    }
    else{
        ROS_INFO("%d\n",people.people.size());
        for(int i=0;i<=people.people.size();i++){
            double x = people.people[i].pos.x;
            double y = people.people[i].pos.y;
            double xGL = x+transform.getOrigin().x();
            double yGL = y+transform.getOrigin().y();
            ROS_INFO("%g, %g\n",x,y);
            if(xGL>-5 && xGL<5 && yGL>-2.0 && yGL<-0.3){
            //if(x>-5 && x<5 && y>-2.0 && y<-0.3){
                dist = distance(0,0,x,y);
                //ROS_INFO("dist=%g\n",dist);
                //ROS_INFO("x=%g\n",transform.getOrigin().x());
                //ROS_INFO("y=%g\n",transform.getOrigin().y());
                if((dist-pre_dist)>x_thresh){
                    ROS_INFO("go\n");
                    cmd_vel.linear.x=vel_x;
                }
                else{
                    ROS_INFO("stop\n");
                    cmd_vel.linear.x=0;
                }
                /*if(x>0){//人がロボットより前にいる時
                //if(x>transform.getOrigin().x()){
                    ROS_INFO("front\n");
                    if((x-pre_x)>x_thresh){//遠ざかっていたら進む
                        ROS_INFO("go\n");
                        cmd_vel.linear.x=vel_x;
                    }
                    else{
                        //ROS_INFO("stop\n");
                        cmd_vel.linear.x=0;
                    }
                }
                else{//人がロボットより後ろにいる時
                    ROS_INFO("behind\n");
                    if((x-pre_x)<-x_thresh){
                        ROS_INFO("go\n");
                        cmd_vel.linear.x=vel_x;
                    }
                    else{
                        //ROS_INFO("stop\n");
                        cmd_vel.linear.x=0;
                    }
                    //cmd_vel.linear.x=0;
                }
                ROS_INFO("x:%g, pre_x:%g\n",x, pre_x);
                ROS_INFO("%g\n",x-pre_x);*/
                ROS_INFO("dist:%g, pre_dist:%g\n",dist, pre_dist);
                ROS_INFO("%g\n",dist-pre_dist);
                pre_dist = dist;
                pre_x = x;
                break;
            }
            else if(x>0 && x<2 && y>-0.5 && y<0.5){
                cmd_vel.linear.x=-vel_x;
                break;
            }
            else{
                cmd_vel.linear.x=0;
                ROS_INFO("stop(no legs)\n");
                break;
            }
        }
    }
        
        vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
} 

