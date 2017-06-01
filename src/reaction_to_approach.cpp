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
const static double range[5][2]={{5,-0.3},{5,-3.0},{0,-3.0},{0,-0.3},{5,-0.3}};

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

void peopleCallback (const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{
    people = *msg;
}

void search_range_marker(const visualization_msgs::Marker& line_strip){
}
        
int main(int argc, char **argv)
{
    ros::init(argc, argv, "reaction_to_approach");
    ros::NodeHandle private_node_handle("~");
    ros::NodeHandle public_node_handle;

    ros::Publisher vel_pub = public_node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::Publisher search_range_pub = public_node_handle.advertise<visualization_msgs::Marker>("search_range_marker", 10);
    ros::Publisher tracking_pub = public_node_handle.advertise<visualization_msgs::Marker>("my_tracking_marker", 10);
    ros::Subscriber laser_sub = public_node_handle.subscribe<people_msgs::PositionMeasurementArray>("people_tracker_measurements", 1, peopleCallback);
    
    tf::TransformListener listener;
  
    ros::Rate rate(10);

    while (ros::ok())
    {
        ////////////visualization/////////////////////////////////////////
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "/odom";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "lines";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;

        line_strip.id = 1;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.1;
        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
        for(int i=0;i<5;i++){
            geometry_msgs::Point p;
            p.x = range[i][0];
            p.y = range[i][1];
            p.z = 0;
            line_strip.points.push_back(p);
        }
        search_range_pub.publish(line_strip);
        //////////////////////////////////////////////
        visualization_msgs::Marker points;
        points.header.frame_id = "/odom";
        points.header.stamp = ros::Time::now();
        points.ns = "points";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.5;
        points.scale.y = 0.5;
        // Line strip is red
        points.color.r = 1.0;
        points.color.a = 1.0;
        

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
        //ROS_INFO("%d\n",people.people.size());
        for(int i=0;i<=people.people.size();i++){
            double x = people.people[i].pos.x;
            double y = people.people[i].pos.y;
            double xGL = x+transform.getOrigin().x();
            double yGL = y+transform.getOrigin().y();
            ROS_INFO("%g, %g\n",x,y);
            if(xGL>range[2][0] && xGL<range[0][0] && yGL>range[1][1] && yGL<range[3][1]){
            //if(x>-5 && x<5 && y>-2.0 && y<-0.3){
                dist = distance(0,0,x,y);
                geometry_msgs::Point p;
                p.x = xGL;
                p.y = yGL;
                p.z = 0;
                points.points.push_back(p);
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
                cmd_vel.linear.x=-0.2;
                break;
            }
            else{
                cmd_vel.linear.x=0;
                ROS_INFO("stop(no legs)\n");
            }
        }
        tracking_pub.publish(points);
    }
        
        vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
} 

