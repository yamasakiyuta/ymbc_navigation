#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include <map>
#include <string>

sensor_msgs::Joy joy_cmd;
geometry_msgs::Twist cmd_vel;
double vel_x=0.2;
double vel_w=0.2;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    joy_cmd = *joy;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spur_ps4joy");
    ros::NodeHandle private_node_handle("~");
    ros::NodeHandle public_node_handle;

    ros::Publisher vel_pub = public_node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::Subscriber joy_sub = public_node_handle.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);

    int cnt=0;
    int no_pub_flag=0;
    
    ros::Rate rate(50);

    while (ros::ok())
    {
        if(joy_cmd.axes.empty()){
            ROS_INFO("stop(empty)\n");
            cmd_vel.linear.x=0;
        }
        else{
            //ROS_INFO("%g\n",cmd_vel.linear.x);
            if(joy_cmd.buttons[4]==1){
                vel_x+=0.01;
                if(vel_x>2.0)vel_x=2.0;
                ROS_INFO("x:%g, w:%g\n",vel_x, vel_w);
            }
            else if(joy_cmd.buttons[6]==1){
                vel_x-=0.01;
                if(vel_x<0.0)vel_x=0.0;
                ROS_INFO("x:%g, w:%g\n",vel_x, vel_w);
            }
            
            if(joy_cmd.buttons[5]==1){
                vel_w+=0.01;
                if(vel_w>20.0)vel_w=20.0;
                ROS_INFO("x:%g, w:%g\n",vel_x, vel_w);
            }
            else if(joy_cmd.buttons[7]==1){
                vel_w-=0.01;
                if(vel_w<0.0)vel_w=0.0;
                ROS_INFO("x:%g, w:%g\n",vel_x, vel_w);
            }
            cmd_vel.linear.x = joy_cmd.axes[1]*vel_x;
            cmd_vel.angular.z = joy_cmd.axes[0]*vel_w;

            if(no_pub_flag==0){
                if(cnt<50){
                    if(joy_cmd.buttons[12]==1){
                        cnt++;
                    }
                    else cnt=0;
                }
                else{
                    no_pub_flag=1;
                    cnt=0;
                    ROS_INFO("NO published cmd_vel\n");
                }
            }
            else{
                if(cnt<50){
                    if(joy_cmd.buttons[12]==1){
                        cnt++;
                    }
                    else cnt=0;
                }
                else{
                    no_pub_flag=0;
                    cnt=0;
                    ROS_INFO("published cmd_vel\n");
                }
            }
        }
        if(cmd_vel.linear.x<0)cmd_vel.angular.z*=-1;
        if(no_pub_flag!=1)vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
} 
