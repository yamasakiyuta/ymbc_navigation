#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
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
    ros::Publisher wp_pub = public_node_handle.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);
    ros::Subscriber joy_sub = public_node_handle.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);

    int cnt_ok[13]={0};
    int cnt[13]={0};
    int pre_buttons[13]={0};
    int wp_cnt=0;
    int no_pub_flag=0;
    int chmod_flag=0;
    int pub_wp_flag=0;
    geometry_msgs::PoseStamped goal_point;
    
    ros::Rate rate(50);

    while (ros::ok())
    {
        if(joy_cmd.axes.empty()){
            ROS_INFO("NO DATA\n");
            cmd_vel.linear.x=0;
        }
        else{
            // L stick
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
            
            if(chmod_flag!=1){
            cmd_vel.linear.x = joy_cmd.axes[1]*vel_x;
            cmd_vel.angular.z = joy_cmd.axes[0]*vel_w;
            }
            else{
            cmd_vel.linear.x = -(joy_cmd.axes[13]-0.5)*vel_x;
            cmd_vel.angular.z = -joy_cmd.axes[11]*vel_w;
            }
           ////////////////////stop publish cmd_vel//////////////////// 
                if(pre_buttons[12]!=joy_cmd.buttons[12])cnt_ok[12]=1;
                    if(joy_cmd.buttons[12]==1){
                        if(cnt_ok[12])cnt[12]++;
                        if(cnt[12]>20){
                            if(no_pub_flag==0){
                            no_pub_flag=1;
                            cnt[12]=0;
                            cnt_ok[12]=0;
                            ROS_INFO("NO published cmd_vel\n");
                            }
                            else{
                            no_pub_flag=0;
                            cnt[12]=0;
                            cnt_ok[12]=0;
                            ROS_INFO("published cmd_vel\n");
                            }
                        }
                    }
                    else cnt[12]=0;
                pre_buttons[12]=joy_cmd.buttons[12];

            /////////////////////////////////////////////////////////////
                if(pre_buttons[9]!=joy_cmd.buttons[9])cnt_ok[9]=1;
                    if(joy_cmd.buttons[9]==1){
                        if(cnt_ok[9])cnt[9]++;
                        if(cnt[9]>20){
                            if(chmod_flag==0){
                            chmod_flag=1;
                            cnt[9]=0;
                            cnt_ok[9]=0;
                            ROS_INFO("gyro mode\n");
                            }
                            else{
                            chmod_flag=0;
                            cnt[9]=0;
                            cnt_ok[9]=0;
                            ROS_INFO("normal mode\n");
                            }
                        }
                    }
                    else cnt[9]=0;
                pre_buttons[9]=joy_cmd.buttons[9];

            /////////////////////////////////////////////////////////////
            
            if(pub_wp_flag==0){
                if(wp_cnt<50){
                    if(joy_cmd.buttons[3]==1){
                        wp_cnt++;
                    }
                    else wp_cnt=0;
                }
                else{
                    goal_point.header.stamp=ros::Time::now();
                    goal_point.header.frame_id="odom";
                    goal_point.pose.position.x=6.0;
                    goal_point.pose.position.y=0.0;
                    goal_point.pose.position.z=0.0;
                    goal_point.pose.orientation.w=1.0;
                    wp_pub.publish(goal_point);
                    pub_wp_flag=1;
                    wp_cnt=0;
                    ROS_INFO("published way_point\n");
                }
            }
        }
        if(cmd_vel.linear.x<0)cmd_vel.angular.z*=-1;
        if(no_pub_flag!=1)vel_pub.publish(cmd_vel);
        /*if(pub_wp_flag==1){
            pub_wp_flag==0;
        }*/
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
} 
