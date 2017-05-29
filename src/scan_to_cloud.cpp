#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include<pcl/point_types.h> //pcl::PointXYZクラスを使う
#include<pcl/point_cloud.h> //pcl::PointCloudクラスを使う
#include <pcl_conversions/pcl_conversions.h>

sensor_msgs::LaserScan scan_data;
pcl::PointCloud<pcl::PointXYZ> cloud;//URGの点群を保存するクラウドを定義
sensor_msgs::PointCloud2 msg;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    scan_data = *scan_in;
    cloud.clear();
    for(int i=0;i<682;i++){
        double rad = ((double)44 + i - 384) * 0.0061; //今のステップの角度を計算

        pcl::PointXYZ p;	//１つの点pを定義
        p.x = (double)scan_data.ranges[i] * cos(rad);	//点pのX座標をURGのデータから計算(1000で割ってミリ表記からメートル表記に直す)
        p.y = (double)scan_data.ranges[i] * sin(rad);	//点pのY座標をURGのデータから計算(1000で割ってミリ表記からメートル表記に直す)
        /*if(scan_data.ranges[i]==0){ //遠すぎてレーザの点が取れなければ20mにする
            p.x=20*sei_fu(cos(rad));
            p.y=20*sei_fu(sin(rad));
        }*/
        p.z = 0.0; //点pのZ座標を計算(常に0としておく)
        
        cloud.points.push_back( p ); //作成した点pをcloudに追加する
    }
    pcl::toROSMsg (cloud, msg);
    /*printf("angle:%f~%f\n",scan_data.angle_min,scan_data.angle_max);
    printf("increment:%f\n",scan_data.angle_increment);
    printf("size:%d\n",sizeof(scan_data.ranges));
    printf("data[%d]:%f\n",1500,scan_data.ranges[1500]);*/
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_to_cloud");
    ros::NodeHandle private_node_handle("~");
    ros::NodeHandle public_node_handle;

    ros::Publisher cloud_pub = public_node_handle.advertise<sensor_msgs::PointCloud2>("cloud", 100);
    ros::Subscriber laser_sub = public_node_handle.subscribe<sensor_msgs::LaserScan>("scan", 1, scanCallback);
  
    ros::Rate rate(40);

    while (ros::ok())
    {
        msg.header.frame_id="cloud";
        msg.header.stamp=ros::Time::now();
        
        cloud_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
} 
