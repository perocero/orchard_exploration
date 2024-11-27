#include<iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <cstdlib>
#include "auto_exploration/best_goal.h"
#include<nav_msgs/Odometry.h>
#include <cmath>
#include<tf/tf.h>
#include "nav_msgs/OccupancyGrid.h"
#include<geometry_msgs/Twist.h>         //线速度角速度库

void idx_xy(int idx);
float distance(int x1, int x2);
int xy_idx(float x, float y);

float boat_x, boat_y;
int boat_idx;
float roll = 0.0, pitch = 0.0, yaw = 0.0;

int filter_flag, filter_point;
float xy_pose[2];

int n_flag = 0;

int time_flag;

int odom_flag = 0;

float dis = 0.0;

int wi = 0;


nav_msgs::OccupancyGrid mapData;
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    mapData = *msg;
    wi = msg->info.width; 
}

void filtercallback(const auto_exploration::best_goal::ConstPtr& filter_data)
{   
        filter_flag = 1;
        if(n_flag == 0)
        {
            filter_point = filter_data->idx;
        }

    //std::cout << "filter num = " << filter_num << '\n' << std::endl;
}

void odominfoCallBack(const nav_msgs::Odometry::ConstPtr& odom)
{
    odom_flag = 1;
    float ix_o = 0.0,iy_o = 0.0;
    boat_x = odom->pose.pose.position.x;
    boat_y = odom->pose.pose.position.y;
    boat_idx = xy_idx(boat_x, boat_y);
    // tf::Quaternion quat;
    // tf::quaternionMsgToTF(odom->pose.pose.orientation,quat);
    // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //std::cout << "boat_x = " << boat_x << "boat_y = " << boat_y << '\n' << std::endl;
}

int  main(int argc,char **argv)
{
    ros::init(argc,argv,"pub_goal");

    ros::NodeHandle pg;

    ros::Subscriber sub_firefly = pg.subscribe<auto_exploration::best_goal>("best_auto_exploration", 10, filtercallback);
    ros::Subscriber sub= pg.subscribe<nav_msgs::OccupancyGrid>("map", 10000 ,mapCallBack);
    ros::Subscriber sub_boat_position = pg.subscribe<nav_msgs::Odometry>("odom", 10000, odominfoCallBack);

    ros::Publisher turtle_vel_pub = pg.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    ros::Publisher point_pub = pg.advertise<geometry_msgs::PointStamped>("point",1,true);
    ros::Publisher goal_pub = pg.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);

    ros::Time current_time;
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.1;

    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float a = 0.0;

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        current_time = ros::Time::now();
        int dalay_flag;
        //std::cout << "flag!!" <<  '\n' << std::endl;
        ros::spinOnce();

        if(filter_flag == 1)
        {
            idx_xy(filter_point);
            x = xy_pose[0];
            y = xy_pose[1];

            a = yaw;
            a = a/180*3.14;

            geometry_msgs::PoseStamped pose_stamped;
            geometry_msgs::PointStamped point_stamped;

            pose_stamped.pose.position.x = x;
            pose_stamped.pose.position.y = y;
            pose_stamped.pose.position.z = z;

            point_stamped.header.stamp = current_time;
            point_stamped.header.frame_id = "map";
            point_stamped.point.x = x;
            point_stamped.point.y = y;
            point_stamped.point.z = z;
            geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(a);    //四元数转换定义
            pose_stamped.pose.orientation.x = goal_quat.x;
            pose_stamped.pose.orientation.y = goal_quat.y;
            pose_stamped.pose.orientation.z = goal_quat.z;
            pose_stamped.pose.orientation.w = goal_quat.w;

            pose_stamped.header.stamp = current_time;
            pose_stamped.header.frame_id = "map";

            point_pub.publish(point_stamped);
            goal_pub.publish(pose_stamped);

            time_flag = 0;
            // while(time_flag == 0)
            // {
            //     //time_flag++;
            //     std::cout << "going goal" << '\n' << std::endl;
            //     std::cout << "input someone go on" << '\n' << std::endl;
            //     std::cin >> time_flag;
            // }
            std::cout << "start next goal" << '\n' << std::endl;

            dis = pow(pow(boat_x - x, 2) + pow(boat_y - y, 2), 0.5);
            while(distance(boat_idx, filter_point) > 0.5)
            {                
                n_flag = 1;
                //std::cout << "going to goal !" << '\n' << std::endl;
                std::cout << "dis = " << dis  <<  '\n' <<  std::endl;

                //std::cout << "x = " << x << "y = " << y << '\n' << std::endl;
                ros::spinOnce();
                dis = pow(pow(boat_x - x, 2) + pow(boat_y - y, 2), 0.5);

                time_flag++;
                std::cout << "time: " << 5000000 - time_flag << std::endl;
                if(time_flag == 5000000) 
                {
                    std::cout << "time out !" << 5000000 - time_flag << std::endl;
                    break;
                }
            }

            for(int count = 0; count < 1000000; count++)
            {
                std::cout  << "wait !!!" <<  '\n' << std::endl;
            }
            n_flag = 0;
        }
        odom_flag = 0;
        
        filter_flag = 0;
        loop_rate.sleep();
    }

    return 0;
}

void idx_xy(int idx)
{
    float w = mapData.info.width, h = mapData.info.width;
    float box = mapData.info.resolution;
    float o_x = mapData.info.origin.position.x, o_y = mapData.info.origin.position.y;
    float ix = 0.0, iy = 0.0;
    // while(mapData.info.width == 0)
    // {
    //     std::cout << "wait map update" << '\n' << std::endl;
    // }
    // ix = idx/mapData.info.width;
    // iy = idx%mapData.info.width;

    ix = idx/wi;
    iy = idx%wi;
        //std::cout << "!!!" << std::endl;
    xy_pose[1] = ix*box+o_y;
    xy_pose[0] = iy*box+o_x;
}

float distance(int x1, int x2)
{    
    float x_v[2];
    float start_v[2];
    idx_xy(x1);
    x_v[0] = xy_pose[0]; x_v[1] = xy_pose[1];
    idx_xy(x2);
    start_v[0] = xy_pose[0]; start_v[1] = xy_pose[1];
    return (pow(pow((x_v[0]-start_v[0]),2)+pow((x_v[1]-start_v[1]),2), 0.5));
}

int xy_idx(float x, float y)
{
    float w = 0.0, box = 0.0, o_x = 0.0, o_y = 0.0;
     w = mapData.info.width;
     box = mapData.info.resolution;
     o_x = mapData.info.origin.position.x; o_y = mapData.info.origin.position.y;
    //std::cout << o_x << o_y << std::endl;
    float indx =(  floor((y-o_y)/box)*w)+( floor((x-o_x)/box) );
    int out  = int (indx);
    return out;
}