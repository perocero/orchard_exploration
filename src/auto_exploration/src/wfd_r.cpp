#include<iostream>
#include<ros/ros.h>
#include<ros/console.h>
#include<nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include<sensor_msgs/PointCloud.h>
#include<nav_msgs/Odometry.h>
#include "auto_exploration/array2.h"
#include <vector>

float distance(int x, int y, float boat_x, float boat_y);
int xy_idx(float x, float y);
std::vector<int> ex_points;

#define array_x 10000

float origin_x = 0.0,origin_y = 0.0;
int w = 0, h = 0;
int  array[array_x][array_x];
 float limit = 1.5;
float ix = 0,iy = 0;
float box = 0.0;
float box_x = 0,box_y = 0;

float boat_x = 0.0, boat_y = 0.0;

int fi = 0;
int flag = 0;
struct foriter              //储存边界点结构体
{
    float fx;
    float fy;
}points[100000];


nav_msgs::OccupancyGrid mapData;

void odominfoCallBack(const nav_msgs::Odometry::ConstPtr& odom)
{
    boat_x = odom->pose.pose.position.x;
    boat_y = odom->pose.pose.position.y;
    std::cout << "boat_x = " << boat_x << "    boat_y = " << boat_y << '\n' << std::endl;
 }

void mapinfoCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("map input");                                      //地图数据输入
        mapData=*msg;

    w = msg->info.width;                      
    h = msg->info.height;
    origin_x = msg->info.origin.position.x;
    origin_y = msg->info.origin.position.y;
    box = msg->info.resolution;
    float dis = 0.0;

    std::cout << "width:  " << w << "  height:  " << h  << "  origin x:  " << origin_x << "  origin y:  " << origin_y << "  resolution:  " << box << std::endl;
    
    std::cout << "building points" << std::endl;  

    ex_points.clear();
    fi = 0;      //点云数量标志位更新
    for(int num=0;num<(w*h);num++)
    {
        if(msg->data[num] == 0)
        {
             if(msg->data[num-1] == (-1)||msg->data[num+1]==(-1)||msg->data[num+w]==(-1)||msg->data[num-w]==(-1)) //四邻域
         {
            ix = num/w;
            iy = num%w;
            box_x = ix*box+origin_y;
            box_y = iy*box+origin_x;

            //  dis = distance(box_x, box_y, boat_y, boat_x) ;
            //  //std::cout << "dis = " << dis << '\n' << std::endl;
            //   if(dis <= 4.0)
            //   {
                points[fi].fy = box_x;
                points[fi].fx = box_y;

                ex_points.push_back(xy_idx(box_y, box_x));
                fi++;
            // }
         }
        }
    }
    flag = 1;

}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"readmap");
    ros::NodeHandle rm;

    ros::Publisher pub_point = rm.advertise<sensor_msgs::PointCloud>("wfd_r_point",10);
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "map";
    cloud.points.resize(100000);                                            //点云设置数量,给消息中的数组确定大小
    //cloud.channels.resize(1);
    //cloud.channels[0].name = "intensities";
    //cloud.channels[0].values.resize(10000);

    ros::Subscriber sub = rm.subscribe<nav_msgs::OccupancyGrid>("map",10,mapinfoCallBack);
    ros::Subscriber sub_boat_position = rm.subscribe<nav_msgs::Odometry>("odom",10,odominfoCallBack);
    ros::Publisher pub_f = rm.advertise<auto_exploration::array2>("firefly_rrt", 100);

    ros::Rate loop_rate(10);//接受频率
    while(ros::ok())
    {
        auto_exploration::array2 firefly_data;
        ros::spinOnce(); //队列中有数据时循环一次回调函数
        //ros::spin(); //回调死循环
        
        for(int j=0;j<fi;j++)                                                           //边界点坐标储存在点云信息中等待发布
        {
                cloud.points[j].x = points[j].fx;
                cloud.points[j].y = points[j].fy;
                cloud.points[j].z = 0.5;
                //cloud.channels[0].values[j] = 100*j;
        }
        pub_point.publish(cloud);
        //std::cout  << "cloud is updating" <<  "  fi:  " << fi << std::endl;

        if(flag == 1)
        {
            for(int k=0;k<100000;k++)                                                           //边界点坐标更新
            {
                cloud.points[k].x = 0;
                cloud.points[k].y = 0;
                cloud.points[k].z = 0;
                //cloud.channels[0].values[j] = 100*j;
            }
            flag = 0;      //点云数量标志位更新
        }

            firefly_data.data[0] = ex_points.size();
            for(int i = 0;i<ex_points.size();i++)
            {
                firefly_data.data[i+1] = ex_points[i];
            }
            pub_f.publish(firefly_data);
          
        loop_rate.sleep();     //接受频率
    }

    return 0;
}

float distance(int x, int y, float boat_x, float boat_y)
{
    float out = 0.0;
    out  = pow(	(pow((x-boat_x),2)+pow((y-boat_y),2)),0.5);
    return out;
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
