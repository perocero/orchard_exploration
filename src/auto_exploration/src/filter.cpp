#include<iostream>
#include<ros/ros.h>
#include<tf/tf.h>
#include <vector>
#include <cmath>
#include<nav_msgs/Odometry.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "auto_exploration/array2.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "auto_exploration/array2.h"
#include "auto_exploration/best_goal.h"
#include <queue>

float pi = 3.14159265;

float boat_x = 0.0, boat_y = 0.0;
//float b_x = 0.0, b_y = 0.0, b_z = 0.0, b_w = 0.0;
double roll = 0.0, pitch = 0.0, yaw = 0.0;

float xy_pose[2];
int boat_idx;

void idx_xy(int idx);
int xy_idx(float x, float y);
float distance(int x1, int x2);

std::vector<int> firefly_point;
int firefly_num = 0;
int firefly_flag = 0;

float ri = 1.0;            //1
float rd = 1.0;            //5      1    1.5  10
float rb = 0.0;          //50   500
float ra = 0.0;         //100

int inf_value;
int inf_bar;
int bar_value;
int inf_auto_exploration;

int num_p = 0;

 //std::vector<float> I;          //信息增益
// std::vector<float> D;        //距离代价
// std::vector<float> B;       //障碍物/定位精度
// std::vector<float> A;       //角度代价
std::vector<float> R;

int information(int check_idx);
int barrier(int check_idx);
float angle(int bo_idx, int check_idx);
float h(int bo_idx, int check_idx);
int check_edge(int check_idx);

visualization_msgs::Marker points,line;
geometry_msgs::PointStamped auto_exploration_goal;

nav_msgs::OccupancyGrid mapData;
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    mapData = *msg;
}

void odominfoCallBack(const nav_msgs::Odometry::ConstPtr& odom)
{
    float ix_o = 0.0,iy_o = 0.0;
    boat_x = odom->pose.pose.position.x;
    boat_y = odom->pose.pose.position.y;
    boat_idx = xy_idx(boat_x, boat_y);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom->pose.pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //std::cout << "yaw = " << yaw << std::endl;
}

void fireflycallback(const auto_exploration::array2::ConstPtr& firefly_data)
{
    firefly_flag = 1;
    firefly_num = firefly_data->data[0];
    //std::cout << "firefly num = " << firefly_num << '\n' << std::endl;
    firefly_point.clear();
    for(int i = 0;i<firefly_num;i++)
    {
        //std::cout << "start input" << '\n' << std::endl;
        firefly_point.push_back(firefly_data->data[i+1]);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "filter");
    ros::NodeHandle fi;
    float revenue = 0.0;

    ros::Subscriber sub= fi.subscribe<nav_msgs::OccupancyGrid>("map", 10000 ,mapCallBack);
    ros::Subscriber sub_boat_position = fi.subscribe<nav_msgs::Odometry>("odom",10000,odominfoCallBack);
    ros::Subscriber sub_firefly = fi.subscribe<auto_exploration::array2>("firefly_points", 100, fireflycallback);

    ros::Publisher firefly_pub = fi.advertise<sensor_msgs::PointCloud2>("firefly_p", 100);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 frontier_cloud;

    cloud.width = 100000;
    cloud.height = 1;
    //cloud.points.resize(cloud.width * cloud.height);
    cloud.points.resize(100000);

    ros::Publisher pub = fi.advertise<visualization_msgs::Marker>("/best", 10);
    ros::Publisher targetspub = fi.advertise<geometry_msgs::PointStamped>("/best_points", 10);

    ros::Publisher pub_goal = fi.advertise<auto_exploration::best_goal>("/best_auto_exploration", 10);

        //visualizations  points and lines..
    points.header.frame_id="map";
    line.header.frame_id="map";
    points.header.stamp=ros::Time(0);
    line.header.stamp=ros::Time(0);
        
    points.ns=line.ns = "markers";
    points.id = 0;
    line.id =1;

    points.type = points.POINTS;
    line.type=line.LINE_LIST;

    //Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action =points.ADD;
    line.action = line.ADD;
    points.pose.orientation.w =1.0;
    line.pose.orientation.w = 1.0;
    line.scale.x =  0.03;
    line.scale.y= 0.03;
    points.scale.x=0.3; 
    points.scale.y=0.3; 

    line.color.r =9.0/255.0;
    line.color.g= 91.0/255.0;
    line.color.b =236.0/255.0;
    points.color.r = 255.0/255.0;
    points.color.g = 0.0/255.0;
    points.color.b = 0.0/255.0;
    points.color.a=1.0;
    line.color.a = 1.0;
    points.lifetime = ros::Duration();
    line.lifetime = ros::Duration();

    geometry_msgs::Point p;  

    points.points.clear();
    pub.publish(points) ;

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        auto_exploration::best_goal best_data;
        float temp = 0.0, I = 0.0, D = 0.0, A = 0.0, B = 0.0;
        int best = 0;
        float max;
        int delet_flag = 0;
        ros::spinOnce();
        //std::cout << "roll = " << roll << "pitch = " << pitch << "yaw = " << yaw << '\n' << std::endl;
        if(firefly_flag == 1)
        {

            // std::cout << "now firefly_num =  " << firefly_point.size() << '\n' << std::endl;
            // for(int check_i = 0; check_i < firefly_point.size();check_i++)
            // {
            //     if(firefly_point[check_i] == 0)
            //     {
            //         //std::cout << "delet start " << '\n' << std::endl;
            //         firefly_point.erase(firefly_point.begin() + check_i);
            //         check_i--;
            //     }
            // }

            std::cout << "firefil_num = " << firefly_point.size() << '\n' << std::endl;

            R.clear();
            max = 0.0;
            for(int i = 0; i<firefly_point.size(); i++)
            {
                 //I = ri*h(boat_idx, firefly_point[i])*information(firefly_point[i]);
                 I = ri*information(firefly_point[i]);
                 //I = ri*information(firefly_point[i]);
                D = rd*distance(boat_idx, firefly_point[i]);
                A = ra*angle(boat_idx, firefly_point[i]);
                B = rb*pow(barrier(firefly_point[i]),2);
                revenue = I - B - A - D;                      //注意机器人里程计是反的
                R.push_back(revenue);
            }

            max = R[0];
            best = firefly_point[0];
            for(int pi = 0;pi<firefly_point.size();pi++)
            {            
                if(R[pi] > max)                                    //注意
                {
                    max = R[pi];
                    best = firefly_point[pi];
                    //std::cout << "best_fire = " << firefly_point[pi]  << '\n' << std::endl;
                }
            }
            std::cout << "best_revenue = " << max <<  "  best = "  << best << '\n' << std::endl;

            if(best != 0)
            {
                    best_data.idx = best;
                    pub_goal.publish(best_data);
            }

            idx_xy(best);
            auto_exploration_goal.header.stamp=ros::Time(0);
            auto_exploration_goal.header.frame_id=mapData.header.frame_id;
            auto_exploration_goal.point.x=xy_pose[0];
            auto_exploration_goal.point.y=xy_pose[1];
            auto_exploration_goal.point.z=0.0;
            targetspub.publish(auto_exploration_goal);

            for(int j = 0;j<firefly_point.size();j++)
            {
                idx_xy(firefly_point[j]);
                cloud.points[j].x = xy_pose[0];
                cloud.points[j].y = xy_pose[1];
                cloud.points[j].z = 0.1;
                //std::cout << "frontier x = " << xy_pose[0] << "  frontier y = " << xy_pose[1] << '\n' << std::endl;
            }
            //std::cout << "num = " << firefly_point.size() << '\n' << std::endl;

            pcl::toROSMsg(cloud, frontier_cloud);
            frontier_cloud.header.frame_id = "map";
            firefly_pub.publish(frontier_cloud);

            for(int k=0;k<firefly_point.size();k++)                                                           //边界点坐标更新
            {
                cloud.points[k].x = 0;
                cloud.points[k].y = 0;
                cloud.points[k].z = 0;
                //cloud.channels[0].values[j] = 100*j;
            }
        }

        firefly_flag = 0;

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
    ix = idx/mapData.info.width;
    iy = idx%mapData.info.width;
    //     ix = idx/wi;
    // iy = idx%wi;
    xy_pose[1] = ix*box+o_y;
    xy_pose[0] = iy*box+o_x;
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

float distance(int x1, int x2)
{    
    float x_v[2];
    float start_v[2];
    idx_xy(x1);
    x_v[0] = xy_pose[0]; x_v[1] = xy_pose[1];
    idx_xy(x2);
    start_v[0] = xy_pose[0]; start_v[1] = xy_pose[1];
    return (pow((x_v[0]-start_v[0]),2)+pow((x_v[1]-start_v[1]),2));
}

int information(int check_idx)
{
    inf_value = 0;
    inf_bar = 0;
    inf_auto_exploration = 0;
    int n = 15;
    int check_point[8];
    std::vector<signed char> Data=mapData.data;
    float w = mapData.info.width;
    float h = mapData.info.height;

    for(int i_i = (-n); i_i<n; i_i++)
    {
        for(int i_j = (-n); i_j < n; i_j++)
        {
            if(0 < (check_idx + i_i*w + i_j) && (check_idx + i_i*w + i_j) < (w*h))
            {
                if(Data[check_idx + i_i*w + i_j] == (-1))
                {
                    inf_value++;
                }
            }
        }
    }
    return inf_value;
}

int barrier(int check_idx)
{
    bar_value = 0;
    int n = 10;
    std::vector<signed char> Data=mapData.data;
    float w = mapData.info.width;
    for(int i_i = (-n); i_i<n; i_i++)
    {
        for(int i_j = (-n); i_j < n; i_j++)
        {
            if(Data[check_idx + i_i*w + i_j] > 0)
            {
                bar_value++;
            }
        }
    }
    return bar_value;
}

float angle(int bo_idx, int check_idx)
{
    float b_x, b_y, c_x, c_y, cha_x, cha_y;
    float k = 0.0;
    float out = 0.0;
    idx_xy(boat_idx);
    b_x = xy_pose[0];
    b_y = xy_pose[1];
    idx_xy(check_idx);
    c_x = xy_pose[0];
    c_y = xy_pose[1];
    cha_x = c_x - b_x;
    cha_y = c_y - b_y;
    k = (cha_y)/(cha_x);
    if(cha_x > 0&&cha_y > 0)
    {
        out = pow((atan(k) - yaw), 2);
    }
    if(cha_x > 0&&cha_y < 0)
    {
        out = pow((atan(k) - yaw), 2);
    }
    if(cha_x < 0&&cha_y > 0) 
    {
        out = pow((atan(k) + 4 - yaw), 2);
    }
    if(cha_x < 0&&cha_y < 0)
    {
        out = pow((atan(k) - 4 - yaw), 2);
    }
     //out = pow(atan2(c_y - b_y, c_x - b_x ), 2);
    return (pow(out, 3));
}

// float h(int bo_idx, int check_idx)
// {
//     float h_rad = 500;
//     float h_out;
//     if(distance(bo_idx, check_idx) > h_rad)
//     {
//         h_out = 0.7;
//     }
//     else h_out = 1;
//     return h_out;
// }

int check_edge(int check_idx)
{
    int n = 15;      //9
    int out = 0;
    std::vector<signed char> Data=mapData.data;
    float w = mapData.info.width;
    float h = mapData.info.height;
    for(int i_i = (-n); i_i<n; i_i++)
    {
        for(int i_j = (-n); i_j < n; i_j++)
        {
            if(0 < (check_idx + i_i*w + i_j) && (check_idx + i_i*w + i_j) < (w*h))
            {
                if(Data[check_idx + i_i*w + i_j] > 0)
                {
                    out++;
                }
            }
        }
    }
    return out;
}
