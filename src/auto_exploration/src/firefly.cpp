#include "ros/ros.h"
#include <iostream>
#include <time.h>
#include <vector>
#include "auto_exploration/array2.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include<nav_msgs/Odometry.h>

void idx_xy(int idx);
float distance(int x1, int x2);
int xy_idx(float x, float y);
int check_edge(int check_idx);

float I0[2];
float limit_wfd = 0.5;
float limit_rrt = 0.5;

int trans_flag = 0;

int finishwfd = 0;
int finishrrt = 0;

float boat_x = 0.0, boat_y = 0.0;
int boat_idx = 0;

int wfd_num = 0, rrt_num = 0;
std::vector<int> wfd_point;
std::vector<int> rrt_point;
std::vector<int> sum_points;
float xy_pose[2];

int wfdcall_flag = 0;
int mapcall_flag = 0;
int odomcall_flag = 0;
int rrtcall_flag = 0;

nav_msgs::OccupancyGrid mapData;

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    mapcall_flag = 1;
    mapData = *msg;
}

void wfdcallback(const auto_exploration::array2::ConstPtr& wfd_data)
{
    wfdcall_flag = 1;
    wfd_num = wfd_data->data[0];
    //std::cout << "wfd num = " << wfd_num << '\n' << std::endl;
    wfd_point.clear();
    for(int i = 0;i<wfd_num;i++)
    {
        //std::cout << "start input" << '\n' << std::endl;
        wfd_point.push_back(wfd_data->data[i+1]);
    }

}

void rrtcallback(const auto_exploration::array2::ConstPtr& rrt_data)
{
    rrtcall_flag = 1;
    rrt_num = rrt_data->data[0];
    //std::cout << "wfd num = " << wfd_num << '\n' << std::endl;
    rrt_point.clear();
    for(int i = 0;i<rrt_num;i++)
    {
        //std::cout << "start input" << '\n' << std::endl;
        rrt_point.push_back(rrt_data->data[i+1]);
    }
}

void odominfoCallBack(const nav_msgs::Odometry::ConstPtr& odom)
{
    odomcall_flag = 1;
    float ix_o = 0.0,iy_o = 0.0;
    //std::cout << "build start position\n" << std::endl;
    boat_x = odom->pose.pose.position.x;
    boat_y = odom->pose.pose.position.y;

    boat_idx = xy_idx(boat_x, boat_y);
    //std::cout << "boat_idx  = " << boat_idx << std::endl;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "firefly");
    ros::NodeHandle fn;

    ros::Subscriber sub= fn.subscribe<nav_msgs::OccupancyGrid>("map", 10000 ,mapCallBack);

    ros::Subscriber sub_boat_position = fn.subscribe<nav_msgs::Odometry>("odom",10000,odominfoCallBack);

    ros::Subscriber sub_wfd = fn.subscribe<auto_exploration::array2>("firefly_wfd", 100, wfdcallback);

    ros::Subscriber sub_rrt = fn.subscribe<auto_exploration::array2>("firefly_rrt", 100, rrtcallback);

    ros::Publisher WFD_pub = fn.advertise<sensor_msgs::PointCloud2>("firefly_wfds", 100);
    pcl::PointCloud<pcl::PointXYZ> cloud_wfd;
    sensor_msgs::PointCloud2 frontier_cloud_wfd;

    cloud_wfd.width = 100000;
    cloud_wfd.height = 1;
    //cloud.points.resize(cloud.width * cloud.height);
    cloud_wfd.points.resize(100000);

    ros::Publisher rrt_pub = fn.advertise<sensor_msgs::PointCloud2>("firefly_rrts", 100);
    pcl::PointCloud<pcl::PointXYZ> cloud_rrt;
    sensor_msgs::PointCloud2 frontier_cloud_rrt;

    cloud_rrt.width = 100000;
    cloud_rrt.height = 1;
    //cloud.points.resize(cloud.width * cloud.height);
    cloud_rrt.points.resize(100000);

    ros::Publisher pub_f = fn.advertise<auto_exploration::array2>("firefly_points", 10);

    ros::Time current_time,last_time;
    current_time = ros::Time::now();

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        auto_exploration::array2 firefly_data;
        ros::spinOnce();  

                for(int check_i = 0; check_i < wfd_point.size();check_i++)
                {
                    if(check_edge(wfd_point[check_i]) >= 1)
                    {
                        //std::cout << "delet start " << '\n' << std::endl;
                        wfd_point.erase(wfd_point.begin() + check_i);
                        check_i--;
                    }
                }
                for(int check_i = 0; check_i < rrt_point.size();check_i++)
                {
                    if(check_edge(rrt_point[check_i]) >= 1)
                    {
                        //std::cout << "delet start " << '\n' << std::endl;
                        rrt_point.erase(rrt_point.begin() + check_i);
                        check_i--;
                    }
                }

                if(wfd_point.size() < 40)
                {
                    trans_flag = 1;
                }
                else trans_flag = 0;

            if(wfdcall_flag == 1)
            //if(wfdcall_flag == 1)
            {
                //std::cout << "wfd_point  =  " << wfd_point.size() << '\n' << std::endl;
                for(int wi_f = 0;wi_f<wfd_point.size();wi_f++)
                {
                    for(int wj_f = 0;wj_f<wfd_point.size();wj_f++)
                    {
                        if(wi_f != wj_f)
                        {
                            if(distance(wfd_point[wi_f], wfd_point[wj_f]) < limit_wfd)
                            {
                                I0[0] = exp(-distance(boat_idx, wfd_point[wi_f]));
                                I0[1] = exp(-distance(boat_idx, wfd_point[wj_f]));
                                if(I0[1] < I0[0])
                                {
                                    //std::cout << "delete " << '\n' << std::endl;
                                    wfd_point.erase(wfd_point.begin()+wj_f);
                                }
                            }
                        }
                    }
                }

                //std::cout << "wfd_point_delete  =  " << wfd_point.size() << '\n' << std::endl;
                for(int wj = 0;wj<wfd_point.size();wj++)
                {
                    idx_xy(wfd_point[wj]);
                    cloud_wfd.points[wj].x = xy_pose[0];
                    cloud_wfd.points[wj].y = xy_pose[1];
                    cloud_wfd.points[wj].z = 0.1;
                    //std::cout << "frontier x = " << xy_pose[0] << "  frontier y = " << xy_pose[1] << '\n' << std::endl;
                }

                pcl::toROSMsg(cloud_wfd, frontier_cloud_wfd);
                frontier_cloud_wfd.header.frame_id = "map";
                WFD_pub.publish(frontier_cloud_wfd);

                for(int wk=0;wk<wfd_point.size();wk++)                                                           //边界点坐标更新
                {
                    cloud_wfd.points[wk].x = 0;
                    cloud_wfd.points[wk].y = 0;
                    cloud_wfd.points[wk].z = 0;
                    //cloud.channels[0].values[j] = 100*j;
                }
            }

            if(rrtcall_flag == 1)
            //if(rrtcall_flag == 1)
                {
                    //std::cout << "start wfd explorstion" << '\n' << std::endl;
                    //std::cout << "rrt_point  =  " << rrt_point.size() << '\n' << std::endl;
                    for(int ri_f = 0;ri_f<rrt_point.size();ri_f++)
                    {
                        for(int rj_f = 0;rj_f<rrt_point.size();rj_f++)
                        {
                            if(ri_f != rj_f)
                            {
                                if(distance(rrt_point[ri_f], rrt_point[rj_f]) < limit_rrt)
                                {
                                    I0[0] = exp(-distance(boat_idx, rrt_point[ri_f]));
                                    I0[1] = exp(-distance(boat_idx, rrt_point[rj_f]));
                                    if(I0[1] < I0[0])
                                    {
                                        //std::cout << "delete " << '\n' << std::endl;
                                        rrt_point.erase(rrt_point.begin()+rj_f);
                                    }
                                }
                            }
                        }
                    }

                    //std::cout << "rrt_point_delete  =  " << rrt_point.size() << '\n' << std::endl;
                    for(int rj = 0;rj<rrt_point.size();rj++)
                    {
                        idx_xy(rrt_point[rj]);
                        cloud_rrt.points[rj].x = xy_pose[0];
                        cloud_rrt.points[rj].y = xy_pose[1];
                        cloud_rrt.points[rj].z = 0.1;
                        //std::cout << "frontier x = " << xy_pose[0] << "  frontier y = " << xy_pose[1] << '\n' << std::endl;
                    }

                    pcl::toROSMsg(cloud_rrt, frontier_cloud_rrt);
                    frontier_cloud_rrt.header.frame_id = "map";
                    rrt_pub.publish(frontier_cloud_rrt);

                    for(int rk=0;rk<rrt_point.size();rk++)                                                           //边界点坐标更新
                    {
                        cloud_rrt.points[rk].x = 0;
                        cloud_rrt.points[rk].y = 0;
                        cloud_rrt.points[rk].z = 0;
                        //cloud.channels[0].values[j] = 100*j;
                    }
                }


                if(rrtcall_flag == 1&&trans_flag == 1)
                {
                    std::cout << "start rrt explorstion" << '\n' << std::endl;
                    for(int sum_i = 0; sum_i<rrt_point.size();sum_i++)
                    {
                        sum_points.push_back(rrt_point[sum_i]);
                    }
                    firefly_data.data[0] = sum_points.size();
                    //std::cout << "sum_num = " << firefly_rrt.data[0] << '\n' << std::endl;
                    for(int sir = 0;sir<sum_points.size();sir++)
                    {
                        firefly_data.data[sir+1] = sum_points[sir];
                    }
                    pub_f.publish(firefly_data);

                    sum_points.clear();
                }

                if(wfdcall_flag == 1&&trans_flag == 0)
                {
                    //std::cout << "start wfd explorstion" << '\n' << std::endl;
                    for(int sum_i = 0; sum_i<wfd_point.size();sum_i++)
                    {
                        sum_points.push_back(wfd_point[sum_i]);
                    }
                    firefly_data.data[0] = sum_points.size();
                    //std::cout << "sum_num = " << firefly_rrt.data[0] << '\n' << std::endl;
                    for(int sir = 0;sir<sum_points.size();sir++)
                    {
                        firefly_data.data[sir+1] = sum_points[sir];
                    }
                    pub_f.publish(firefly_data);

                    sum_points.clear();
                }         

            rrtcall_flag = 0;
            wfdcall_flag = 0;

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
    // ix = idx/wi;
    // iy = idx%wi;
    ix = idx/mapData.info.width;
    iy = idx%mapData.info.width;
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

int check_edge(int check_idx)
{
    int n = 9;      //9
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