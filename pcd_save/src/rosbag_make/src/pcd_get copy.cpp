#include <Eigen/Geometry> 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <boost/thread.hpp>
#include <time.h>
#include "pcd_grid_divider.h"

#include <unistd.h> 
#include <pcl/filters/voxel_grid.h>
#include <sstream>
#include <string>
#include <vector>


//计时
clock_t  clock_start, clock_end;
std::vector<Eigen::Isometry3d> all_pose;
std::vector<double> all_pose_stamp;
std::vector<std::vector<pcl::PointXYZ>> all_points;
std::vector<double> all_points_stamp;
std::string _out_folder= "/home/jz/map_save/";
double odom_x=0;
double odom_y=0;


void odomeCallback(const nav_msgs::Odometry laserOdometry)
{
    Eigen::Quaterniond q( laserOdometry.pose.pose.orientation.w, 
			  laserOdometry.pose.pose.orientation.x, 
			  laserOdometry.pose.pose.orientation.y, 
			  laserOdometry.pose.pose.orientation.z );
    Eigen::Isometry3d T(q);
    T.pretranslate( Eigen::Vector3d( laserOdometry.pose.pose.position.x, 
				     laserOdometry.pose.pose.position.y, 
				     laserOdometry.pose.pose.position.z ));
    odom_x=laserOdometry.pose.pose.position.x;
    odom_y=laserOdometry.pose.pose.position.y; 
    all_pose.push_back( T );
    all_pose_stamp.push_back(double(laserOdometry.header.stamp.toSec()));


}
 
void cloudCallback(const sensor_msgs::PointCloud2 ros_cloud)
{
    //ROS_INFO("cloud is going");
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_temp;
    pcl::fromROSMsg(ros_cloud, pcl_cloud_temp);
    std::vector<pcl::PointXYZ> cloud_vec;
 
    for(int i=0;i<int(pcl_cloud_temp.points.size());i++)
    {
	pcl::PointXYZ temp_one_point;
	temp_one_point.x=pcl_cloud_temp.points[i].x;
	temp_one_point.y=pcl_cloud_temp.points[i].y;
	temp_one_point.z=pcl_cloud_temp.points[i].z;
	cloud_vec.push_back(temp_one_point);
    }
    all_points.push_back(cloud_vec);
    all_points_stamp.push_back(double(ros_cloud.header.stamp.toSec()));//把时间戳数据压进去
    cloud_vec.clear();
}
 
void pcd_maker()
{
    int pcd_num= 0;
    int state_pcd= 2;
    double min_x = 0;
    double max_x = 0;
    double min_y = 0;
    double max_y = 0;

    int grid_size = 50;
    int points_num = 0;     
    int pcd_print= 0;
    int pcd_save_id= 0;
    int odom_move_fig_x = 0;
    int odom_move_fig_y = 0; 
    int odom_grid_x = 0; //odom_trans_x / grid_size;
    int odom_grid_y = 0; //odom_trans_y / grid_size;
    int cloud_temp_num_save = 0;
    int cloud_temp_num_flag = 0;
    std::cout<< "allpose = "<< all_pose.size()<<endl;
    Eigen::Isometry3d T ;// 位置放在这个循环外会有BUG 内存溢出
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud( new pcl::PointCloud<pcl::PointXYZI> ); 
    // sleep(5);
    // time_test = 0;
    // tmp_save->clear();
    ROS_INFO("pcd_all_begin");
    while(state_pcd)
    {
        usleep(100);
    if(all_pose.size()!= 0 )
     {  
        if((int(all_points_stamp.size())>(10+pcd_num)*2)&&(int(all_pose_stamp.size())>(10+pcd_num)))
        {    
            //  sync
            if(state_pcd==2)
            {
                state_pcd=1;

                double points_time_first = all_points_stamp[0];
                double pose_time_first = all_pose_stamp[0];

                if( pose_time_first > points_time_first )
                {
                    while( pose_time_first != points_time_first )
                    {
                        auto itr = all_points_stamp.begin();
                        all_points_stamp.erase(itr);
                        points_time_first = all_points_stamp[0];
                    }
                }
                else if( pose_time_first < points_time_first )
                {
                    while( pose_time_first != points_time_first )
                    {
                        auto itr = all_pose_stamp.begin();
                        all_pose_stamp.erase(itr);
                        pose_time_first = all_pose_stamp[0];
                    }
                }
            }
        pcd_num++;
        pcd_print++;   
        }
        else
            continue;

        T = all_pose[pcd_num];
        for ( int cloud_temp_num = 0; cloud_temp_num<int(all_points[pcd_num*2].size()); cloud_temp_num++ ) //可能有BUG 关于点数
            {
                // if(cloud_temp_num_flag > 5 )
                // {
                // cloud_temp_num_flag = 0 ;
                // std::cout<<"cloud_temp_num = "<<cloud_temp_num<<endl;
                // }
                Eigen::Vector3d point;
                point[0] = all_points[pcd_num*2][cloud_temp_num].x;
                point[1] = all_points[pcd_num*2][cloud_temp_num].y; 
                point[2] = all_points[pcd_num*2][cloud_temp_num].z;
                Eigen::Vector3d pointWorld = T *point;

                // cloud_temp_num_save = cloud_temp_num;

                pcl::PointXYZI p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];

                pointCloud->points.push_back( p );
                // tmp_save->points.push_back( p );

                if (point[0]< min_x) {
                min_x = point[0];
                }
                if (point[0]> max_x) {
                max_x = point[0];
                }
                if (point[1]< min_y) {
                min_y = point[1];
                }
                if (point[1]> max_y) {
                max_y = point[1];
                } 
            }
        // cloud_temp_num_flag++;
        ROS_INFO("pcd_num=%d",pcd_num);

        if(pcd_print>20)
        {
        pcd_print=0;
        ROS_INFO("pcd_num=%d",pcd_num);
        std::cout<<"odom_x = "<<odom_x<<"   odom_y = "<<odom_y<<endl;
        odom_grid_x = static_cast<int>(floor( odom_x / grid_size )- floor( min_x / grid_size ) + 1);  
        odom_grid_y = static_cast<int>(floor( odom_y / grid_size )- floor( min_y / grid_size ) + 1);
        std::cout<< "grid_size = "<< grid_size <<endl;
        std::cout<< "min_x =  "<< min_x << "   max_x =  "<< max_x << "  min_y =  "<< min_y<< "   max_y =  "<< max_y << endl;
        std::cout<< "odom_x - min_x =  "<< odom_x - min_x << "  odom_y - min_y =  "<< odom_y - min_y<<endl;
        std::cout<<"odom_grid_x,odom_grid_y = ( "<<odom_grid_x<<" , "<<odom_grid_y<< " )" <<endl;
        std::cout<< "odom_grid_x =  "<< odom_grid_x<< "  odom_grid_y =  "<< odom_grid_y<<endl;
        std::cout<< "oodom_move_fig_x    = "<< odom_move_fig_x<<  "   oodom_move_fig_y    = "<< odom_move_fig_y<<endl;  
         if( odom_grid_x != odom_move_fig_x || odom_grid_y != odom_move_fig_y )
            {
            // state_pcd=0;
            std::cout<<"state_pcd=0;"<<endl;
            std::cout<<"odom_grid_move "<<endl;
            std::cout<<"start_save"<<endl;
            int min_x_b = grid_size * static_cast<int>(floor(min_x / grid_size));
            int max_x_b = grid_size * static_cast<int>(floor(max_x / grid_size) + 1);
            int min_y_b = grid_size * static_cast<int>(floor(min_y / grid_size));
            int max_y_b = grid_size * static_cast<int>(floor(max_y / grid_size) + 1);
            int div_x = (max_x_b - min_x_b) / grid_size;
            int div_y = (max_y_b - min_y_b) / grid_size;
            int grid_num = div_x * div_y;

            odom_move_fig_x = odom_grid_x;
            odom_move_fig_y = odom_grid_y;
            std::cout<< "odom_grid!= odom_move_fig   (odom_grid_pose_x,odom_grid_pose_y) =  ("<< odom_grid_x<< " , "<< odom_grid_y<<" )" <<endl;
            std::cout<< "!!!!!!  look grid_num =  "<< grid_num<< "  div_x= "<<div_x <<"  div_y = "<< div_y << endl;
            pcd_save_id = (odom_grid_y - 1)* div_x+ odom_grid_x;
            // Define filename, lower/upper bound of every grid                       要看看应该是每个格子的命名方式
            std::vector< MAP_TOOLS::pcd_grid > grids(grid_num);                                   //一共有多少块
            for (int y = 0; y < div_y; y++) {
                for (int x = 0; x < div_x; x++) {
                int id = div_x * y + x;
                grids[id].grid_id = id;
                grids[id].grid_id_x = x;
                grids[id].grid_id_y = y;
                grids[id].lower_bound_x = min_x_b + grid_size * x;
                grids[id].lower_bound_y = min_y_b + grid_size * y;
                grids[id].filename = _out_folder + std::to_string(int(grid_size)) + "_" +
                                    std::to_string(grids[id].lower_bound_x) + "_" +
                                    std::to_string(grids[id].lower_bound_y) + ".pcd";
                grids[id].name = std::to_string(int(grid_size)) + "_" +
                                std::to_string(grids[id].lower_bound_x) + "_" +
                                std::to_string(grids[id].lower_bound_y) + ".pcd";
                }
            }
            std::cout<<"grid_num = "<<grid_num <<endl;
            for (MAP_TOOLS::PointCloud::const_iterator p = pointCloud->points.begin();                         //存方块内的点   // Search minimum and maximum points along x and y axis.                     查询地图的x,y轴最大最小长度
                p != pointCloud->points.end(); p++) {
                int idx = static_cast<int>(
                    floor((p->x - static_cast<float>(min_x_b)) / grid_size));
                int idy = static_cast<int>(
                    floor((p->y - static_cast<float>(min_y_b)) / grid_size));
                int id = idy * div_x + idx;
                MAP_TOOLS::Point tmp = *p;
                grids[id].cloud.push_back( tmp );///我觉得存点的时候从头走到尾有点浪费 看看问问wxx
            }    
            if(pcd_save_id - div_x - odom_grid_x > 0  ) {       
                std::cout<< "if_1 pcd_save_id - div_x - odom_grid_x= "<<pcd_save_id - div_x - odom_grid_x<<endl;
                for (int i = 0; i < pcd_save_id - div_x - odom_grid_x; i++) {
                    if (grids[i].cloud.points.size() > 0 ) {

                        pcl::io::savePCDFileBinary(grids[i].filename, grids[i].cloud);
                        std::cout << "Wrote " << grids[i].cloud.points.size() << " points to "
                                    << grids[i].filename << "." << std::endl;
                        points_num += grids[i].cloud.points.size();
                        }
                    }
                    // state_pcd = 0;
                }   
            if(grid_num - (odom_grid_y + 1)* div_x > 0 ) {

                std::cout<< "pcd_save_id = "<< pcd_save_id <<endl;
                std::cout<< "if_2 grid_num - (odom_grid_y + 1)* div_x = "<< grid_num -(odom_grid_y + 1)* div_x <<endl;
                std::cout<< "div_x * odom_grid_y = "<< div_x * odom_grid_y << endl;
                std::cout<< "div_x = "<< div_x<<" div_y = "<< div_y<<"     odom_grid_y = " << odom_grid_y << endl;
                for (int i =  div_x * ( odom_grid_y + 1)  ; i <  div_x * div_y  ; i++) {
                    if (grids[i].cloud.points.size() > 0 ) {

                        pcl::io::savePCDFileBinary(grids[i].filename, grids[i].cloud);
                        std::cout << "Wrote " << grids[i].cloud.points.size() << " points to "
                                    << grids[i].filename << "." << std::endl;
                        points_num += grids[i].cloud.points.size();
                        }
                    }
                    // state_pcd = 0;
                }
            std::cout<< "min_x =  "<< min_x << "max_x =  "<< max_x << "  min_y =  "<< min_y<< "max_y =  "<< max_y << endl;    
            cout<<"state_pcd_over"<<endl;
            }
        }
     }      
    } 
    // clock_end=clock(); 
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "pcd_get");
    ros::NodeHandle n;
    ROS_INFO("begin");
    ros::Subscriber odome_sub = n.subscribe("/lio_sam/mapping/odometry", 50, odomeCallback, ros::TransportHints().tcpNoDelay());///lio_sam/mapping/odometry
    ros::Subscriber cloud_sub = n.subscribe("/lio_sam/deskew/cloud_deskewed", 50, cloudCallback, ros::TransportHints().tcpNoDelay());//可行/lio_sam/mapping/map_local  /lio_sam/deskew/cloud_deskewed
    ///velodyne_cloud_registered
    boost::thread server(pcd_maker);
    ros::spin();
    return 0;
}



