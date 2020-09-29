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
#include <map>
// #include <shared_ptr>
//计时
clock_t  clock_start, clock_end;
std::vector<Eigen::Isometry3d> all_pose;  //allpose会不会爆掉
std::vector<double> all_pose_stamp;
std::vector<std::vector<pcl::PointXYZ>> all_points;
std::vector<double> all_points_stamp;
std::string _out_folder= "/home/jz/map_save/";
double odom_x=0;
double odom_y=0;

struct keyForMap
{
    int x;
    int y;
};

template<class keyForMap>
struct PLess	
{
	// functor for operator<
	bool operator()(const keyForMap& pLeft, const keyForMap& pRight) const
	{
		double left = pLeft.x*250251 + pLeft.y;
		double right = pRight.x*250251 + pRight.y;
		return left < right;		// 这个比较器与默认比较器不同的地方
	}
};

void odomeCallback(const nav_msgs::Odometry laserOdometry)
{
    Eigen::Quaterniond q( laserOdometry.pose.pose.orientation.w, 
			  laserOdometry.pose.pose.orientation.x, 
			  laserOdometry.pose.pose.orientation.y, 
			  laserOdometry.pose.pose.orientation.z );
    Eigen::Isometry3d T(q);  //3.3版本eigen废除iosmetry 
    // Eigen::Transform <double,3,Eigen::Isometry> R;  
    // R.rotate(q); 
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
    int grid_size = 30;

    std::vector< std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> > ptr_vector;

    std::cout<< "allpose = "<< all_pose.size()<<endl;
    Eigen::Isometry3d T ;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud( new pcl::PointCloud<pcl::PointXYZI> ); 
    std::map<keyForMap, pcl::PointCloud<pcl::PointXYZI>::Ptr, PLess<keyForMap> > pcMap;

    ROS_INFO("pcd_all_begin");
    while(state_pcd)
    {
        usleep(100);
    
    // 1. sync
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
        clock_start=clock();
        }


        T = all_pose[pcd_num];
        ROS_INFO("pcd_num=%d",pcd_num);

        // 2.  判断odom在的格子->周围有没有划分好格子 ->建格子
        double odom_x = T.translation().x();    //读取odom里的x位置
        double odom_y = T.translation().y();    //读取odom里的y位置
        int odom_center_x = static_cast<int>( round (odom_x / grid_size  ) )* grid_size ;  //odom里的x位置,四舍五入确定该odom所属于的中心x位置
        int odom_center_y = static_cast<int>( round (odom_y / grid_size  ) )* grid_size ;  //odom里的y位置,四舍五入确定该odom所属于的中心y位置
        for(int setp_x = -1 ; setp_x <= 1; setp_x++)   //遍历以自己为中心位置的从-1到 1 的3x3的格子
        {
            for(int setp_y = -1 ; setp_y <= 1; setp_y++)
            {
                int round_x = odom_center_x + grid_size * setp_x;  //round_x指的是 x包含自己内的周围9个格子的x,y坐标，为索引ID做准备
                int round_y = odom_center_y + grid_size * setp_y;

                keyForMap findKey{ round_x, round_y };  //将round_x,round_y俩数组形式存进 findKey
                auto it = pcMap.find(findKey);  // 通过迭代器查找findKey

                if(it == pcMap.end())   //如果没有查到findKey
                {
                    //没找到，建立一个
                    pcl::PointCloud<pcl::PointXYZI>::Ptr new_grid( new pcl::PointCloud<pcl::PointXYZI> ); 
                    // ptr_vector.push_back( new_grid );
                    pcMap.insert( std::pair<keyForMap, pcl::PointCloud<pcl::PointXYZI>::Ptr>(findKey, new_grid) ); //插入std::map中两个容器的比较值特就是findKey和new_grid
                }
            }
        }

        int bound_size = int(grid_size * 1.5);        //中心点四周边界的大小
        int bound_x_up = odom_center_x + bound_size;  //x上边界
        int bound_x_down = odom_center_x - bound_size;
        int bound_y_up = odom_center_y + bound_size;
        int bound_y_down = odom_center_y - bound_size;

        // 3. point transform to the world frame
        for ( int cloud_temp_num = 0; cloud_temp_num<int(all_points[pcd_num*2].size()); cloud_temp_num++ ) //遍历一帧内的数据
        {
            Eigen::Vector3d point;   //该帧内点云的x,y,z  后面进行坐标系转换成世界坐标系
            point[0] = all_points[pcd_num*2][cloud_temp_num].x;
            point[1] = all_points[pcd_num*2][cloud_temp_num].y; 
            point[2] = all_points[pcd_num*2][cloud_temp_num].z;
            Eigen::Vector3d pointWorld = T *point;

            pcl::PointXYZI p ;
            p.x = pointWorld[0];
            p.y = pointWorld[1];
            p.z = pointWorld[2];

            if( p.x >= bound_x_down && p.x <= bound_x_up )      //世界坐标系的在上边界内在下边界内的数据点进行操作
            {
                if( p.y >= bound_y_down && p.y <= bound_y_up )
                {
                    int p_center_x = static_cast<int>( round ( p.x / grid_size  ) )* grid_size ;
                    int p_center_y = static_cast<int>( round ( p.y / grid_size  ) )* grid_size ;

                    keyForMap key{ p_center_x, p_center_y };  //将p点的中心位置x，y存进key二维数组中
                    pcMap[key]->push_back( p );   //将点p以key的形式压到pcMap中的keyForMap   问下xx
                }
            }
        }

        // down sample
        for(int setp_x = -1 ; setp_x <= 1; setp_x++)
        {
            for(int setp_y = -1 ; setp_y <= 1; setp_y++)   //初始化 周围点的矩阵 放置pcMap的关键词中 将之前存在mpMap中的第二个也就是指针指向的点云数据进行降采样 随后输出
            {
                int round_x = odom_center_x + grid_size * setp_x;
                int round_y = odom_center_y + grid_size * setp_y;

                keyForMap findKey{ round_x, round_y };  //不懂有些问题
                auto it = pcMap.find(findKey);   //没懂

                if((it->second)->size() == 0)
                    continue;

                pcl::VoxelGrid<pcl::PointXYZI> sor;
                sor.setInputCloud ((it->second));//为什么是第二个
                sor.setLeafSize (0.2, 0.2, 0.2);
                pcl::PointCloud<pcl::PointXYZI>::Ptr grid_pc( new pcl::PointCloud<pcl::PointXYZI> ); 
                sor.filter (*grid_pc);
                pcMap[findKey] = grid_pc;  //将关键词与之对应的指针指向降采样数据
            }
        }

        clock_end=clock();
        if(state_pcd==1&&double(clock_end-clock_start)/ CLOCKS_PER_SEC>2)
        {
            int count = 0;
            // save to pcd
            for ( auto it = pcMap.begin(); it != pcMap.end(); it++ )
            {
                if((it->second)->size() == 0)
                    continue;

                // pcl::VoxelGrid<pcl::PointXYZI> sor;
                // sor.setInputCloud ((it->second));
                // sor.setLeafSize (0.2, 0.2, 0.2);
                // pcl::PointCloud<pcl::PointXYZI>::Ptr grid_pc( new pcl::PointCloud<pcl::PointXYZI> ); 
                // sor.filter (*grid_pc);

                keyForMap key = it->first;
                std::string filename = _out_folder + std::to_string(int(grid_size)) + "_" +
                                                    std::to_string(key.x) + "_" +
                                                    std::to_string(key.y) + ".pcd";
                pcl::io::savePCDFileBinary(filename, *(it->second) );
                count++;

                std::cout << key.x << "  " << key.y << std::endl;
                std::cout << (it->second)->size() << std::endl;

            }
            std::cout << count << std::endl;
            std::cout << " End " << std::endl;
            break;
        }
     }      
    } 
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



