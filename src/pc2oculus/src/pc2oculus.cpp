//Standard C++
#include <ios>
#include <sstream>

//ROS
#include <ros/ros.h>

//ROS msg formats
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>


//BOOST
#include <boost/foreach.hpp>

//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//  ros::NodeHandle nh;
sensor_msgs::PointCloud cloud;
    int marker_count=0;

class pubsub
{
public:
    pubsub();
    void subscriber_callback(const PointCloud::ConstPtr&);
    void subscriber_callback1(const visualization_msgs::MarkerArray::ConstPtr&);
private:
    ros::NodeHandle nh;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
};

pubsub::pubsub() {
    publisher = nh.advertise<sensor_msgs::PointCloud>("/points", 1);
    //publisher = nh.advertise<sensor_msgs::PointCloud2>("/points", 1);
    //subscriber = nh.subscribe<PointCloud>("/octomap_point_cloud_centers", 1, &pubsub::subscriber_callback, this);
    subscriber = nh.subscribe<visualization_msgs::MarkerArray> ("/occupied_cells_vis_array", 1, &pubsub::subscriber_callback1, this);
    //subscriber = nh.subscribe<PointCloud>("/camera/depth/points", 1, &pubsub::subscriber_callback, this);
}

void pubsub::subscriber_callback(const PointCloud::ConstPtr& raw)
{

    //Print original pointcloud information
    std::cout << "Original - \tHeight:" << raw->height
              << "\tWidth:" << raw->width
              << "\tFields: (" << pcl::getFieldsList(*raw) << ")."
              << std::endl;

    PointCloud::Ptr clean(new PointCloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*raw, *clean, indices);

    /*Print original pointcloud information
    std::cout << "Cleaned - \tHeight:" << clean->height
              << "\tWidth:" << clean->width
              << "\tFields: (" << pcl::getFieldsList(*clean) << ")."
              << std::endl;
    */


    const PointCloud::Ptr filtered(new PointCloud);

    /*
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (clean);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*filtered);

    const PointCloud::Ptr voxelized(new PointCloud);

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(clean);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel.filter(*voxelized);

//    sensor_msgs::PointCloud2 output;
  //  pcl_conversions::fromPCL(*filtered, output);
    //publisher.publish(voxelized);
    //std::cout << filtered->points;

/*
    std::cout << "Filtered - \tHeight:" << voxelized->height
              << "\tWidth:" << voxelized->width
              //<< "\tFields: (" << pcl::getFieldsList(*filtered) << ")."
              << std::endl;
*/
    geometry_msgs::Point32 p;
    BOOST_FOREACH (const pcl::PointXYZ& pt, clean->points){
          //std::cout << pt.x << ", " << pt.y << ", " << pt.z << "\n";
          p.x = pt.x;
          p.y = pt.y;
          p.z = pt.z;
          cloud.points.push_back(p);
    }

    publisher.publish(cloud);
    cloud.points.clear();

}

void pubsub::subscriber_callback1(const visualization_msgs::MarkerArray::ConstPtr& raw)
{
    sensor_msgs::PointCloud2 output;
    BOOST_FOREACH (const visualization_msgs::Marker& marker, raw->markers){
        if(marker.points.size() > (1.1*marker_count))
        {
            geometry_msgs::Point32 p;
            BOOST_FOREACH (const geometry_msgs::Point& pt, marker.points){
                  //std::cout << pt.x << ", " << pt.y << ", " << pt.z << "\n";
                  p.x = pt.x;
                  p.y = pt.y;
                  p.z = pt.z;
                  cloud.points.push_back(p);

            }

            std::cout << "Published\n";
            publisher.publish(cloud);
            marker_count=marker.points.size();

            /*
            BOOST_FOREACH (const std_msgs::ColorRGBA colour, marker.colors ){;}
            std::cout << "colours: " << marker.colors.size() << "\n";
            BOOST_FOREACH (const geometry_msgs::Point point, marker.points ){;}
            std::cout << "points: " << marker.points.size() << "\n";
            marker_count=marker.points.size();
            publisher.publish(output);
            //  frame++;
            */

        }
    }

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pc2oculus");
  pubsub ps;
  ros::spin();
}
