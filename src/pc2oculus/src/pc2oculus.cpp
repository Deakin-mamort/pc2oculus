#include <ros/ros.h>
#include <ios>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_types.h>
#include <pcl_ros/filters/filter.h>
#include <boost/foreach.hpp>
#include <sstream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
int count;
void callback(const PointCloud::ConstPtr& unfiltered)
{

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Point> ("/points", 100);
  geometry_msgs::Point p;

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*unfiltered,*filtered, indices);

  BOOST_FOREACH (const pcl::PointXYZ& pt, filtered->points){
//        std::cout << pt.x << ", " << pt.y << ", " << pt.z << "\n";
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        pub.publish(p);

  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pc2oculus");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);
  ros::spin();
}
