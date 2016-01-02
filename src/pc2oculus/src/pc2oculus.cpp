#include <ros/ros.h>
#include <ios>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/filters/filter.h>
#include <boost/foreach.hpp>
#include <sstream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//  ros::NodeHandle nh;
sensor_msgs::PointCloud cloud;



class pubsub
{
public:
    pubsub();
    void callback(const PointCloud::ConstPtr& image);
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
};

pubsub::pubsub() {
    pub = nh.advertise<sensor_msgs::PointCloud>("/points", 1000);
    sub = nh.subscribe<PointCloud>("/camera/depth/points", 1000, &pubsub::callback, this);
}

void pubsub::callback(const PointCloud::ConstPtr& unfiltered)
{
      geometry_msgs::Point32 p;

      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*unfiltered,*filtered, indices);

      BOOST_FOREACH (const pcl::PointXYZ& pt, filtered->points){
    //        std::cout << pt.x << ", " << pt.y << ", " << pt.z << "\n";
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            cloud.points.push_back(p);
      }

     pub.publish(cloud);
     cloud.points.clear();

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pc2oculus");
  pubsub ps;
  ros::spin();
}
