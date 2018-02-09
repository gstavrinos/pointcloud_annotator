#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

bool more;
ros::Publisher pub;
std::string in_topic;
int width, length;
int red_value, green_value, blue_value, sx, sy;

void cloud_callback (const sensor_msgs::PointCloud2& msg){

    more = false;

    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(msg, pcl_cloud);

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromPCLPointCloud2(pcl_cloud, cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloud, cloud, indices);
    bool done = false;

    while(!done){
        for(unsigned i=0; i<cloud.width; i+sy){
            for(unsigned j=i; )
            /*std::cout << cloud.points[i].x << std::endl;
            std::cout << cloud.points[i].y << std::endl;
            std::cout << cloud.points[i].z << std::endl;
            std::cout << cloud.points[i].getRGBVector3i()[0] << std::endl;
            std::cout << cloud.points[i].getRGBVector3i()[1] << std::endl;
            std::cout << cloud.points[i].getRGBVector3i()[2] << std::endl;
            std::cout << cloud.points[i] << std::endl;*/
            cloud.points[i].r = red_value;
            cloud.points[i].g = green_value;
            cloud.points[i].b = blue_value;
        }
        sensor_msgs::PointCloud2 output;
        pcl::toPCLPointCloud2(cloud, pcl_cloud);
        pcl_conversions::fromPCL(pcl_cloud, output);
        pub.publish (output);
        if(true){
            done = true;
        }
    }
    more = true;
}


int main (int argc, char** argv){
    ros::init (argc, argv, "pointcloud_annotator");
    ros::NodeHandle nh;

    nh.param("pointcloud_annotator/input_topic", in_topic, std::string("zed/point_cloud/cloud_registered/limitcloud"));

    nh.param("pointcloud_annotator/width", width, 100);
    nh.param("pointcloud_annotator/length", length, 120);

    nh.param("pointcloud_annotator/slide_x", sx, 10);
    nh.param("pointcloud_annotator/slide_y", sy, 10);

    nh.param("pointcloud_annotator/red_value", red_value, 255);
    nh.param("pointcloud_annotator/green_value", green_value, 255);
    nh.param("pointcloud_annotator/blue_value", blue_value, 0);

    ros::Subscriber sub = nh.subscribe (in_topic, 1, cloud_callback);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud_annotator/points", 1);

    more = true;
    while(ros::ok()){
        if(more){
            ros::spinOnce();
        }
    }
}