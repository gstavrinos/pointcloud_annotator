#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED 

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <boost/filesystem.hpp>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "pointcloud_annotator/Update.h"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace fs = ::boost::filesystem;

std::string path;

void getAll(const fs::path& root, const std::string& ext, std::vector<std::string>& ret){
    if(fs::exists(root) and fs::is_directory(root)){
        fs::recursive_directory_iterator it(root);
        fs::recursive_directory_iterator endit;
        while(it != endit){
            if(fs::is_regular_file(*it) && it->path().extension() == ext){
                ret.push_back(it->path().filename().string());
            }
            it++;
        }
    }
}

void getOnly(const fs::path& root, const std::string& ext, const std::string& cont, std::vector<std::string>& ret){
    if(fs::exists(root) and fs::is_directory(root)){
        fs::recursive_directory_iterator it(root);
        fs::recursive_directory_iterator endit;
        while(it != endit){
            if(fs::is_regular_file(*it) and it->path().extension() == ext and it->path().filename().string().find(cont) != std::string::npos){
                ret.push_back(it->path().filename().string());
            }
            it++;
        }
    }
}

bool serviceCallback (pointcloud_annotator::Update::Request &req, pointcloud_annotator::Update::Response &res){
    std::vector<std::string> files;
    if(req.annotation.empty()){
        getAll(fs::path(path), ".pcd", files);
    }
    else{
        getOnly(fs::path(path), ".pcd", req.annotation, files);
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 pc2;
    pcl::PCLPointCloud2 pcl_cloud;
    std::string s = "";
     while (!files.empty()){
        s = path+files.back();
        if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (s, *cloud) == -1){
            std::cout <<  "Couldn't read file " <<  s << " \n" << std::endl;
        }
        pcl::toPCLPointCloud2(*cloud, pcl_cloud);
        pcl_conversions::fromPCL(pcl_cloud, pc2);
        pc2.header.frame_id = req.annotation;
        res.set_of_points.push_back(pc2);
        files.pop_back();
  }
    return true;
}

int main (int argc, char** argv){
    ros::init (argc, argv, "annotated_pointcloud_publisher");
    ros::NodeHandle nh;

    path = ros::package::getPath("pointcloud_annotator") + "/data/";

    ros::ServiceServer service = nh.advertiseService("annotated_pointcloud_publisher/update", serviceCallback);

    ros::spin();
}