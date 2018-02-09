#include <algorithm>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

int more;
ros::Publisher pub;
std::string in_topic;
double width, length, tolerance;
int red_value, green_value, blue_value, sx, sy;

void cloud_callback (const sensor_msgs::PointCloud2& msg){

    more = 0;

    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(msg, pcl_cloud);

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromPCLPointCloud2(pcl_cloud, cloud);
    pcl::PointCloud<pcl::PointXYZRGB> cloud2 = pcl::PointCloud<pcl::PointXYZRGB>(cloud);

    for(unsigned i=0; i<cloud.width; i+=sy){
        for(unsigned j=0; j<cloud.height; j+=sx){
            unsigned no_nan = 0;
            /*for(unsigned k=0;k<width;k++){
                for(unsigned l=0;l<length;l++){
                    if(j+l < cloud.height and i+k < cloud.width){
                        if(cloud.at(i+k,j+l).x == cloud.at(i+k,j+l).x and cloud.at(i+k,j+l).y == cloud.at(i+k,j+l).y and cloud.at(i+k,j+l).z == cloud.at(i+k,j+l).z){
                            nan_only = false;
                            no_nan++;
                            cloud.at(i+k,j+l).r = red_value;
                            cloud.at(i+k,j+l).g = green_value;
                            cloud.at(i+k,j+l).b = blue_value;
                        }
                    }
                }
            }*/
            double maxw = 0.f;
            double maxl = 0.f;
            for(unsigned k=i;k<cloud.width;k++){
                for(unsigned l=j;l<cloud.height;l++){
                    if(!(std::isnan(cloud.at(k,l).x) or 
                        std::isnan(cloud.at(k,l).y) or 
                        std::isnan(cloud.at(k,l).z) or 
                        std::isinf(cloud.at(k,l).x) or 
                        std::isinf(cloud.at(k,l).y) or 
                        std::isinf(cloud.at(k,l).z) or 
                        std::isnan(cloud.at(i,j).x) or 
                        std::isnan(cloud.at(i,j).y) or 
                        std::isnan(cloud.at(i,j).z) or 
                        std::isinf(cloud.at(i,j).x) or 
                        std::isinf(cloud.at(i,j).y) or 
                        std::isinf(cloud.at(i,j).z))){
                        no_nan++;
                        double ll = cloud.at(i,j).x - cloud.at(k,l).x;
                        double w = cloud.at(i,j).y - cloud.at(k,l).y;
                        // I can't get abs to work?! wtf!
                        if(ll < 0){
                            ll = -ll;
                        }
                        if(w < 0){
                            w = -w;
                        }
                        if(ll <= length and w <= width){
                            cloud.at(k,l).r = red_value;
                            cloud.at(k,l).g = green_value;
                            cloud.at(k,l).b = blue_value;
                            if(ll > maxl){
                                maxl = ll;
                            }
                            if(w > maxw){
                                maxw = w;
                            }
                        }
                        if(abs(cloud.at(i,j).x - cloud.at(k,l).x) > length and abs(cloud.at(i,j).y - cloud.at(k,l).y) > width){
                            k = cloud.width;
                            l = cloud.height;
                        }
                    }
                }
            }
            if(maxl >= length-tolerance and maxw >= width-tolerance){
                sensor_msgs::PointCloud2 output;
                pcl::toPCLPointCloud2(cloud, pcl_cloud);
                pcl_conversions::fromPCL(pcl_cloud, output);
                pub.publish (output);
                cloud = pcl::PointCloud<pcl::PointXYZRGB>(cloud2);
                std::cout << "\033[1;33m====================================\033[0m" << std::endl;
                std::cout << "\033[1;33m=======\033[0m \033[1;31mPointCloud Annotator\033[0m \033[1;33m=======\033[0m" << std::endl;
                std::cout << "\033[1;33m===============\033[0m \033[1;31mHelp\033[0m \033[1;33m===============\033[0m" << std::endl;
                std::cout << "\033[1;33m====================================\033[0m" << std::endl;
                std::cout << "\033[1;34m-q\033[0m \033[1;34m--quit\033[0m: quit program" << std::endl;
                std::cout << "\033[1;34m-s\033[0m \033[1;34m--skip\033[0m: skip current points" << std::endl;
                std::cout << "\033[1;34mAnything else\033[0m: annotation name" << std::endl;
                std::cout << "- Annotate the \033[1;33mhighlighted\033[0m points: -\n";
                std::string input;
                std::cin>>input;
                std::transform(input.begin(), input.end(), input.begin(), ::tolower);
                if(input == "--quit" or input == "-q"){
                    i = cloud.width;
                    j = cloud.height;
                    more = -1;
                }
                else if(input != "--skip" and input != "-s"){

                }
            }
        }
    }
    if(more == 0){
        std::string input = "answer";
        while(input != "yes" and input != "y" and input!="no" and input!= "n"){
            std::cout << "This pointcloud has been annotated. Do you want to continue to the next one? ((y)es/(n)o)" << std::endl;
            std::cin>>input;
            std::transform(input.begin(), input.end(), input.begin(), ::tolower);
        }
        if(input == "yes" or input == "y"){
            more = 1;
        }
        else{
            more = -1;
        }
    }
}


int main (int argc, char** argv){
    ros::init (argc, argv, "pointcloud_annotator");
    ros::NodeHandle nh;

    nh.param("pointcloud_annotator/input_topic", in_topic, std::string("zed/point_cloud/cloud_registered/limitcloud"));

    nh.param("pointcloud_annotator/width", width, 0.55);
    nh.param("pointcloud_annotator/length", length, 0.65);
    nh.param("pointcloud_annotator/tolerance", tolerance, 0.20);

    nh.param("pointcloud_annotator/slide_x", sx, 20);
    nh.param("pointcloud_annotator/slide_y", sy, 20);

    nh.param("pointcloud_annotator/red_value", red_value, 255);
    nh.param("pointcloud_annotator/green_value", green_value, 255);
    nh.param("pointcloud_annotator/blue_value", blue_value, 0);

    ros::Subscriber sub = nh.subscribe (in_topic, 1, cloud_callback);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud_annotator/points", 1);

    more = true;
    while(ros::ok()){
        if(more == 1){
            ros::spinOnce();
        }
        else if(more == -1){
            std::cout << "Goodbye!" << std::endl;
            break;
        }
    }
}