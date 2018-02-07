#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

bool more;
ros::Publisher pub;
std::string in_topic;
float robot_width, robot_length;
int red_value, green_value, blue_value;

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& msg){

    more = false;
    sensor_msgs::PointCloud2 output;
    output.header.frame_id = msg->header.frame_id;
    output.height = msg->height;
    output.width = msg->width;
    output.fields = msg->fields;
    output.is_bigendian = msg->is_bigendian;
    output.point_step = msg->point_step;
    output.row_step = output.width * output.point_step;
    output.is_dense = msg->is_dense;
    bool done = false;
    int cnt = 0;
    sensor_msgs::PointCloud2Modifier pcd_modifier(output);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(output, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(output, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(output, "b");
    sensor_msgs::PointCloud2Iterator<float> iter_x2(output, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y2(output, "y");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r2(output, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g2(output, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b2(output, "b");
    while(!done){
        output.header.stamp = ros::Time::now();
        output.data = msg->data;
        for (iter_x=iter_x2, iter_y=iter_y2, iter_r = iter_r2, iter_g=iter_g2, iter_b=iter_b2;iter_x != iter_x.end();++iter_x, ++iter_y, ++iter_r, ++iter_g, ++iter_b){
            if(!(*iter_y > robot_width/2.0f or *iter_y < -robot_width/2.0f)){
                *iter_r = red_value;
                *iter_g = green_value;
                *iter_b = blue_value;
            }
        }
        done = true;
        pub.publish (output);
    }
    more = true;
}


int main (int argc, char** argv){
    ros::init (argc, argv, "pointcloud_annotator");
    ros::NodeHandle nh;

    nh.param("pointcloud_annotator/input_topic", in_topic, std::string("zed/point_cloud/cloud_registered/limitcloud"));
    nh.param("pointcloud_annotator/robot_width", robot_width, 0.55f);
    nh.param("pointcloud_annotator/robot_length", robot_length, 0.65f);
    nh.param("red_value", red_value, 255);
    nh.param("green_value", green_value, 255);
    nh.param("blue_value", blue_value, 0);
    ros::Subscriber sub = nh.subscribe (in_topic, 1, cloud_callback);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud_annotator/points", 1);

    more = true;
    while(ros::ok()){
        if(more){
            ros::spinOnce();
        }
    }
}