#ifndef GLOBAL_LOCALIZATION
#define GLOBAL_LOCALIZATION

// c++ libraries
#include <string>

// ros standarts libreries
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>

// local libreries
#include "mask_processing/RoadCenterCorection.h"
#include "localization/GlobalLocation.h"
#include "csv_map_publisher/GetWaypointsInRadius.h"


using namespace ros;
using namespace std;

class GlobalLocalizingNode
{
private:
    sensor_msgs::NavSatFix cur_gps_signal;
    geometry_msgs::Quaternion cur_heading_quaternions;
    NodeHandle nh;
    Publisher g2l_pub;
    Subscriber imu_sub, gps_sub;
    ServiceClient get_waypoints_client;

    double d_lon, d_lat, d_yaw;

public:
    GlobalLocalizingNode(int argc, char ** argv);
    ~GlobalLocalizingNode();
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg_ptr);
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg_ptr);
    void publish_global_location();
};



GlobalLocalizingNode::GlobalLocalizingNode(int argc, char ** argv)
{
    // use param server
    string node_name, publisher_topic_name, imu_sub_topic_name, gps_sub_topic_name;
    int node_queue_size;
    nh.getParam("global_localization_node_name", node_name); // "global_localization"
    nh.getParam("publisher_topic_name", publisher_topic_name); // "global_location"
    nh.getParam("imu_sub_topic_name", imu_sub_topic_name); // "arsys/imu"
    nh.getParam("gps_sub_topic_name", gps_sub_topic_name); // "arsys/gps"
    nh.getParam("g2l_queue_size", node_queue_size); // 1000

    // node initialyzation
    init(argc, argv, node_name);
    g2l_pub = nh.advertise<localization::GlobalLocation>(publisher_topic_name, node_queue_size);
    imu_sub = nh.subscribe(imu_sub_topic_name, node_queue_size, imu_callback);
    gps_sub = nh.subscribe(gps_sub_topic_name, node_queue_size, gps_callback);
    get_waypoints_client = nh.serviceClient<csv_map_publisher::GetWaypointsInRadius>
                ("get_waypoints_in_radius");
    
    // initialize current sensor readings
    cur_gps_signal = * ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gps_sub_topic_name);
    cur_heading_quaternions = ros::topic::waitForMessage<sensor_msgs::Imu>(imu_sub_topic_name)->orientation;

    // initialize corection
    d_lon = 0; // meters
    d_lat = 0; // meters
    d_yaw = 0; // radians
}

GlobalLocalizingNode::~GlobalLocalizingNode(){}

void GlobalLocalizingNode::imu_callback(const sensor_msgs::Imu::ConstPtr& msg_ptr)
{

}

void GlobalLocalizingNode::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg_ptr)
{

}

void GlobalLocalizingNode::publish_global_location()
{

}




int main(int argc, char  *argv[])
{
    GlobalLocalizingNode g_l_node(argc, argv);
    //TODO find a way to spin the subscribers while publishing
    g_l_node.publish_global_location();
    return 0;
}

#endif
