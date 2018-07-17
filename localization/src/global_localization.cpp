
// ros includes
#include <sensor_msgs/NavSatFix.h>

//local includes
#include "../include/localization/global_localization.h"

GlobalLocalizingNode::GlobalLocalizingNode(int argc, char ** argv)
{
    // use param server
    string node_name, publisher_topic_name, imu_sub_topic_name,
            gps_sub_topic_name, rc_correction_topic_name, cam_link;
    int node_queue_size;
    // TODO add parmaeters to launch
    nh.getParam("global_localization_node_name", node_name); // "global_localization"
    nh.getParam("publisher_topic_name", publisher_topic_name); // "global_location"
    nh.getParam("imu_sub_topic_name", imu_sub_topic_name); // "arsys/imu"
    nh.getParam("gps_sub_topic_name", gps_sub_topic_name); // "arsys/gps"
    nh.getParam("rc_correction_topic_name", rc_correction_topic_name); // "road_center_correction"
    nh.getParam("g2l_queue_size", node_queue_size); // 1000
    nh.getParam("radius_of_interest", radius_of_interest); // 20
    nh.getParam("cam_link", cam_link); //cam1_link 

    // node initialyzation
    init(argc, argv, node_name);
    g2l_pub = nh.advertise<localization::GlobalLocation>(publisher_topic_name, node_queue_size);
    imu_sub = nh.subscribe(imu_sub_topic_name, node_queue_size, imu_callback);
    gps_sub = nh.subscribe(gps_sub_topic_name, node_queue_size, gps_callback);
    rc_correction_sub = nh.subscribe(rc_correction_topic_name, node_queue_size, rc_correction_callback);
    get_waypoints_client = nh.serviceClient<csv_map_publisher::GetWaypointsInRadius>
                ("get_waypoints_in_radius");
    
    // initialize current sensor readings
    cur_gps = * ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gps_sub_topic_name);
    cur_heading_quaternions = ros::topic::waitForMessage<sensor_msgs::Imu>(imu_sub_topic_name)->orientation;

    // get transformation from cam_link to base_link
    tf::TransformListener listener;
    tf::StampedTransform transform;
    // TODO add check fransform not found
    listener.lookupTransform(cam_link, "base_link", ros::Time(0), transform);
    base_to_cam_transform = transform;

    // initialize correction
    lon_error = 0; // radians needed to sabtruct from matured longtitude
    lat_error = 0; // radians needed to sabtruct from matured latitude
}

GlobalLocalizingNode::~GlobalLocalizingNode(){}

void GlobalLocalizingNode::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    cur_heading_quaternions = msg->orientation;
}

void GlobalLocalizingNode::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    cur_gps = *msg;
}

vector<double> GlobalLocalizingNode::transform_corections_to_base_frame(
            double heading_road_angle, double road_center_offset)
{
    double dx, dy, heading_road_angle_base_link, road_center_offset_base_link;
    dx = base_to_cam_transform.getOrigin().x();
    dy = base_to_cam_transform.getOrigin().y();

    // -1 because of the cordinate movment
    heading_road_angle_base_link = (-1) * heading_road_angle;
    
    
    //TODO add
}

vector<sensor_msgs::NavSatFix> GlobalLocalizingNode::use_map_service()
{
    vector<sensor_msgs::NavSatFix> rel_points;
    csv_map_publisher::GetWaypointsInRadius srv;
    srv.request.location = cur_gps;
    srv.request.radius = radius_of_interest;
    if (get_waypoints_client.call(srv))
    {
        // TODO assign the array into a cpp vector
        // rel_points.assign(srv.response.relevant_waypoints;
        if(rel_points.size() < 2)
        {
            ROS_ERROR("found less then 2 points try increasing the 'radius_of_interest'");
        }
    }
    else
    {
       ROS_ERROR("Failed to call get_waypoints_in_radius");
    }
    return rel_points;
}

vector<double> GlobalLocalizingNode::get_accurate_location_on_map_with_corections(
        vector<double> corections_in_base_frame)
{
    // get nearby relevant points with service
    vector<sensor_msgs::NavSatFix> rel_points = use_map_service();
    // 
    // TODO
}

void GlobalLocalizingNode::rc_correction_callback(const mask_processing::RoadCenterCorection& msg)
{
    // recive the corrections and transform them to base_link frame
    vector<double> corections_in_base_frame = transform_corections_to_base_frame(
            msg.heading_road_angle, msg.road_center_offset);
    
    // calculate lon, lat acording to corections
    vector<double> location_from_IP = get_accurate_location_on_map_with_corections(
        corections_in_base_frame);
    
    // update d_lon and d_lat
    double IP_lon = location_from_IP.front();
    double IP_lat = location_from_IP.back();
    double cur_gps_lon = cur_gps.longitude;
    double cur_gps_lat = cur_gps.latitude;

    lon_error = cur_gps_lon - IP_lon;
    lat_error = cur_gps_lat - IP_lat;

}

void GlobalLocalizingNode::publish_global_location()
{
    localization::GlobalLocation location;
    
    sensor_msgs::NavSatFix gps_to_publish;
    gps_to_publish = cur_gps;
    gps_to_publish.latitude -= lat_error;
    gps_to_publish.longitude -= lon_error;

    location.orientation = cur_heading_quaternions; //assumes heading from nav system is accurate
    location.position = gps_to_publish;
    location.header = gps_to_publish.header;

    g2l_pub.publish(location);
}


int main(int argc, char  *argv[])
{
    GlobalLocalizingNode g_l_node(argc, argv);
    //TODO use unsynchronyzed spin for subscribers and while(rate) for publisher
    
    g_l_node.publish_global_location();
    return 0;
}


