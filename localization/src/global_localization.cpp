
//local includes
#include "../include/localization/global_localization.h"

GlobalLocalizingNode::GlobalLocalizingNode()
{
    // use param server
    string publisher_topic_name, imu_sub_topic_name,
            gps_sub_topic_name, rc_correction_topic_name, cam_link;
    int node_queue_size;
    double path_res;
    
    // TODO change to dnmic reconfig
    nh.getParam("publisher_topic_name", publisher_topic_name); // "global_location"
    nh.getParam("imu_sub_topic_name", imu_sub_topic_name); // "arsys/imu"
    nh.getParam("gps_sub_topic_name", gps_sub_topic_name); // "arsys/gps"
    nh.getParam("rc_correction_topic_name", rc_correction_topic_name); // "road_center_correction"
    nh.getParam("g2l_queue_size", node_queue_size); // 1000
    nh.getParam("radius_of_interest", radius_of_interest); // 50
    nh.getParam("path_fitting_deg", path_fitting_deg); // 4
    nh.getParam("cam_link", cam_link); //cam1_link 
    nh.getParam("path_resolution", path_res); // 0.5

    // node initialyzation
    
    g2l_pub = nh.advertise<localization::GlobalLocation>(publisher_topic_name, node_queue_size);
    imu_sub = nh.subscribe<sensor_msgs::Imu>
            (imu_sub_topic_name, node_queue_size, &GlobalLocalizingNode::imu_callback, this);
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
            (gps_sub_topic_name, node_queue_size, &GlobalLocalizingNode::gps_callback, this);
    rc_correction_sub = nh.subscribe<mask_processing::RoadCenterCorection>
            (rc_correction_topic_name, node_queue_size,
             &GlobalLocalizingNode::rc_correction_callback, this);
    get_waypoints_client = nh.serviceClient<csv_map_publisher::GetWaypointsInRadius>
                ("get_waypoints_in_radius");
    
    // initialize current sensor readings
    cur_gps = * ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gps_sub_topic_name);
    cur_heading_quaternions = ros::topic::waitForMessage<sensor_msgs::Imu>(imu_sub_topic_name)->orientation;

    // get transformation from cam_link to base_link
    tf::TransformListener listener;
    tf::StampedTransform transform;
    // TODO add check fransform not found
    listener.waitForTransform("base_link", cam_link, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("base_link", cam_link, ros::Time(0), transform);
    cam_to_base_transform = transform;

    // initialize correction
    lon_error = 0; // degrees needed to sabtruct from matured longtitude
    lat_error = 0; // degrees needed to sabtruct from matured latitude
}

GlobalLocalizingNode::~GlobalLocalizingNode(){}

void GlobalLocalizingNode::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    cur_heading_quaternions = msg->orientation;
}

void GlobalLocalizingNode::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    cur_gps = *msg;
    publish_global_location();
}

vector<double> GlobalLocalizingNode::transform_corections_to_base_frame(
            double heading_road_angle, double road_center_offset)
{
    double heading_road_angle_base_link, road_center_offset_base_link;

    // -1 because of the cordinate movment
    heading_road_angle_base_link = (-1) * heading_road_angle;
    // find a point on the road center in cam_link
    tf::Vector3 closest_point_in_cam_link(
        (-1)*road_center_offset*sin(heading_road_angle),
        (-1)*road_center_offset*cos(heading_road_angle), 
        cam_to_base_transform.getOrigin().getZ()); 
    // transform the point to base_link
    tf::Vector3 closest_point_in_base_link = cam_to_base_transform * closest_point_in_cam_link;
    // base link road equetion y-y1 = (x-x1)*tan(theta)
    // clculate road_center_offset_base_link in base link
    double x1 = closest_point_in_base_link.getX();
    double y1 = closest_point_in_base_link.getY();
    double road_meet_y_axis = y1 - x1*tan(heading_road_angle_base_link);
    // distance of road equation from origin
    road_center_offset_base_link = sin(heading_road_angle_base_link) * road_meet_y_axis;
   
    vector<double> ret{heading_road_angle_base_link,road_center_offset_base_link};
    // ret.push_back(heading_road_angle_base_link);
    // ret.push_back(road_center_offset_base_link);
    return ret;
}

vector<sensor_msgs::NavSatFix> GlobalLocalizingNode::use_map_service()
{
    csv_map_publisher::GetWaypointsInRadius srv;
    srv.request.location = cur_gps;
    srv.request.radius = radius_of_interest;
    vector<sensor_msgs::NavSatFix> rel_points;
    if (get_waypoints_client.call(srv))
    {
        rel_points.assign(
            srv.response.relevant_waypoints.begin(), srv.response.relevant_waypoints.end());
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

sensor_msgs::NavSatFix GlobalLocalizingNode::get_closset_point_on_path(
        vector<sensor_msgs::NavSatFix> path, int deg)
{
    // i fit a polynome to the path relative to the curent location and in meters for convenience
    vector<double> relative_lons_meters, relative_lats_meters;
    for(vector<sensor_msgs::NavSatFix>::iterator it = path.begin();
             it != path.end(); ++it)
    {
        relative_lons_meters.push_back((it->longitude - cur_gps.longitude)*R_EARTH_ISRAEL);
        relative_lats_meters.push_back((it->latitude - cur_gps.latitude)*R_EARTH_ISRAEL);
    }
    vector<double> coeff;
    // fit x,y a polyfit of degree deg
    polyfit(relative_lons_meters, relative_lats_meters, coeff, deg);
    
    // find on fit the closest point
    // sample the fit
    vector<Point2d> sampled_points;
    Point2d cur_point, clossest_point;
    for(double x = -radius_of_interest/2 ; x < radius_of_interest/2; x += path_res)
    {
        double y = 0;
        for(int i = 0; coeff.size(); i++)
        {
            y+= coeff[i] * pow(x, (double)i);
        }
        cur_point = Point2d(x,y);
        sampled_points.push_back(cur_point);
    }
    
    // find the clossest point to the sampled points
    clossest_point = Point2d::clossest2origin(sampled_points);

    // create a NavSatfix
    sensor_msgs::NavSatFix map_location;
    map_location.header = cur_gps.header;
    map_location.altitude = cur_gps.altitude;
    map_location.position_covariance = cur_gps.position_covariance;
    map_location.position_covariance_type = cur_gps.position_covariance_type;
    map_location.status = cur_gps.status;
    map_location.longitude = (clossest_point.get_x()/R_EARTH_ISRAEL)+cur_gps.longitude;
    map_location.latitude = (clossest_point.get_y()/R_EARTH_ISRAEL)+cur_gps.latitude;
    
    return map_location;
}

sensor_msgs::NavSatFix GlobalLocalizingNode::get_accurate_location_on_map_with_corections(
        vector<double> corections_in_base_frame)
{   
    // get nearby relevant points with service
    vector<sensor_msgs::NavSatFix> path_points = use_map_service();
    // get astimated location on map - the closest point to road to gps
    return get_closset_point_on_path(path_points, path_fitting_deg);
}

void GlobalLocalizingNode::rc_correction_callback(mask_processing::RoadCenterCorection msg)
{
    // recive the corrections and transform them to base_link frame
    vector<double> corections_in_base_frame = transform_corections_to_base_frame(
            msg.heading_road_angle, msg.road_center_offset);
    
    // calculate lon, lat acording to corections
    sensor_msgs::NavSatFix location_from_IP = get_accurate_location_on_map_with_corections(
        corections_in_base_frame);
    
    // update d_lon and d_lat
    double IP_lon = location_from_IP.longitude;
    double IP_lat = location_from_IP.latitude;
    double cur_gps_lon = cur_gps.longitude;
    double cur_gps_lat = cur_gps.latitude;

    lon_error = cur_gps_lon - IP_lon;
    lat_error = cur_gps_lat - IP_lat;

    // TODO maybe add memory to lon_error calculation

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
    ros::init(argc, argv, "global_localization");
    GlobalLocalizingNode g_l_node();
    ros::AsyncSpinner spinner(3); 
    spinner.start();
    // ros::spin();
    
    return 0;
}


