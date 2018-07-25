#ifndef GLOBAL_LOCALIZATION
#define GLOBAL_LOCALIZATION

// c++ libraries
#include <string>
#include <vector>
#include <math.h>

// ros standarts libreries
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>

// local libreries
#include "mask_processing/RoadCenterCorection.h"
#include "localization/GlobalLocation.h"
#include "csv_map_publisher/GetWaypointsInRadius.h"
#include "../libs/local_math_lib.h"


using namespace ros;
using namespace std;

class GlobalLocalizingNode
{
private:
    sensor_msgs::NavSatFix cur_gps;
    geometry_msgs::Quaternion cur_heading_quaternions;
    tf::StampedTransform cam_to_base_transform;
    NodeHandle nh;
    Publisher g2l_pub;
    Subscriber imu_sub, gps_sub, rc_correction_sub;
    ServiceClient get_waypoints_client;

    double radius_of_interest;
    int path_fitting_deg;
    double path_res;
    
    /*
     * gps_location - location_error = IP_location
     */
    double lon_error, lat_error;

    /*
    * @brief a private method to transform the errors recived from IP to base link frame
    * @param heading_road_angle the angle from the road center line in camer_frame as
    *   recived from IP,  possitive is right of forward direction like yaw
    * @param road_center_offset the distance from the road center line in camera_frame
    *   as recived from IP, possitive is right of forward direction
    * @return vector<double> 2 parameters vector, 
    *   first is heading angle from road center in base link frame 
    *       zero with x axis and rising counter clockwise
    *   second is offset from road center in base link frame
    *       possitive when road is to the left of vehile - with y axis
    */
    vector<double> transform_corections_to_base_frame(
            double heading_road_angle, double road_center_offset);

    /*
    * @brief a private method to get the location on ,ap according to corrections 
    *   recived from image proccecing
    * @param corections_in_base_frame are the corections in base link frame 2 doubles
    *   first is heading angle from road center
    *   second is  offset from road center
    * @return vector<double> 2 parameters vector, 
    *   first is longtitude in map according to image proccecing 
    *   second is latitude in map according to image proccecing 
    */
    sensor_msgs::NavSatFix get_accurate_location_on_map_with_corections(
        vector<double> corections_in_base_frame);

    /*
    * @brief a private method that uses the map service
    * @return vector<sensor_msgs::NavSatFix> all the relevant points from map in vector
    */
    vector<sensor_msgs::NavSatFix> GlobalLocalizingNode::use_map_service();

    /*
    * @brief a function that estimats a polynomial path from given waypoints and 
    *   returns the  clossest point to the current location on htat polynome
    * @param path a vector of the close waypoints
    * @param deg the degree of the fitted polynomial
    * @return the closest point to the current location on the estimated polynome path
    */
    sensor_msgs::NavSatFix GlobalLocalizingNode::get_closset_point_on_path(
        vector<sensor_msgs::NavSatFix> path, int deg);

public:
    GlobalLocalizingNode(int argc, char ** argv);
    ~GlobalLocalizingNode();

    /*
    * @brief the callback function used by imu_sub that updates the
    *   current orientation from the navigation system
    * @param msg is the Imu recived from navigation system
    */
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

    /*
    * @brief the callback function used by gps_sub that updates the
    *   current gps location from the navigation system
    * @param msg is the NavSatFix recived from navigation system
    */
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

    /*
    * @brief the callback function used by rc_correction_sub that updates
    *   the astimation og lon_error and lat_error according to road center correction
    * @param msg is the RoadCenterCorection recived from image processing
    */
    void rc_correction_callback(const mask_processing::RoadCenterCorection& msg);

    /*
    * @brief the function that publish the position astimated on map 
    *      according to corrections recived from image proccecing
    */
    void publish_global_location();
};

#endif