
// #include <iostream>
// #include <fstream>
#include <exception> 

#include <string>
#include <vector>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

// TODO add multi threded services
// #include "csv_map_publisher/GetForewardWaypointsInRadius.h"
// #include "csv_map_publisher/GetRoadAzimuth.h"
#include "csv_map_publisher/GetWaypointsInRadius.h"

#include "../include/csv_reader.h"
#include "../include/csv_waypoints_map.h"

using namespace std;
using namespace csv_map_publisher;
using namespace boost;
using namespace ros;

namespace csv_map_publisher
{
    typedef GetWaypointsInRadius::Response RadResponse;
    typedef GetWaypointsInRadius::Request RadRequest;

    bool get_radius_waypoints(RadRequest &req, RadResponse &res, CsvWaypointsMap map)
    {
        boost::shared_ptr<SpherePoint2D> 
            location_ptr(new SpherePoint2D(req.location.longitude, req.location.latitude));
        vector<SpherePoint2D> rel_points = map.get_points_in_radius(location_ptr, req.radius);
        for (vector<SpherePoint2D>::iterator it = rel_points.begin(); it!=rel_points.end(); ++it)
        {
            sensor_msgs::NavSatFix cur_navsatfix;

            cur_navsatfix.header = req.location.header;
            cur_navsatfix.status = req.location.status;
            cur_navsatfix.longitude = it->get_lon();
            cur_navsatfix.latitude = it->get_lat();
            cur_navsatfix.altitude = 0; //right now the map does not contain hight
            cur_navsatfix.position_covariance = {0,0,0, 0,0,0, 0,0,0};
            cur_navsatfix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

            res.relevant_waypoints.push_back(cur_navsatfix);
        }
        return true;
    };

    int main(int argc, char **argv)
    {
        string csv_map_path;
        ros::init(argc, argv, "map_publlisher");
        ros::NodeHandle n;
        if (n.hasParam("csv_map_path")) 
        {
            n.getParam("csv_map_path", csv_map_path);
            ROS_INFO("Got param: csv_map_path");
        }
        else {ROS_ERROR("param csv_map_path not found");}

        try 
        { 
            CsvWaypointsMap waypoints_map(csv_map_path);
            boost::function<bool(RadRequest, RadResponse)> b_f =
                     boost::bind(get_radius_waypoints, _1, _2, waypoints_map); 
            ros::ServiceServer service = 
            n.advertiseService<RadRequest,RadResponse>("get_waypoints_in_radius",b_f);
            ros::spin();
        }
        catch (std::exception& e)
        {
            ROS_ERROR(e.what());
        }
      
        return 0;
    };
}; // namespace csv_map_publisher
