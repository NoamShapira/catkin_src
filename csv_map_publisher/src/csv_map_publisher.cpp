
#include <iostream>
#include <fstream>

#include <string>
#include <vector>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>

// TODO add multi threded services
// #include "csv_map_publisher/GetForewardWaypointsInRadius.h"
// #include "csv_map_publisher/GetRoadAzimuth.h"
#include "csv_map_publisher/GetWaypointsInRadius.h"

#include "../include/csv_reader.h"

using namespace std;
using namespace csv_map_publisher;
using namespace boost;
using namespace ros;

namespace csv_map_publisher
{
    typedef GetWaypointsInRadius::Response RadResponse;
    typedef GetWaypointsInRadius::Request RadRequest;

    class Point2D
    {
    public:
        float lon_, lat_;
        Point2D(float lon, float lat)
        {
            lon_ = lon;
            lat_ = lat;
        };
        Point2D()
        {
            lon_ = 0;
            lat_ = 0;
        }
    };

    class CsvWaypointsMap
    {
    private:
        vector<Point2D> waypoints_map_;

    public:
        CsvWaypointsMap(string csv_map_path)
        {

            CSVReader csv_reader(csv_map_path);
            vector<vector<string>> string_data = csv_reader.getData();
            vector<vector<string>>::iterator it= string_data.begin();
            //start from the second row, asumes all rows exept first are only numbers
            //check if lon_before_lat
            bool lon_before_lat;
            if (string_data.front().front() == "lat") {
                lon_before_lat = true;
            }
            else {
                lon_before_lat = false;
            }
            ++it;

            while (it!=string_data.end()) {
                Point2D cur_point;
                if(lon_before_lat){
                    cur_point.lon_ = stof(it->front());
                    cur_point.lat_ = stof(it->back());
                }
                else {
                    cur_point.lat_ = stof(it->front());
                    cur_point.lon_ = stof(it->back());
                }
                waypoints_map_.push_back(cur_point);
                ++it;
            } 
        }
        vector<Point2D> get_points_in_radius(Point2D location, float radius)
        {
            // TODO
        }
    };


    bool get_radius_waypoints(RadRequest &req, RadResponse &res, CsvWaypointsMap map)
    {
        Point2D location(req.location.longitude, req.location.latitude);
        vector<Point2D> rel_points = map.get_points_in_radius(location, req.radius);
        for (vector<Point2D>::iterator it = rel_points.begin(); it!=rel_points.end(); ++it)
        {
            sensor_msgs::NavSatFix cur_navsatfix;
            cur_navsatfix.header = req.location.header;
            cur_navsatfix.status = req.location.status;
            cur_navsatfix.longitude = it->lon_;
            cur_navsatfix.latitude = it->lat_;
            cur_navsatfix.altitude = 0; //right now the map does not contain hight
            cur_navsatfix.position_covariance = {0,0,0, 0,0,0, 0,0,0};
            cur_navsatfix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

            res.relevant_waypoints.push_back(cur_navsatfix);
        }
        return true;
    };

    int main(int argc, char **argv)
    {
        CsvWaypointsMap waypoints_maps(string("a ")); // TODO add ros param
        ros::init(argc, argv, "map_publlisher");
        ros::NodeHandle n;
        boost::function<bool(RadRequest, RadResponse)> b_f =
                     boost::bind(get_radius_waypoints, _1, _2, waypoints_maps); 
        ros::ServiceServer service = 
            n.advertiseService<RadRequest,RadResponse>("get_waypoints_in_radius",b_f);
        ros::spin();

        return 0;
    };
}; // namespace csv_map_publisher
