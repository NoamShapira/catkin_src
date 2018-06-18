
#include <iostream>
#include <fstream>
#include <exception> 

#include <string>
#include <vector>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <math.h> 
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
        double lon_, lat_;
        Point2D(double lon, double lat)
        {
            lon_ = lon;
            lat_ = lat;
        };
        Point2D()
        {
            lon_ = 0;
            lat_ = 0;
        }
        double distance_from_point(Point2D target)
        {
            double dist = sqrt((lon_-target.lon_)*(lon_-target.lon_) + 
                                (lat_-target.lat_)*(lat_-target.lat_));
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
            bool lon_before_lat;
            if ((string_data.front().front() == "lon") ||
                (string_data.front().front() == "longitude")) {
                lon_before_lat = true;
            }
            else {
                lon_before_lat = false;
            }
            ++it; //start parse numbers from second row

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

        vector<Point2D> get_points_in_radius(Point2D location, double radius)
        {
            vector<Point2D> close_points;
            for(vector<Point2D>::iterator it=waypoints_map_.begin();
                 it != waypoints_map_.end(); ++it)
            {
                if(it->distance_from_point(location) < radius)
                {
                    //TODO ask dori tomorow if waypoints_map_ may be changed from the return value
                    close_points.push_back(*it);
                }
            }
            return close_points;
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
            CsvWaypointsMap waypoints_maps(csv_map_path);
            boost::function<bool(RadRequest, RadResponse)> b_f =
                     boost::bind(get_radius_waypoints, _1, _2, waypoints_maps); 
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
