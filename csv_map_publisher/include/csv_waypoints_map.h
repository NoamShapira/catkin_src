
#ifndef CSV_WAYPOINT_MAP
#define CSV_WAYPOINT_MAP

#include <vector>
#include <string>

#include "../include/point2d.h"
#include "../include/csv_reader.h"


using namespace std;

namespace csv_map_publisher
{
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
                    cur_point.set_lon(stof(it->front()));
                    cur_point.set_lat(stof(it->back()));
                }
                else {
                    cur_point.set_lat(stof(it->front()));
                    cur_point.set_lon(stof(it->back()));
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
                    close_points.push_back(it.base);
                }
            }
            return close_points;
        }
    };
    
} // csv_map_publisher

#endif 
