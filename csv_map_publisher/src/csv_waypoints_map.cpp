#include "../include/csv_waypoints_map.h"

namespace csv_map_publisher
{
    CsvWaypointsMap::CsvWaypointsMap(string csv_map_path)
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
            SpherePoint2D cur_point;
            if(lon_before_lat){
                cur_point.set_lon(stod(it->front()));
                cur_point.set_lat(stod(it->back()));
            }
            else {
                cur_point.set_lat(stod(it->front()));
                cur_point.set_lon(stod(it->back()));
            }
            waypoints_map_.push_back(cur_point);
            ++it;
        } 
    }

    vector<SpherePoint2D> CsvWaypointsMap::get_points_in_radius(boost::shared_ptr<SpherePoint2D> location, double radius)
    {
        vector<SpherePoint2D> close_points;
        for(vector<SpherePoint2D>::iterator it = waypoints_map_.begin();
                it != waypoints_map_.end(); ++it)
        {
            if(it->distance_to(location) <= radius)
            {
                close_points.push_back(*it);
            }
        }
        return close_points;
    }
    
} // csv_map_publisher