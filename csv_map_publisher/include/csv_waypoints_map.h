
#ifndef CSV_WAYPOINT_MAP
#define CSV_WAYPOINT_MAP

#include <vector>
#include <string>

#include "../include/sphere_point2d.h"
#include "../include/csv_reader.h"


using namespace std;

namespace csv_map_publisher
{
    /*
     * @brief A class to save and have searches on 
     *  2 dimntenal waypoints given in csv file
     */
    class CsvWaypointsMap
    {
    private:
        //All the (lon, lat) waypoints int listed in the csv file
        vector<SpherePoint2D> waypoints_map_;

    public:
        /*
        * @brief the class constructor
        * @param csv_map_path is the global path to the csv file
        */
        CsvWaypointsMap(string csv_map_path);

        /*
        * @brief a function to get all waypoints in map in a 
        *   given distance from a given location
        * @param radius the radius of interest around the interest point
        * @param location the interest point
        * @return a vector of all the revanl points 
        */
        vector<SpherePoint2D> get_points_in_radius(boost::shared_ptr<SpherePoint2D> location, double radius);  
    };
} //csv_map_publisher


#endif 
