#ifndef POINT_TWO_D
#define POINT_TWO_D

#include <math.h>
#include <exception>
#include <string>
#include <sstream>
#include <boost/shared_ptr.hpp>

#include "../include/point2d.h"

using namespace std;
using namespace boost;

/*
 * @brief A class that represent a 2 dimentional point on a shpere (lon, lat)
 */
class SpherePoint2D : public Point2D
    {
    private:
        //This is a boolean that matks wither the value is metured in degrees (or radians)
        bool in_deg_;
        //A constant for distance calculations
        const double R_EARTH_ISRAEL = 6371207;
        //A constant for convertions between difrent angle units
        const double DEG_TO_RAD = M_PI/180;

    public:
        /*
         * @brief A full constructor
         * @param lon represents longtitude
         * @param lat represents latitude
         * @param in_deg represents whether the the values are metured in degrees
         */
        SpherePoint2D(double lon, double lat, bool in_deg);

        /*
         * @brief A constructor that asumes the values are in degrees 
         *  like SpherePoint2D(lon,lat,true)
         * @param lon represents longtitude
         * @param lat represents latitude
         */
        SpherePoint2D(double lon, double lat);

        /*
         * @brief A empty constructor like SpherePoint2D(0,0,true)
         */
        SpherePoint2D();

        /*
         * @brief A destructor
         */
        ~SpherePoint2D();

        /*
         * @brief the implementation for virtual method in point, calculate the
         *  distance in meters to a given point. if the point is not SpherePoint2D
         *  an exception will be thrown
         * @param point_ptr A pointer to the destenation point
         * @return Thr distance in meters
         */
        double distance_to(boost::shared_ptr<Point2D> point_ptr);

        /* 
         * @brief the real calculation of distance between two point on the serface
         *  of a sphere
         * @param point_ptr A pointer to the destenation point
         * @return Thr distance in meters  
         */
        double haversine_distance_to(boost::shared_ptr<SpherePoint2D> point_ptr);

        /*
         * @brief A wraper for get_x() for convinience 
         * @return the longtitude (or x) value
         */
        double get_lon() {return get_x();}

        /*
         * @brief A wraper for get_y() for convinience 
         * @return the latitude (or y) value
         */
        double get_lat() {return get_y();}

        /*
         * @brief A wraper for set_x() for convinience 
         * @param lon  longtitude (or x) value
         */
        void set_lon(double lon) {set_x(lon);}

        /*
         * @brief A wraper for set_y() for convinience 
         * @return latitude (or y) value
         */
        void set_lat(double lat) {set_y(lat);}
    };


#endif