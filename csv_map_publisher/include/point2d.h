#ifndef POINT_TWO_D
#define POINT_TWO_D

#include <math.h>

class Point2D
    {
    private:
        double lon_, lat_;

    public:
        
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

        double get_lon() {return lon_;}
        double get_lat() {return lat_;}

        void set_lon(double lon) {lon_ = lon;}
        void set_lat(double lat) {lat_ = lat;}
    };

#endif