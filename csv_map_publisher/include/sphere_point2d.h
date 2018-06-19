#ifndef POINT_TWO_D
#define POINT_TWO_D

#include <math.h>
#include <exception>
#include <string>
#include <boost/shared_ptr.hpp>

#include "../include/point2d.h"

using namespace std;
using namespace boost;


class SpherePoint2D : Point2D
    {
    private:
        bool in_deg_;
        const double R_EARTH_ISRAEL = 6371207;
        const double ANGLE_TO_RAD = M_PI/180;

    public:
        SpherePoint2D(double lon, double lat, bool in_deg = true);
        SpherePoint2D();
        ~SpherePoint2D();

        double distance_to(boost::shared_ptr<Point2D> point_ptr);
        double haversine_distance_to(SpherePoint2D * point_ptr);

        double get_lon() {return get_x();}
        double get_lat() {return get_y();}

        void set_lon(double lon) {set_x(lon);}
        void set_lat(double lat) {set_y(lat);}
    };


SpherePoint2D::SpherePoint2D(double lon, double lat, bool in_deg = true) 
: Point2D(lon,lat)
{
    in_deg = in_deg_;
}

SpherePoint2D::SpherePoint2D():Point2D()
{
    in_deg_ = true;
}

SpherePoint2D::~SpherePoint2D() {}

double SpherePoint2D::distance_to(boost::shared_ptr<Point2D> point_ptr)
{
    boost::shared_ptr<SpherePoint2D> sphere_point_ptr = 
        boost::dynamic_pointer_cast<SpherePoint2D>(point_ptr);
    if (!sphere_point_ptr)
    {

        stringstream err_msg; 
        err_msg << "distace can be calclated only between"<<
            "same type of points\n tried to clculate distance" << 
            "from sphere point to non shpere point";
        throw (runtime_error(err_msg.str()));
    }

    double lam1, lam2, phi1, phi2;
    if(in_deg_)
    {
        lam1 = get_x() * ANGLE_TO_RAD;
        phi1 = get_y() * ANGLE_TO_RAD;
    }
    else
    {
        lam1 = get_x();
        phi1 = get_y();
    }
    if (sphere_point_ptr->in_deg_) 
    {
        lam2 = sphere_point_ptr->get_x() *ANGLE_TO_RAD;
        phi2 = sphere_point_ptr->get_y() *ANGLE_TO_RAD;
    }
    else
    {
        lam2 = sphere_point_ptr->get_x();
        phi2 = sphere_point_ptr->get_y();
    }
    
    double d_lamda = lam2- lam1;
    double d_phi = phi2 - phi1;
    double dist = sqrt(d_phi*d_phi + d_lamda*d_lamda) * R_EARTH_ISRAEL;
}

double SpherePoint2D::haversine_distance_to(SpherePoint2D * point_ptr)
{      
    double lam1, lam2, phi1, phi2;
    if(in_deg_)
    {
        lam1 = get_x() * ANGLE_TO_RAD;
        phi1 = get_y() * ANGLE_TO_RAD;
    }
    else
    {
        lam1 = get_x();
        phi1 = get_y();
    }
    if (point_ptr->in_deg_) 
    {
        lam2 = point_ptr->get_x() *ANGLE_TO_RAD;
        phi2 = point_ptr->get_y() *ANGLE_TO_RAD;
    }
    else
    {
        lam2 = point_ptr->get_x();
        phi2 = point_ptr->get_y();
    }
    
    double d_lamda = lam2- lam1;
    double d_phi = phi2 - phi1;

    double a = sin(d_phi/2)*sin(d_phi/2) +
                (cos(phi1)*cos(phi2) * sin(d_lamda/2)*sin(d_lamda/2));
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return (R_EARTH_ISRAEL * c);
}


#endif