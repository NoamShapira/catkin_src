#include "../include/sphere_point2d.h"

SpherePoint2D::SpherePoint2D(long double lon, long double lat)
    : Point2D(lon,lat), in_deg_(true)
{}

SpherePoint2D::SpherePoint2D(long double lon, long double lat, bool in_deg) 
    : Point2D(lon,lat), in_deg_(in_deg)
{}

SpherePoint2D::SpherePoint2D()
    : Point2D(), in_deg_(true)
{}

SpherePoint2D::~SpherePoint2D() 
{}

double SpherePoint2D::distance_to(boost::shared_ptr<Point2D> point_ptr)
{
    boost::shared_ptr<SpherePoint2D> sphere_point_ptr;
    sphere_point_ptr = boost::dynamic_pointer_cast<SpherePoint2D>(point_ptr);
    if (!sphere_point_ptr)
    {

        stringstream err_msg; 
        err_msg << "distace can be calclated only between"<<
            "same type of points\n tried to clculate distance" << 
            "from sphere point to non shpere point";
        throw (runtime_error(err_msg.str()));
    }

    //A convention names for angles in radians
    double lam1, lam2, phi1, phi2;
    if(in_deg_)
    {
        lam1 = get_x() * DEG_TO_RAD;
        phi1 = get_y() * DEG_TO_RAD;
    }
    else
    {
        lam1 = get_x();
        phi1 = get_y();
    }
    if (sphere_point_ptr->in_deg_) 
    {
        lam2 = sphere_point_ptr->get_x() *DEG_TO_RAD;
        phi2 = sphere_point_ptr->get_y() *DEG_TO_RAD;
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

double SpherePoint2D::haversine_distance_to(boost::shared_ptr<SpherePoint2D>  point_ptr)
{   
    //A convention names for angles in radians
    double lam1, lam2, phi1, phi2;
    if(in_deg_)
    {
        lam1 = get_x() * DEG_TO_RAD;
        phi1 = get_y() * DEG_TO_RAD;
    }
    else
    {
        lam1 = get_x();
        phi1 = get_y();
    }
    if (point_ptr->in_deg_) 
    {
        lam2 = point_ptr->get_x() *DEG_TO_RAD;
        phi2 = point_ptr->get_y() *DEG_TO_RAD;
    }
    else
    {
        lam2 = point_ptr->get_x();
        phi2 = point_ptr->get_y();
    }
    
    double d_lamda = lam2- lam1;
    double d_phi = phi2 - phi1;

    //The haversine formula for distance calaulation a the surface of a sphere
    double a = sin(d_phi/2)*sin(d_phi/2) +
                (cos(phi1)*cos(phi2) * sin(d_lamda/2)*sin(d_lamda/2));
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return (R_EARTH_ISRAEL * c);
}

