#ifndef POINT2D
#define POINT2D

#include <boost/shared_ptr.hpp>

using namespace boost;

class Point2D
{
private:
    double x_, y_;
public:
    Point2D(double x, double y);
    Point2D::Point2D();
    ~Point2D();
    double get_x();
    double get_y();
    void set_x(double x);
    void set_y(double y);

    virtual double distance_to(boost::shared_ptr<Point2D> point_ptr)=0;
};


Point2D::Point2D(double x, double y)
{
    x_ = x;
    y_ = y;
}
Point2D::Point2D()
{
    x_ = 0;
    y_ = 0;
}

Point2D::~Point2D()
{
}

double Point2D::get_x() {return x_;}
double Point2D::get_y() {return y_;}
void Point2D::set_x(double x) {x_ = x;}
void Point2D::set_y(double y) {y_ = y;}


#endif