#include <string>
#include <sstream>

#include "../include/point2d.h"

Point2D::Point2D(long double x, long double y): x_(x), y_(y)
{}
Point2D::Point2D(): x_(0), y_(0)
{}

Point2D::~Point2D()
{}

void Point2D::print() 
{
    cout.precision(16);
    cout << "(" << x_ << ", " << y_ << ")\n";
}
string Point2D::to_string()
{
    stringstream msg;
    msg.precision(16);
    msg << "(" << get_x() << "," << get_y() << ")";
    return msg.str();   
}
long double Point2D::get_x() {return x_;}
long double Point2D::get_y() {return y_;}
void Point2D::set_x(long double x) {x_ = x;}
void Point2D::set_y(long double y) {y_ = y;}