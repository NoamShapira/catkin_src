#include "../include/point2d.h"


Point2D::Point2D(double x, double y): x_(x), y_(y)
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
double Point2D::get_x() {return x_;}
double Point2D::get_y() {return y_;}
void Point2D::set_x(double x) {x_ = x;}
void Point2D::set_y(double y) {y_ = y;}