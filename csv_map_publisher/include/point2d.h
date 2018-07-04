#ifndef POINT2D
#define POINT2D

#include <boost/shared_ptr.hpp>
#include <string>
#include <iostream>

using namespace std;

/*
 * @brief A interface for 2 dimentional points 
 *  contain simple getters and setters and a virtual distance_to mathod
 */
class Point2D
{
private:
    //The 2 values of the 2d point
    long double x_, y_;
public:
    /*
     * @brief A full constructor
     * @param x first cordinate value
     * @param y second cordinate value
     */
    Point2D(long double x, long double y);

    /*
     * @brief empty constructor (x,y) = (0,0)
     */
    Point2D();

    /*
     * @brief distructor
     */
    ~Point2D();

    /*
     * @brief getter for the first cordinate
     * @return first cordinate value
     */
    long double get_x();

    /*
     * @brief getter for the second cordinate
     * @return second cordinate value
     */
    long double get_y();

    /*
     * @brief setter for first cordinate
     * @param x the valuu for first cordinate
     */
    void set_x(long double x);
    
    /*
     * @brief setter for second cordinate
     * @param x the valuu for second cordinate
     */
    void set_y(long double y);

    /*
     * @brief a simple method that prints the points values to the standart output
     *  this is a virtual but not pure virtual method
     */
    virtual void print();

    /*
     * @brief a simple method that returns the points values as a string "(x,y)"
     * @return a string representarion of the point
     */
    virtual string to_string();

    /*
     * @brief this interface only pure vertual method,
     *  this method is virtual because distance is calulated difrent for difrent types of points
     * @param point_ptr a pointer to another point
     * @return the distance to the other point in meters
     */
    virtual double distance_to(boost::shared_ptr<Point2D> point_ptr)=0;
};

#endif