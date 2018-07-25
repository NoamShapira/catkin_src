
#ifndef LOCAL_MATH_LIB
#define LOCAL_MATH_LIB

#include <Eigen/QR>
#include <stdio.h>
#include <vector>

const double R_EARTH_ISRAEL = 6371207;
const double DEG_TO_RAD = M_PI/180;

using namespace std;

void polyfit(const vector<double> &xv, const vector<double> &yv, vector<double> &coeff, int order)
{
	Eigen::MatrixXd A(xv.size(), order+1);
	Eigen::VectorXd yv_mapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
	Eigen::VectorXd result;

	assert(xv.size() == yv.size());
	assert(xv.size() >= order+1);

	// create matrix
	for (size_t i = 0; i < xv.size(); i++)
	for (size_t j = 0; j < order+1; j++)
		A(i, j) = pow(xv.at(i), j);

	// solve for linear least squares fit
	result = A.householderQr().solve(yv_mapped);

	coeff.resize(order+1);
	for (size_t i = 0; i < order+1; i++)
		coeff[i] = result[i];
}



class Point2d
{
private:
	double x,y;
public:
	Point2d();
	Point2d(double xv, double yv);
	~Point2d();
	double distance(Point2d other);
	double get_x(){return x;}
	double get_y(){return y;}

	static Point2d clossest2origin(const vector<Point2d> points);
	double get_L2norm();
};

Point2d::Point2d(): x(0.0),y(0.0) {}
Point2d::Point2d(double xv, double yv): x(xv), y(yv) {}
Point2d::~Point2d() {}
double Point2d::distance(Point2d other)
{
	return( sqrt((x-other.get_x())*(x-other.get_x()) + (y-other.get_y())*(y-other.get_y())));
}

double Point2d::get_L2norm()
{
	Point2d origin(0.0,0.0);
	return (distance(origin));
}


Point2d Point2d::clossest2origin(vector<Point2d> points)
{
	Point2d cur_min;
	double min_dist = numeric_limits<double>::max();
	
	for(vector<Point2d>::iterator it = points.begin(); it != points.end(); it++)
	{
		if(it->get_L2norm() <= min_dist){
			min_dist = it->get_L2norm();
			cur_min = * it;
		}
	}
	return cur_min;
}

#endif