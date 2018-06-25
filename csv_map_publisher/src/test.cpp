#include <string>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <exception>

#include <type_traits>

#include "../include/csv_reader.h"
#include "../include/csv_waypoints_map.h"
#include "../include/sphere_point2d.h"

using namespace std;
using namespace csv_map_publisher;

int main(int argc, char const *argv[])
{
    string good_path = "//home/rlb10/catkin_ws/src/csv_map_publisher/csv/Arsuf_Driving_Path.csv";
    string bad_path = "//home/bad_path.csv";
    
    try
    {
        cout.precision(16);
        cout << "\n---testing CSVReader---\n"<< endl;
        CSVReader csv_reader(good_path);
        cout << "created a reader" << endl;
        vector<vector<string>> csv_data = csv_reader.getData();
        
        cout << "loaded the data" << endl;
        cout << "first row is: (" << csv_data.front().front() << "," 
            << csv_data.front().back()<< ")" << endl;
        cout << "expected: (latitude,longitude)" << endl;
        
        cout << "last row is: (" << csv_data.back().front() << "," 
            << csv_data.back().back()<< ")" << endl; 
        cout << "expected: (34.81286242818,32.1961762516997)" << endl;
        
        cout << "number of rows on csv file is: " << csv_data.size() << endl;
        cout << "expected: 146" << endl;

        cout << "number of cols in first csv row is: " << csv_data.front().size() << endl;
        cout << "expected: 2" << endl;

       
        cout << "\n---testing get_points_in_radius---\n"<< endl;
        CsvWaypointsMap waypoints_map(good_path);
        boost::shared_ptr<SpherePoint2D> p1_ptr(new SpherePoint2D(32.1961762516997,34.8128624281800));
        p1_ptr->print();
        boost::shared_ptr<SpherePoint2D> p2_ptr(new SpherePoint2D());
        p2_ptr->print();
        vector<SpherePoint2D> rel_points;
        double radius_of_interse = 10;
        rel_points = waypoints_map.get_points_in_radius(p1_ptr, radius_of_interse);
        cout << "number of relevant points is: " << rel_points.size() << endl;
        if (rel_points.size() < 10) {
            
            for(vector<SpherePoint2D>::iterator it = rel_points.begin();
                         it != rel_points.end(); it++)
            {
                cout << "\t";
                it->print(); 
            }
            
        }
        
    }
    catch(const std::exception& e)
    {
        cout << "\n\nexception was thrown\ne.what: " << e.what() << endl;
        cout << "typeid(e): " << typeid(e).name() <<endl;
    }

    cout << "\n----test ended----" << endl;
    return 0;
}



