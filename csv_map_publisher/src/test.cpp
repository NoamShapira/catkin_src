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
        cout << "\n---testing CSVReader---\n"<< endl;
        CSVReader csv_reader(good_path);
        cout << "created a reader" << endl;
        vector<vector<string>> csv_data = csv_reader.getData();
        cout << "loaded the data" << endl;
        cout << "first row is: (" << csv_data.front().front() << "," 
            << csv_data.front().back()<< ")" << endl;
        cout << "last row is: (" << csv_data.back().front() << "," 
            << csv_data.back().back()<< ")" << endl; 
        cout << "number of rows on csv file is: " << csv_data.size() << endl;
        cout << "number of cols in first csv row is: " << csv_data.front().size() << endl;

        cout << "\n---testing get_points_in_radius---\n"<< endl;
        CsvWaypointsMap waypoints_map(good_path);
        // vector<SpherePoint2D> relevant_points;
        SpherePoint2D p(1.0,0);
        SpherePoint2D p();
        
    }
    catch(const std::exception& e)
    {
        cout << "exception was thrown\ne.what: " << e.what() << endl;
        cout << "typeid(e): " << typeid(e).name() <<endl;
    }

    cout << " ---------- " << endl << "test ended" << endl;
    return 0;
}



