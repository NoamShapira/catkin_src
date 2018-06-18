#include <string>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <exception>

#include "../include/csv_reader.h"

//test csv_reader
using namespace std;

int main(int argc, char const *argv[])
{
    string path = "//home/rlb10/catkin_ws/src/csv_map_publisher/csv/Arsuf_Driving_Path.csv";
    
    try
    {
        CSVReader csv_reader(path);
        cout << "created a reader" << endl;
        vector<vector<string>> csv_data = csv_reader.getData();
        cout << "loaded the data" << endl;
        cout << "the (0,0) elem is: " << csv_data.front().front() << endl; 
    
    }
    catch(const exception& e)
    {
        cout << "exception was thrown" << endl;
        cout << e.what() << endl;
        cout << typeid(e).name() <<endl;
        // cerr << e.what() << endl;
    }

    cout << " ---------- " << endl << "test ended" << endl;
    return 0;
}



