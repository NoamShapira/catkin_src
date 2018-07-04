#include <string>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <exception>

#include <type_traits>
#include <gtest/gtest.h>

#include "../include/csv_reader.h"
#include "../include/csv_waypoints_map.h"
#include "../include/sphere_point2d.h"

using namespace std;
using namespace csv_map_publisher;


TEST(CsvReaderTest, parsingTest)
{
    string good_path = "//home/rlb10/catkin_ws/src/csv_map_publisher/csv/Arsuf_Driving_Path.csv";
    string bad_path = "//home/bad_path.csv";
    
    // test reading a known csv file
    CSVReader csv_reader(good_path);
    vector<vector<string>> csv_data = csv_reader.getData();
    ASSERT_EQ(csv_data.front().front(), "latitude");
    ASSERT_EQ(csv_data.front().back(),"longitude");
    ASSERT_EQ(csv_data.back().front(),"34.81286242818");
    ASSERT_EQ(csv_data.back().back(),"32.1961762516997");
    ASSERT_EQ(csv_data.size(), 146);
    ASSERT_EQ(csv_data.front().size(), 2);

    // test you dont change the csv when toucing the data
    csv_data.pop_back();
    ASSERT_EQ(csv_data.size(), 145);
    CSVReader csv_reader2(good_path);
    csv_data = csv_reader2.getData();
    ASSERT_EQ(csv_data.size(), 146);

    // test throwing exaption for wrong file path
    string bad_path_error_msg = "Opening file '//home/bad_path.csv' failed, it either doesn't exist or is not accessible.";
    try
    {
        CSVReader csv_reader3(bad_path);
        csv_reader3.getData();
        FAIL() << "Expected std::run_time error";
    }
    catch(const std::exception& e)
    {
        ASSERT_EQ(e.what(), bad_path_error_msg);
    } 
}

TEST(SpherePoint2DTest, classTest)
{
    boost::shared_ptr<SpherePoint2D> p1_ptr(new SpherePoint2D(32.1961762516997,34.8128624281800));
    ASSERT_EQ(p1_ptr->to_string(),"(32.1961762516997,34.81286242818)");
    boost::shared_ptr<SpherePoint2D> p2_ptr(new SpherePoint2D());
    ASSERT_EQ(p2_ptr->to_string(),"(0,0)");
}

TEST(CsvWaypointsMapTest, get_points_in_raidiusTest)
{
    string good_path = "//home/rlb10/catkin_ws/src/csv_map_publisher/csv/Arsuf_Driving_Path.csv";
    CsvWaypointsMap waypoints_map(good_path);
    boost::shared_ptr<SpherePoint2D> p1_ptr(new SpherePoint2D(32.1961762516997,34.8128624281800));
    vector<SpherePoint2D> rel_points;
    rel_points = waypoints_map.get_points_in_radius(p1_ptr, 10);
    ASSERT_EQ(rel_points.size(), 4);
    rel_points = waypoints_map.get_points_in_radius(p1_ptr, 0);
    ASSERT_EQ(rel_points.size(), 1); //only the point itself should be 0 distance from itself
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    // srand((unsigned int)random());
    return RUN_ALL_TESTS();
}
