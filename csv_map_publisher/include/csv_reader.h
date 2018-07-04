#ifndef CSV_READER
#define CSV_READER

#include <fstream>
#include <iostream>
#include <sstream>
#include <exception> 
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
 
/*
* @brief A class to read data from a csv file. 
*/
class CSVReader
{
	//The path to the csv file
	std::string fileName;
	//The char of string that seperats the requested values
	std::string delimeter;
 
public:
	/*
     * @brief a simple constructor 
     * @param filename the csv path
	 * @param delm the string that seperats the values in the csv default ','
     */
	CSVReader(std::string filename, std::string delm = ",") :
			fileName(filename), delimeter(delm)
	{ }

	/*
     * @brief Function to fetch data from a CSV File
     * @return a vector of vectors of strings 
	 * 	containing the values seperated by the delm
     */
	std::vector<std::vector<std::string> > getData();
};
 
#endif
