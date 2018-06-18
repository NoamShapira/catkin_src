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
 * A class to read data from a csv file.
 */
class CSVReader
{
	std::string fileName;
	std::string delimeter;
 
public:
	CSVReader(std::string filename, std::string delm = ",") :
			fileName(filename), delimeter(delm)
	{ }
 
	// Function to fetch data from a CSV File
	std::vector<std::vector<std::string> > getData();
};
 
/*
* Parses through csv file line by line and returns the data
* in vector of vector of strings.
*/
std::vector<std::vector<std::string> > CSVReader::getData()
{
	std::ifstream f_stream;
	f_stream.exceptions(std::ifstream::failbit);
	try 
	{
		f_stream.open(fileName);
	}
	catch (const std::exception& e) 
	{
		std::stringstream msg;
    	msg << "Opening file '" << fileName 
			<< "' failed, it either doesn't exist or is not accessible.";
    	throw std::runtime_error(msg.str());
	}
	

	std::vector<std::vector<std::string> > dataList;
 
	std::string line = "";
	// Iterate through each line and split the content using delimeter
	while (getline(f_stream, line))
	{
		std::vector<std::string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
		dataList.push_back(vec);
	}
	// Close the File
	f_stream.close();
 
	return dataList;
}
#endif
