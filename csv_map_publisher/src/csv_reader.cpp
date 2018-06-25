#include "../include/csv_reader.h"

/*
* Parses through csv file line by line and returns the data
* in vector of vector of strings.
*/
std::vector<std::vector<std::string> > CSVReader::getData()
{
	std::ifstream f_stream(fileName);
	std::vector<std::vector<std::string> > dataList;
	std::string line = "";
	if( f_stream.is_open())
	{
		// Iterate through each line and split the content using delimeter
		while (std::getline(f_stream, line))
		{
			std::vector<std::string> vec;
			boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
			dataList.push_back(vec);
		}
	}
	else
	{
		std::stringstream msg;
    	msg << "Opening file '" << fileName 
			<< "' failed, it either doesn't exist or is not accessible.";
    	throw std::runtime_error(msg.str());
	}
	// Close the File
	f_stream.close();
 
	return dataList;
}