#ifndef CLSTANGOPOSEDATA_HPP
#define CLSTANGOPOSEDATA_HPP
#include <tango_client_api.h>

#include <sstream>
#include <cstdlib>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

class ClsTangoPoseData: public TangoPoseData{

public:
	std::string to_str(double timestamp){
		std::ostringstream stm ;
		stm << timestamp ;
		return stm.str();
	}
	ClsTangoPoseData(const TangoPoseData* data){
		orientation[0] = data->orientation[0];
		orientation[1] = data->orientation[1];
		orientation[2] = data->orientation[2];
		orientation[3] = data->orientation[3];
		translation[0] = data->translation[0];
		translation[1] = data->translation[1];
		translation[2] = data->translation[2];
		status_code = data->status_code;
		frame = data->frame;
		timestamp = data->timestamp;
	}
	
	~ClsTangoPoseData(){
	}

	int write_to_file(std::string path, std::string id, std::string name){
		std::ofstream fp;
		std::string filename = path + std::string("/");
		filename += id + std::string("_")+name+ std::string(".csv");
		//LOGI("Writing to file %s", filename.c_str());
		fp.open(filename.c_str(), std::ofstream::app); // TODO: +W?
		fp << timestamp << ",";
		fp << orientation[0] << "," << orientation[1] << ",";
		fp << orientation[2] << "," << orientation[3] << ",";
		fp << translation[0] << "," << translation[1] << "," << translation[2] << "\n";
		fp.close();
		// TODO: Error Code
		return 0;
	}

private:
	
};
#endif
