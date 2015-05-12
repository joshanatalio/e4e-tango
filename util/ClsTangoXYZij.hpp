#ifndef CLSTANGOXYZIJ_HPP
#define CLSTANGOXYZIJ_HPP
#include <tango_client_api.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class ClsTangoXYZij:public TangoXYZij{
	std::string to_str(double timestamp){
		std::ostringstream stm ;
		stm << timestamp ;
		return stm.str();
	}
public:
	ClsTangoXYZij(const TangoXYZij* data){
		ij_rows = data->ij_rows;
		ij_cols = data->ij_cols;
		xyz_count = data->xyz_count;
		timestamp = data->timestamp;
		xyz = new float[xyz_count][3];
		ij = new uint32_t[ij_cols * ij_rows];
		memcpy(xyz, data->xyz,xyz_count * 3 * sizeof(float));
		memcpy(ij, data->ij,ij_cols * ij_rows*sizeof(uint32_t));
	}
	
	~ClsTangoXYZij(){
		delete[] xyz;
		delete[] ij;

		xyz = nullptr;
		ij = nullptr;
	}
	static int write_to_initialization(std::string path, std::string id, std::string name, void * param){

	}

	int write_to_file(std::string path, std::string id, std::string name){
		pcl::PointCloud<pcl::PointXYZ> output;
		std::string filename = path + std::string("/");
		filename += id + std::string("_")+name + std::string("_") + 
			to_str(timestamp) + std::string(".ply");
		for(int i = 0; i < xyz_count; i++){
			output.push_back(pcl::PointXYZ(xyz[i][0],xyz[i][1], xyz[i][2]));
		}
		pcl::io::savePLYFileBinary(filename.c_str(),output);
		return 0;
	}
private:
	
};
#endif
