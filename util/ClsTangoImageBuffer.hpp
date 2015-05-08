#ifndef CLSTANGOIMAGEBUFFER_HPP
#define CLSTANGOIMAGEBUFFER_HPP
#include <tango_client_api.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef enum e_format{
	RGBA=0,
	YUV_NV21,
	GRAY
}format_t;
class ClsTangoImageBuffer:public TangoImageBuffer{
	std::string to_str(double timestamp){
		std::ostringstream stm ;
		stm << timestamp ;
		return stm.str();
	}
public:
	format_t fmt;
	ClsTangoImageBuffer(const TangoImageBuffer* data, format_t format){
		int bufsize;
		fmt=format;
		width = data->width;
		height = data->height;
		if(fmt == RGBA){
			bufsize = width*height*4;
		} else if(fmt == YUV_NV21){
			bufsize = width * ((height*3)/2);
		} else if(fmt == GRAY){
			bufsize = width * height;
		}

		stride = data->stride;
		timestamp = data->timestamp;
		frame_number = data->frame_number;
		this->format = data->format;
		this->data = new uint8_t[bufsize];
		memcpy(this->data, data->data, bufsize);
	}
	
	~ClsTangoImageBuffer(){
		delete[] data;
	}

	int write_to_file(std::string path, std::string id, std::string name){
		int bufsize;
		int cv_fmt, cv_height, cv_width, cv_conv;
		
		if(fmt == RGBA){
			cv_height = height;
			cv_width = width;
			cv_fmt = CV_8UC4;
			cv_conv = CV_RGB2GRAY;
		} else if(fmt == YUV_NV21){
			cv_height = (height*3)/2;
			cv_width = width;
			cv_fmt = CV_8UC1;
			cv_conv = CV_YUV2BGR_NV21;
		} else {// Default to gray
			cv_height = height;
			cv_width = width;
			cv_fmt = CV_8UC1;
			cv_conv = CV_GRAY2BGR;
		}
		// TODO: Convert format from RGBA_8888 to BGRA_8888
		cv::Mat imageSRC(cv_height, cv_width, cv_fmt, data);
		cv::Mat imageBGR(height, width, CV_8UC3, 0);
		cv::cvtColor(imageSRC, imageBGR, cv_conv);

		std::string filename = path + std::string("/");
		filename += id + std::string("_")+name + std::string("_") + 
			to_str(timestamp) + std::string(".jpeg");
		//LOGI("Writing to file %s", filename.c_str());
		cv::imwrite(filename.c_str(), imageBGR);
		imageSRC.release();
		imageBGR.release();
		//TODO: Error Code
		return 0;
	}
private:
	
};
#endif
;
