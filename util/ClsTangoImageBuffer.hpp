#ifndef CLSTANGOIMAGEBUFFER_HPP
#define CLSTANGOIMAGEBUFFER_HPP
#include <tango_client_api.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <turbojpeg.h>

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
		stride = data->stride;
		timestamp = data->timestamp;
		frame_number = data->frame_number;
		this->format = data->format;

		if(fmt == RGBA){
			bufsize = stride*height*4;
		} else if(fmt == YUV_NV21){
			bufsize = stride * ((height*3)/2);
		} else if(fmt == GRAY){
			bufsize = stride * height;
		}
		this->data = new uint8_t[bufsize];
		memcpy(this->data, data->data, bufsize);
	}
		
	~ClsTangoImageBuffer(){
		delete[] data;
	}
	static int write_to_initialization(std::string path, std::string id, std::string name, void * param){
		struct TangoCameraIntrinsics *calib = (struct TangoCameraIntrinsics*)param;
		std::ofstream fp;
		std::string filename = path + std::string("/");
		filename += id + std::string("_")+name+ std::string(".yml");

		LOGI("Writing initialization file %s", filename.c_str());
		fp.open(filename.c_str());
		fp << "image_width: "  << calib->width << "\n";
		fp << "image_height: "  << calib->height << "\n";
		fp << "camera_name: "  << id << "\n";
		fp << "camera_matrix:\n";
		fp << "rows: 3\n";
		fp << "cols: 3\n";
		fp << "data: [" << calib->fx << ", 0, " << calib->cx << ", 0, " << calib->fy << ", " << calib->cy <<", 0, 0, 1]\n";
		fp << "distortion_model: plumb_bob\n";
		fp << "distortion_coefficients:\n";
		fp << "rows: 1\n";
		fp << "cols: 5\n";
		fp << "data: [" << calib->distortion[0] << ", " << calib->distortion[1] << ", " << calib->distortion[2] << ", " << calib->distortion[3] << ", " << calib->distortion[4] << "]\n";
		fp << "camera_matrix:\n";
		fp << "rows: 3\n";
		fp << "cols: 3\n";
		fp << "data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n";
		fp << "projection_matrix:\n";
		fp << "rows: 3\n";
		fp << "cols: 4\n";
		fp << "data: [" << calib->fx << ", 0, " << calib->cx << ", 0, 0, " << calib->fy << ", " << calib->cy <<", 0, 0, 0, 1, 0]\n";
		fp.close();
		// TODO: Error Code
		return 0;
	}
	int write_to_file(std::string path, std::string id, std::string name){
		int bufsize;
		int cv_fmt, cv_height, cv_width, cv_conv;
		
		if(fmt == RGBA){
			cv_height = height;
			cv_width = stride;
			cv_fmt = CV_8UC4;
			cv_conv = CV_RGB2GRAY;
		} else if(fmt == YUV_NV21){
			cv_height = (height*3)/2;
			cv_width = stride;
			cv_fmt = CV_8UC1;
			cv_conv = CV_YUV2BGR_NV21;
		} else {// Default to gray
			cv_height = height;
			cv_width = stride;
			cv_fmt = CV_8UC1;
			cv_conv = CV_GRAY2BGR;
		}
		// TODO: Convert format from RGBA_8888 to BGRA_8888
		cv::Mat imageSRC(cv_height, cv_width, cv_fmt, data);
		cv::Mat imageBGR(height, stride, CV_8UC3, 0);

		cv::cvtColor(imageSRC, imageBGR, cv_conv);
		std::vector<uchar> bufOut;
        std::vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
        compression_params.push_back(10);

		std::string filename = path + std::string("/");
		filename += id + std::string("_")+name + std::string("_") +
			to_str(timestamp) + std::string(".jpg");
		//LOGI("Writing to file %s", filename.c_str());
		cv::Rect myROI(0, 0, width-1, height-1);
		cv::Mat imageBGRCrop(imageBGR, myROI);
		cv::imwrite(filename.c_str(), imageBGRCrop, compression_params);

		imageSRC.release();
		imageBGR.release();
		//imageBGRCrop.release();
		//TODO: Error Code
		return 0;
	}

	
};
#endif
;
