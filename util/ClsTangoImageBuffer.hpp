#ifndef CLSTANGOIMAGEBUFFER_HPP
#define CLSTANGOIMAGEBUFFER_HPP
#include <tango_client_api.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <jpeglib.h>

typedef enum e_format{
	RGBA=0,
	YUV_NV21,
	GRAY
}format_t;

#define MAX_THREADS 16
static bool initialized[MAX_THREADS];
static struct jpeg_compress_struct * _cinfo = new struct jpeg_compress_struct[MAX_THREADS];
static struct jpeg_error_mgr *jerr = new struct jpeg_error_mgr[MAX_THREADS];

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
	static int write_to_initialization(std::string path, std::string id, std::string name, void *param){
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
	int write_to_file(std::string path, std::string id, std::string name, int threadid){
		int bufsize;
		FILE *outfile;
        /* this is a pointer to one row of image data */
        JSAMPROW row_pointer[1];
        struct jpeg_compress_struct *cinfo = &_cinfo[threadid];
		cv::Mat imageSRC((height*3)/2, stride, CV_8UC1, data);
		cv::Mat imageBGR(height, stride, CV_8UC3, 0);
		cv::cvtColor(imageSRC, imageBGR, CV_YUV2BGR_NV21);

        if(!initialized[threadid]){
         	cinfo->err = jpeg_std_error(&jerr[threadid]);
            jpeg_create_compress(cinfo);

            /* Setting the parameters of the output file here */
            cinfo->image_width = width;
            cinfo->image_height = height;
            cinfo->input_components = 3;
            cinfo->in_color_space = JCS_RGB;

            /* default compression parameters, we shouldn't be worried about these */
            jpeg_set_defaults(cinfo);
            cinfo->num_components = 3;
            cinfo->dct_method = JDCT_FLOAT;
            jpeg_set_quality(cinfo, 50, TRUE);
            initialized[threadid] = true;
            LOGI("JPEG Initialization finished (%d)", threadid);
        }

		std::string filename = path + std::string("/");
		filename += id + std::string("_") + name;
		filename += std::string("_") + to_str(timestamp) + std::string(".jpg");
		//LOGI("Writing to file %s", filename.c_str());

        outfile = fopen( filename.c_str(), "wb" );
        /* Now do the compression .. */
        jpeg_stdio_dest(cinfo, outfile);
        jpeg_start_compress(cinfo, TRUE );
        /* like reading a file, this time write one row at a time */
        while( cinfo->next_scanline < cinfo->image_height )
        {
        	row_pointer[0] = &imageBGR.data[ cinfo->next_scanline * stride * cinfo->input_components];
        	jpeg_write_scanlines( cinfo, row_pointer, 1 );
        }
        /* similar to read file, clean up after we're done compressing */
        jpeg_finish_compress( cinfo );

        fclose( outfile );
        /* success code is 1! */

		imageSRC.release();
		imageBGR.release();
		//TODO: Error Code
		return 0;
	}
};

#endif

