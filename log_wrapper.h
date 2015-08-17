// log_wrapper.h - a class template that records datas from the vehicle system
// Note: template parameter DataType is specified by the class template user who
// wants to log odometry data as a struct, for example, the odometry struct can 
// be defined as:
// struct odometry
// {
//   double x;  // axis of which points to the right
//	 double y;  // axis of which points to the earth
//	 double z;  // axis of which points to the front
//
//	 double yaw_rad;  // around y axis, positive when turning right horizontally
// };
// Usage: 
// Set an initial counter before entering the image generating loop,
// and then update the same counter in the end of the loop. For example,
// -----------------------------------------------------------------------//
// long img_counter = 0;                                                  //
// while ( 1 )															  //
// {																	  //
//   local_log_wrapper.start_log_image_pair( img_counter, imgL, imgR );	  //
//   img_counter++;														  //
// }																	  //
// -----------------------------------------------------------------------//

#ifndef LOG_WRAPPER_H
#define LOG_WRAPPER_H 

#include <sstream>
#include <string>
#include <fstream>
using std::stringstream;
using std::string;
using std::ofstream;

#include <opencv/highgui.h>
using cv::Mat;

template < class DataType >
class log_wrapper
{
public:
	// Image loggers
	log_wrapper( string image_dir );
	log_wrapper( string image_dir, string filename );
	
	void start_log_image_pair( long image_cntr, IplImage* pImgL, IplImage* pImgR );
	void start_log_image_pair( long image_cntr, IplImage* pImgL, IplImage* pImgR, Mat disp16 );

	// Odometry loggers
	void start_log_odom( DataType odom  );

	// Laser range loggers

private:
	// Image loggers
	string base_name_L;
	string base_name_R;
	string base_name_disp;

	long image_counter;
	stringstream ss;
	string image_counter_str;

	string image_name_L;
	string image_name_R;
	string disp16_name;

	// Odometry loggers
	ofstream odomFile;

	// Laser range loggers
	
};

#include "log_wrapper.cpp"
#endif
