#include <iostream>
using std::endl;

template < class DataType >
log_wrapper< DataType >::log_wrapper( string stereo_dir ) : odomFile( string( stereo_dir + "online_odom.txt" ) )
{
	// Default directory for stereo image pairs
	base_name_L = stereo_dir + "L_";
	base_name_R = stereo_dir + "R_";
	base_name_disp = stereo_dir + "disp_";
	
	image_counter = 0;
	ss << image_counter;
	ss >> image_counter_str;

	image_name_L = base_name_L + image_counter_str + ".jpg";
	image_name_R = base_name_R + image_counter_str + ".jpg";
	disp16_name = base_name_disp + image_counter_str + ".jpg";
}

template < class DataType >
log_wrapper< DataType >::log_wrapper( string stereo_dir, string filename ) : odomFile( stereo_dir + filename )
{
	// Default directory for stereo image pairs
	base_name_L = stereo_dir + "L_";
	base_name_R = stereo_dir + "R_";
	base_name_disp = stereo_dir + "disp_";
	
	image_counter = 0;
	ss << image_counter;
	ss >> image_counter_str;

	image_name_L = base_name_L + image_counter_str + ".jpg";
	image_name_R = base_name_R + image_counter_str + ".jpg";
	disp16_name = base_name_disp + image_counter_str + ".jpg";
}

template < class DataType >
void log_wrapper< DataType >::start_log_image_pair( long image_cntr, IplImage* pImgL, IplImage* pImgR )
{
	image_counter = image_cntr;

	ss << image_counter;
	ss >> image_counter_str;
	
	image_name_L = base_name_L + image_counter_str + ".jpg";
	image_name_R = base_name_R + image_counter_str + ".jpg";
	
	// Save the image pair
	cvSaveImage( image_name_L.c_str( ), pImgL );
	cvSaveImage( image_name_R.c_str( ), pImgR );

	ss.clear( );
}

template < class DataType >
void log_wrapper< DataType >::start_log_image_pair( long stereo_cntr, IplImage* pImgL, IplImage* pImgR, Mat disp16 )
{
	image_counter = stereo_cntr;

	ss << image_counter;
	ss >> image_counter_str;

	image_name_L = base_name_L + image_counter_str + ".jpg";
	image_name_R = base_name_R + image_counter_str + ".jpg";
	disp16_name = base_name_disp + image_counter_str + ".jpg";

	// Save the image pair
	cvSaveImage( image_name_L.c_str( ), pImgL );
	cvSaveImage( image_name_R.c_str( ), pImgR );

	// Save the disparity map of 16 bit
	imwrite( disp16_name, disp16 );

	ss.clear( );
}

template < class DataType >
void log_wrapper< DataType >::start_log_odom( DataType odom )
{
	odomFile << odom.x << " " 
		     << odom.y << " " 
			 << odom.z << " "
			 << odom.yaw_rad << endl;
}
