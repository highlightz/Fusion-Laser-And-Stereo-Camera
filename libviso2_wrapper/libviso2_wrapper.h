// To use this wrapper class, put all libviso2 .h and .cpp files in directory 'libviso2'.
// Refer to 'http://www.cvlibs.net/software/libviso/'

#ifndef LIBVISO2_WRAPPER_H
#define LIBVISO2_WRAPPER_H

// libviso2 Includes
#include "libviso2/viso_stereo.h"

// System Includes
#include <string>

// OpenCV Includes
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

class iplImageWrapper
{
public:	    
	iplImageWrapper( IplImage* RetfImgGrey )
	{  
		pic = cvCreateImage( cvSize( RetfImgGrey->width, RetfImgGrey->height ), RetfImgGrey->depth, RetfImgGrey->nChannels );
		cvCopy( RetfImgGrey, pic );
		width = pic->width;
		height = pic->height;
    }

	// For offline demo
	iplImageWrapper( std::string image_name )
	{
		const char* name = image_name.c_str( );
		pic = cvLoadImage( name, 0 );
		width = pic->width;
		height = pic->height;
	}

	~iplImageWrapper( )
	{
		if ( pic )
		{
			cvReleaseImage( &pic );
		}
	}

	int32_t get_width( )
	{
		return width;
	}

	int32_t get_height( )
	{
		return height;
	}

	uint8_t get_pixel( int32_t u, int32_t v )
	{
		return pic->imageData[ v * pic->widthStep + u ];
	}

public:
	int32_t width;
	int32_t height;
	
	IplImage* pic;
};

// This struct stores the necessary pose information of a gound vehicle.
struct odometry
{
	double x;  // axis of which points to the right
	double y;  // axis of which points to the earth
	double z;  // axis of which points to the front

	double yaw_rad;  // around y axis, positive when turning right horizontally
};

class libviso2_wrapper
{
public:
	libviso2_wrapper( VisualOdometryStereo::parameters aParam );
	
	// The principal driver for running libviso2
	void run( iplImageWrapper& left_img, iplImageWrapper& right_img );
	
	// For displaying odometry computating statistics
	double getInliers( ) const;
	
	// Return the x, y, z, yaw info to class user
	odometry getOdometry( ) const;

	// Return the [R, t] matrix to class user 
	Matrix getPose ( ) const;
	
	double computeDurationDistance( ) const;
	
	double distanceToWaypoint( const double x, const double y );
	
	void reinitializePose( );	
	
	void drawOdometryCurve( cv::Mat& bkground );
private:
	// Set most important visual odometry parameters,
	// for a full parameter list, look at: viso_stereo.h
	VisualOdometryStereo::parameters param;

	VisualOdometryStereo m_viso;

	// Current pose (this matrix transforms a point from the current
	// frame's camera coordinates to the first frame's camera coordinates)
	Matrix pose;

	double inliers;

	odometry odom;
	
private:
	void rotmat_to_euler( double R[9], double ola[3] );  // To be validated
};

#endif
