#ifndef BB2_WRAPPER_H
#define BB2_WRAPPER_H

#include "StereoCamera.h"
#include <fstream>
#include <vector>

using std::ofstream;
using std::vector;
using cv::Mat;

// The point cloud refers to camera as a reference coordinate frame.
struct PointCloud
{
	float x, y, z;
	int r, g, b;
	int row, col;
};

class bb2_wrapper : public CstereoCamera
{
public:
	bb2_wrapper( int sw = 320, int sh = 240 );
	~bb2_wrapper( );

	void setDispRange( int minDisp, int maxDisp );

	bool StartCamera( );

	// Show camera info
	void showCameraInfo( );

	Mat getDepthImage16( ) const;

	// Only for display
	IplImage* getStereo( ) const;

	PointCloud getPointCloud( ) const;

	bool StereoMatch( ); 

	void showInterestPointsDepth( ) const;

	void showInterestPoints3D( ) const;

	void showAvrgDepth( );

	vector< CvPoint3D32f >  showInterestArea( ) const;
private:
	int minDisparity;
	int maxDisparity;
	PointCloud pc;
	
	vector< float > interestPointsDepth;
	void fillInterestPointsDepth( ); 

	vector < CvPoint3D32f > interestPoints3D;
	void fillInterestPoints3D( );

	vector< float > avrgDepth;
	void fillAvrgDepth( );

	vector< CvPoint3D32f > interestArea;
	void fillInterestArea( );
};

#endif
