#include "hokuyo_wrapper.h"
#include "DirectionGenerator.h"

#include "bb2_wrapper.h"
#include "libviso2_wrapper.h"

int main( int argc, char** argv )
{
	// Initialize hokuyo_wrapper object, and start the laser range finder
	hokuyo_wrapper laser( argc, argv );
	laser.startHokuyo( );
    	laser.setInterestRadius( 10000 );  // 10 meters
	
	// Initialize DirectionGenerator object    
    	DirectionGenerator dg;
	dg.setInterestRadius( 4500 );  // 4.5 meters
    
    	cv::Mat laserPoints;
    
    	// Initialize bb2_wrapper object, and start the stereo camera
    	const int WIDTH = 640;
    	const int HEIGHT = 480;
	bb2_wrapper m_camera( WIDTH, HEIGHT );
	IplImage* pframeL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 4 );
	IplImage* pframeR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 4 );
	IplImage* pfL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 3 );
	IplImage* pfR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 3 );
	IplImage* pGrayL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 1 );
	IplImage* pGrayR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 1 );
	
	// Start camera
	if ( !m_camera.StartCamera( ) )
	{
		std::cout << "StartCamera failed!" << std::endl;
		return -1;
	}
	
	// Initialize libviso2_wrapper object
	// Calibration parameters for bb2 
	VisualOdometryStereo::parameters param;
	double f  = 278.69; 
	double cu = 317.00; 
	double cv = 246.89; 
	double parambase = 0.12;
	
	param.calib.f  = f;  // focal length in pixels
	param.calib.cu = cu;  // principal point (u-coordinate) in pixels
	param.calib.cv = cv;  // principal point (v-coordinate) in pixels
	param.base     = parambase;  // baseline in meters
	
	// Draw odometry curve on this image
	cv::Mat odomCurve( 640, 480, CV_8UC3 );
	
	libviso2_wrapper libviso2( param );
	
	// Main processing loop
	while ( 1 )
	{
		// Acquire laser sequence data
		laser.bufferDistance( );
        	vector< long > distance = laser.getDistance( );
        
        	// Generate next waypoint
        	double x_next = 0.0;
        	double y_next = 0.0;
        	dg.genWaypoint( distance, x_next, y_next );
        	
        	const double toleranceThreshold = 0.2;  // 0.2 meter, wild value, to be tuned
        	double distanceToWaypoint = libviso2.distanceToWaypoint( x_next, y_next );
        	while ( distanceToWaypoint > toleranceThreshold )
        	{
        		// TODO:
        		// Here, drive the car towards the target
        		
        		// Start visual odometry, and init its coordinate frame;
        		// meanwhile, map the generated waypoint into odometry frame
        		if ( m_camera.AcquireFrame( ) && m_camera.StereoMatch( ) )
			{
				pframeL = m_camera.GetRetfImgL( );
				pframeR = m_camera.GetRetfImgR( );
				m_camera.Four2Three( pframeL, pfL );
				m_camera.Four2Three( pframeR, pfR );
	
				cvCvtColor( pfL, pGrayL, CV_BGR2GRAY );
				cvCvtColor( pfR, pGrayR, CV_BGR2GRAY );
			}
			
			iplImageWrapper left_image( pGrayL );
			iplImageWrapper right_image( pGrayR );
			
			libviso2.run( left_image, right_image );
			
			libviso2.drawOdometryCurve( odomCurve );
			cv::imshow( "Odometry", odomCurve );
			cv::waitKey( 5 );
			
			// Update the distance to the next target 
			distanceToWaypoint = libviso2.distanceToWaypoint( x_next, y_next );
        	}
        	
        	// When the target has been reached, 
        	// the odometry coordinate should be reinitialized
        	libviso2.reinitializePose( );
	}

	return 0;
}


