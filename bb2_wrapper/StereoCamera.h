#ifndef _STEREO_CAMERA_H_
#define _STEREO_CAMERA_H_

// System Includes
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <iostream>
#include <fstream>

// PGR Includes
#include "triclops.h"  // The primary include file of the Triclops library.
#include "pgrflycapture.h"
#include "pgrflycapturestereo.h"
#include "pnmutils.h"  // Includes utilities reading and writing PGM and PPM format images.

// OpenCV Includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

//#define  GetRawOpencvImg 0

//////////////////////////////////////////////////////////////////////////
// Stereo camera class
//////////////////////////////////////////////////////////////////////////
class CstereoCamera{
//private:
protected:
	// Capture Image
	FlyCaptureContext		flycapture;
	FlyCaptureImage			flycaptureImage;
	FlyCaptureInfoEx		pInfo;
	FlyCapturePixelFormat   pixelFormat;
	FlyCaptureError			fe;

	// Deal with stereo data
	TriclopsInput       stereoData;
	TriclopsInput       colorData;
	TriclopsImage16     depthImage16;
	TriclopsImage       monoImage;
	TriclopsColorImage  colorImage;
	TriclopsContext     triclops;
	TriclopsError       te;

	int stereoWidth;
	int stereoHight;

	// Information extracted from the FlycaptureImage
	int imageCols;
	int imageRows;
	int imageRowInc;
	int iSideBySideImages;
	unsigned long timeStampSeconds;
	unsigned long timeStampMicroSeconds;

	char* szCalFile;

	FlyCaptureImage tempColorImage;
	FlyCaptureImage tempMonoImage;
	unsigned char* rowIntColor;
	unsigned char* rowIntMono;

	// image size
	int iMaxCols;
	int iMaxRows;

	// Camera status, if it is ready to capture image
	bool runStatus;

	// If stereo match is turned on
	bool stereoStatus;

	// OutputData, in OpenCV struct
#ifdef GetRawOpencvImg
	IplImage*	cvImgC;		// source color image data captured from stereo camera, include 2 image data
	IplImage*	cvImgCL;	// color image of left camera
	IplImage*	cvImgCR;	// color image of right camera
#endif

	IplImage*	cvRtfImgCL;  // color rectified image of left camera
	IplImage*	cvRtfImgCR;  // color rectified image of right camera
	
	IplImage*	cvImgStereo;  // stereo match result

public:
	CstereoCamera(int sw=320, int sh=240);
	~CstereoCamera();

	virtual bool StartCamera();		//
	bool StopCamera();

	bool AcquireFrame();	//acquire current frame from stereo camera, save the data into opencv images

	virtual bool StereoMatch(); 

	void SetStereoSize(int sw, int sh);

	inline void EnableStereoMatch(bool stereoOn = true)
	{
		stereoStatus = stereoOn;
	}

#ifdef GetRawOpencvImg
	inline IplImage* GetRawImg()
	{
		if(runStatus && cvImgC!=NULL) 
			return cvImgC;
		else
			return NULL;
	}

	inline IplImage* GetLeftImg()	
	{ 
		if(runStatus && cvImgCL!=NULL) 
			return cvImgCL;
		else
			return NULL;
	}
	inline IplImage* GetRightImg()	
	{
		if(runStatus && cvImgCR!=NULL)
			return cvImgCR; 
		else
			return NULL;
	}
#endif

	inline IplImage* GetStereoImg()
	{
		if(runStatus && cvImgStereo!=NULL)
			return cvImgStereo;
		else
			return NULL;
	}

	inline IplImage* GetRetfImgR()
	{
		if(runStatus && cvRtfImgCR!=NULL)
			return cvRtfImgCR;
		else
			return NULL;
	}

	inline IplImage* GetRetfImgL()
	{
		if(runStatus && cvRtfImgCL!=NULL)
			return cvRtfImgCL;
		else
			return NULL;
	}
	int Four2Three(IplImage *src, IplImage *dst);
};


//
// Macro to check, report on, and handle Triclops API error codes.
//
inline bool _HANDLE_TRICLOPS_ERROR( char* function,TriclopsError  error )
{ 
	if( error != TriclopsErrorOk )
	{ 
		printf( "ERROR: %s reported %s.\n", function, triclopsErrorToString( error ) ); 
		return false;
	}
	return true;
}

//
// Macro to check, report on, and handle Flycapture API error codes.
//
inline bool _HANDLE_FLYCAPTURE_ERROR( char* function, FlyCaptureError error )
{ 
	if( error != FLYCAPTURE_OK ) 
	{
		printf( "ERROR: %s reported %s.\n", function, flycaptureErrorToString( error ) ); 
		return false; 
	}
	return true;
}

#endif
