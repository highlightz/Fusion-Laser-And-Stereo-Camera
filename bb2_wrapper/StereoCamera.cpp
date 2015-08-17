#include "StereoCamera.h"

CstereoCamera::CstereoCamera(int sw, int sh)
{
	iMaxRows = 0;
	iMaxCols = 0;

	runStatus = false;

	stereoStatus = false;
	
	rowIntColor = NULL;
	rowIntMono = NULL;

#ifdef GetRawOpencvImg
	cvImgC = NULL;
	cvImgCL = NULL;
	cvImgCR = NULL;
#endif

	cvRtfImgCL = NULL;
	cvRtfImgCR = NULL;

	stereoWidth = sw;
	stereoHight	= sh;

	cvImgStereo = NULL;
}

CstereoCamera::~CstereoCamera()
{
	if(rowIntColor!=NULL)
	{
		delete[] rowIntColor;
		rowIntColor=NULL;
	}
	if(rowIntMono!=NULL)
	{
		delete[] rowIntMono;
		rowIntMono=NULL;
	}

#ifdef GetRawOpencvImg
	if (cvImgC!=NULL)
		cvReleaseImageHeader(&cvImgC);
	if(cvImgCL!=NULL)
		cvReleaseImage(&cvImgCL);
	if(cvImgCR!=NULL)
		cvReleaseImage(&cvImgCR);
#endif

	if(cvImgStereo!=NULL)
		cvReleaseImage(&cvImgStereo);

	if(cvRtfImgCL!=NULL)
		cvReleaseImageHeader(&cvRtfImgCL);
	if(cvRtfImgCR!=NULL)
		cvReleaseImageHeader(&cvRtfImgCR);
}

bool CstereoCamera::StartCamera()
{
	if(runStatus)
		return true;

	// Open the camera
	fe = flycaptureCreateContext( &flycapture );
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycaptureCreateContext()", fe ) ) 
		return false;

	// Initialize the Flycapture context
	fe = flycaptureInitialize( flycapture, 0 );
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycaptureInitialize()", fe ) )
		return false;

	// Save the camera's calibration file, and return the path 
	fe = flycaptureGetCalibrationFileFromCamera( flycapture, &szCalFile );
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycaptureGetCalibrationFileFromCamera()", fe ) )
		return false;

	// Create a Triclops context from the cameras calibration file
	te = triclopsGetDefaultContextFromFile( &triclops, szCalFile );
	if( ! _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", te ) )
		return false;

	// Get camera information
	fe = flycaptureGetCameraInfo( flycapture, &pInfo );
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycatpureGetCameraInfo()", fe ) ) 
		return false;

	if (pInfo.CameraType == FLYCAPTURE_COLOR)
		pixelFormat = FLYCAPTURE_RAW16;	 
	else 
		pixelFormat = FLYCAPTURE_MONO16;
	
	switch (pInfo.CameraModel)
	{
	case FLYCAPTURE_BUMBLEBEE2:
		unsigned long ulValue;
		flycaptureGetCameraRegister( flycapture, 0x1F28, &ulValue );

		if ( ( ulValue & 0x2 ) == 0 )
		{// Hi-res BB2
			iMaxCols = 1024; 
			iMaxRows = 768;   
		}
		else
		{// Low-res BB2
			iMaxCols = 640;
			iMaxRows = 480;
		}
		break;

	case FLYCAPTURE_BUMBLEBEEXB3:
		iMaxCols = 1280;
		iMaxRows = 960;
		break;

	default:
		te = TriclopsErrorInvalidCamera;
		if( ! _HANDLE_TRICLOPS_ERROR( "triclopsCheckCameraModel()", te ) )
			return false;
		break;
	}

	// Start transferring images from the camera to the computer
	fe = flycaptureStartCustomImage( 
		flycapture, 3, 0, 0, iMaxCols, iMaxRows, 100, pixelFormat);
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycaptureStart()", fe ) )
		return false;

	// Set up some stereo parameters:
	// Set to 320x240 output images
	te = triclopsSetResolution( triclops, stereoHight, stereoWidth );
	if( ! _HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", te ) )
		return false;

	if(cvRtfImgCR!=NULL && (cvRtfImgCR->width!=stereoWidth || cvRtfImgCR->height!=stereoHight) )
	{
		cvReleaseImageHeader(&cvRtfImgCR);
		cvRtfImgCR = NULL;
	}
	if(cvRtfImgCR==NULL)	cvRtfImgCR = cvCreateImageHeader(cvSize(stereoWidth,stereoHight),8,4);

	if(cvRtfImgCL!=NULL && (cvRtfImgCL->width!=stereoWidth || cvRtfImgCL->height!=stereoHight) )
	{
		cvReleaseImageHeader(&cvRtfImgCL);
		cvRtfImgCL = NULL;
	}
	if(cvRtfImgCL==NULL)	cvRtfImgCL = cvCreateImageHeader(cvSize(stereoWidth,stereoHight),8,4);

	if(cvImgStereo!=NULL && (cvImgStereo->width!=stereoWidth||cvImgStereo->height!=stereoHight))
	{
		cvReleaseImage(&cvImgStereo);
		cvImgStereo = NULL;
	}

	if(cvImgStereo==NULL)	
	{
		cvImgStereo = cvCreateImage(cvSize(stereoWidth,stereoHight),8,1);
		cvSetZero(cvImgStereo);
	}
	//cvNot(cvImgStereo, cvImgStereo);

	// Set disparity range
	te = triclopsSetDisparity( triclops, 0, 100 );
	if( ! _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", te ) )
		return false;  

	// Lets turn off all validation except subpixel and surface
	// This works quite well
	te = triclopsSetTextureValidation( triclops, 0 );
	if( ! _HANDLE_TRICLOPS_ERROR( "triclopsSetTextureValidation()", te ) )
		return false;
	te = triclopsSetUniquenessValidation( triclops, 0 );
	if( ! _HANDLE_TRICLOPS_ERROR( "triclopsSetUniquenessValidation()", te ) )
		return false;

	// Turn on sub-pixel interpolation
	te = triclopsSetSubpixelInterpolation( triclops, 1 );
	if( ! _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te ) )
		return false;

	// Grab an image from the camera
	fe = flycaptureGrabImage2( flycapture, &flycaptureImage );
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycaptureGrabImage()", fe ) )
		return false;

	// Extract information from the FlycaptureImage
	imageCols = flycaptureImage.iCols;
	imageRows = flycaptureImage.iRows;
	imageRowInc = flycaptureImage.iRowInc;
	iSideBySideImages = flycaptureImage.iNumImages;
	timeStampSeconds = flycaptureImage.timeStamp.ulSeconds;
	timeStampMicroSeconds = flycaptureImage.timeStamp.ulMicroSeconds;

	if(rowIntColor!=NULL)
	{
		delete[] rowIntColor;
		rowIntColor=NULL;
	}
	if(rowIntMono!=NULL)
	{
		delete[] rowIntMono;
		rowIntMono=NULL;
	}

	rowIntColor = new unsigned char[ imageCols * imageRows * iSideBySideImages * 4 ];
	rowIntMono = new unsigned char[ imageCols * imageRows * iSideBySideImages ];

	tempColorImage.pData = rowIntColor;
	tempMonoImage.pData = rowIntMono;

	//opencv
#ifdef GetRawOpencvImg
	if(cvImgCL!=NULL && (cvImgCL->width!=imageCols || cvImgCL->height!=imageRows) )
	{
		cvReleaseImage(&cvImgCL);
		cvImgCL = NULL;
	}
	if(cvImgCR!=NULL && (cvImgCR->width!=imageCols || cvImgCR->height!=imageRows) )
	{
		cvReleaseImage(&cvImgCR);
		cvImgCR = NULL;
	}
	if(cvImgC!=NULL && (cvImgC->width!=imageCols*iSideBySideImages || cvImgC->height!=imageRows) )
	{
		cvReleaseImageHeader(&cvImgC);
		cvImgC = NULL;
	}

	if(cvImgCL==NULL)	cvImgCL = cvCreateImage(cvSize(imageCols,imageRows),8,4);
	if(cvImgCR==NULL)	cvImgCR = cvCreateImage(cvSize(imageCols,imageRows),8,4);
	if(cvImgC==NULL)	cvImgC = cvCreateImageHeader(cvSize(imageCols*iSideBySideImages,imageRows),8,4);
#endif

	runStatus = true;
	return true;
}

bool CstereoCamera::StopCamera()
{
	if(!runStatus)
		return true;

	runStatus = false;

	// Close the camera
	fe = flycaptureStop( flycapture );
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycaptureStop()", fe ) )
		return false;

	fe = flycaptureDestroyContext( flycapture );
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycaptureDestroyContext()", fe ) )
		return false;

	// Destroy the Triclops context
	te = triclopsDestroyContext( triclops ) ;
	if( ! _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", te ) )
		return false;

	if(rowIntColor!=NULL)
	{
		delete[] rowIntColor;
		rowIntColor=NULL;
	}
	if(rowIntMono!=NULL)
	{
		delete[] rowIntMono;
		rowIntMono=NULL;
	}

	return true;
}

bool CstereoCamera::AcquireFrame()
{
	if (runStatus)
	{
		// Grab an image from the camera
		fe = flycaptureGrabImage2( flycapture, &flycaptureImage );
		if( ! _HANDLE_FLYCAPTURE_ERROR( "flycaptureGrabImage()", fe ) )
			return false;

		timeStampSeconds = flycaptureImage.timeStamp.ulSeconds;
		timeStampMicroSeconds = flycaptureImage.timeStamp.ulMicroSeconds;

		// Convert the pixel interleaved raw data to row interleaved format
		fe = flycapturePrepareStereoImage( flycapture, flycaptureImage, &tempMonoImage, &tempColorImage  );
		if( ! _HANDLE_FLYCAPTURE_ERROR( "flycapturePrepareStereoImage()", fe ) )
			return false;

#ifdef GetRawOpencvImg
		cvImgC->imageData = (char*)rowIntColor;

		// If cvImgCL is 4-channel one, ROI in OpenCV is also available
		cvSetImageROI(cvImgC,cvRect(0,0,imageCols,imageRows));
		cvCopy(cvImgC,cvImgCR);
		cvResetImageROI(cvImgC);

		cvSetImageROI(cvImgC,cvRect(imageCols,0,imageCols,imageRows));
		cvCopy(cvImgC,cvImgCL);
		cvResetImageROI(cvImgC);
#endif

		return true;
	}
	
	return false;
}
int CstereoCamera::Four2Three(IplImage *src, IplImage *dst)
{
	IplImage *pfr,*pfg,*pfb,*pfa,*dstt;
	pfr = cvCreateImage(cvGetSize(src),src->depth,1);
	pfg = cvCreateImage(cvGetSize(src),src->depth,1);
	pfb = cvCreateImage(cvGetSize(src),src->depth,1);
	pfa = cvCreateImage(cvGetSize(src),src->depth,1);
	dstt = cvCreateImage(cvGetSize(src),src->depth,3);
	cvSplit(src,pfr,pfg,pfb,pfa);
	cvMerge(pfr,pfg,pfb,NULL,dstt);
	cvResize(dstt,dst);
	cvReleaseImage(&pfr);
	cvReleaseImage(&pfg);
	cvReleaseImage(&pfb);
	cvReleaseImage(&pfa);
	cvReleaseImage(&dstt);
	return true;
}
bool CstereoCamera::StereoMatch()
{
	if(runStatus)
	{		
		// Pointers to positions in the color buffer that correspond to the beginning
		// of the red, green and blue sections
		unsigned char* redColor = NULL;
		unsigned char* greenColor = NULL;
		unsigned char* blueColor = NULL; 

		redColor = rowIntColor;
		if (flycaptureImage.iNumImages == 2)
		{
			greenColor = redColor + ( 4 * imageCols );
			blueColor = redColor + ( 4 * imageCols );
		}

		if (flycaptureImage.iNumImages == 3)
		{
			greenColor = redColor + ( 4 * imageCols );
			blueColor = redColor + ( 2 * 4 * imageCols );
		}

		// Use the row interleaved images to build up a packed TriclopsInput.
		// A packed triclops input will contain a single image with 32 bpp.
		te = triclopsBuildPackedTriclopsInput( imageCols, imageRows, imageRowInc * 4,
			timeStampSeconds, timeStampMicroSeconds, greenColor, &colorData );
		_HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );

		// rectify the color image
		TriclopsPackedColorImage  RcImage;
		te = triclopsRectifyPackedColorImage( triclops, TriCam_LEFT, 
			&colorData, &RcImage );
		_HANDLE_TRICLOPS_ERROR( "triclopsRectifyPackedColorImage()", te );

		cvRtfImgCL->imageData = (char*) RcImage.data;

		te = triclopsBuildPackedTriclopsInput( imageCols, imageRows, imageRowInc * 4,
			timeStampSeconds, timeStampMicroSeconds, redColor, &colorData );
		_HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );
		te = triclopsRectifyPackedColorImage( triclops, TriCam_RIGHT, 
			&colorData, &RcImage );
		_HANDLE_TRICLOPS_ERROR( "triclopsRectifyPackedColorImage()", te );

		cvRtfImgCR->imageData = (char*) RcImage.data;
		
		if(stereoStatus)
		{
			float	       x, y, z; 
			int		       r, g, b;	 
			int		       nPoints = 0;		// Successfully matched number of points
			int		       pixelinc ;
			int		       i, j, k;
			unsigned short*     row;
			unsigned short      disparity;

			// Pointers to positions in the mono buffer that correspond to the beginning
			// of the red, green and blue sections
			unsigned char* redMono = NULL;
			unsigned char* greenMono = NULL;
			unsigned char* blueMono = NULL; 

			redMono = rowIntMono;
			if (flycaptureImage.iNumImages == 2)
			{
				greenMono = redMono + imageCols;
				blueMono = redMono + imageCols;
			}

			if (flycaptureImage.iNumImages == 3)
			{
				greenMono = redMono + imageCols;
				blueMono = redMono + ( 2 * imageCols );
			}

			// Use the row interleaved images to build up an RGB TriclopsInput.  
			// An RGB triclops input will contain the 3 raw images (1 from each camera).
			te = triclopsBuildRGBTriclopsInput( imageCols, imageRows, imageRowInc, timeStampSeconds, 
				timeStampMicroSeconds, redMono, greenMono, blueMono, &stereoData);
			_HANDLE_TRICLOPS_ERROR( "triclopsBuildRGBTriclopsInput()", te );

			// Preprocessing the images
			te = triclopsRectify( triclops, &stereoData );
			_HANDLE_TRICLOPS_ERROR( "triclopsRectify()", te );

			// Stereo processing
			te = triclopsStereo( triclops ) ;
			_HANDLE_TRICLOPS_ERROR( "triclopsStereo()", te );

			// Retrieve the interpolated depth image from the context
			te = triclopsGetImage16( triclops, TriImg16_DISPARITY, TriCam_REFERENCE, &depthImage16 );
			_HANDLE_TRICLOPS_ERROR( "triclopsGetImage16()", te );

			// Rectify the color image if applicable
			if ( pixelFormat == FLYCAPTURE_RAW16 )
			{
				te = triclopsRectifyColorImage( triclops, TriCam_REFERENCE, &colorData, &colorImage );
				_HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", te );
			}
			else
			{
				te = triclopsGetImage( triclops, TriImg_RECTIFIED, TriCam_REFERENCE, &monoImage );
				_HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );
			}

			// Determine the number of pixels spacing per row
			pixelinc = depthImage16.rowinc/2;
			for ( i = 0, k = 0; i < depthImage16.nrows; i++ )
			{
				row     = depthImage16.data + i * pixelinc;
				for ( j = 0; j < depthImage16.ncols; j++, k++ )
				{
					disparity = row[j];

					// do not save invalid points
					if ( disparity < 0xFF00 )
					{
						// convert the 16 bit disparity value to floating point x,y,z
						triclopsRCD16ToXYZ( triclops, i, j, disparity, &x, &y, &z );

						// look at points within a range
						if ( z < 5.0 )
						{
							if ( pixelFormat == FLYCAPTURE_RAW16 )
							{
								r = (int)colorImage.red[k];
								g = (int)colorImage.green[k];
								b = (int)colorImage.blue[k];		  
							}
							else
							{
								// For mono cameras, we 1just assign the same value to RGB
								r = (int)monoImage.data[k];
								g = (int)monoImage.data[k];
								b = (int)monoImage.data[k];
							}

						//	fprintf( pointFile, "%f %f %f %d %d %d %d %d\n", x, y, z, r, g, b, i, j );						
							cvImgStereo->imageData[i*cvImgStereo->widthStep+j] = (int) (5-z)*255/5;
							nPoints++;
						}
						else
						{
							cvImgStereo->imageData[i*cvImgStereo->widthStep+j] = 0;
						}
					}
					else
					{
						cvImgStereo->imageData[i*cvImgStereo->widthStep+j] = 0;
					}
				}
			}

			printf( "Points count: %d\n", nPoints );

			redMono = NULL;
			greenMono = NULL;
			blueMono = NULL;
		}
		
		// Delete the image buffer, it is not needed once the TriclopsInput
		// has been built		
		redColor = NULL;
		greenColor = NULL;
		blueColor = NULL;

		return true;
		
	}
	return false;
}

void CstereoCamera::SetStereoSize(int sw, int sh)
{
	if (!runStatus)
	{
		stereoWidth = sw;
		stereoHight = sh;
	}
	else
	{
		StopCamera();
		stereoWidth = sw;
		stereoHight = sh;
		StartCamera();
	}
}
