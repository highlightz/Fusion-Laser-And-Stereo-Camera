#include "bb2_wrapper.h"

#include <iostream>
using std::cout;
using std::endl;
using std::fixed;
using std::setprecision;

bb2_wrapper::bb2_wrapper( int sw, int sh ) : CstereoCamera( sw, sh ), interestPointsDepth( 15 ), interestPoints3D( 15 ), avrgDepth( 3 ), interestArea( 2500 )
{
	minDisparity = 0;
	maxDisparity = 64;
}

bb2_wrapper::~bb2_wrapper( )
{
}

void bb2_wrapper::setDispRange( int minDisp, int maxDisp )
{
	minDisparity = minDisp;
	maxDisparity = maxDisp;
}

bool bb2_wrapper::StartCamera( )
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
	te = triclopsSetDisparity( triclops, minDisparity, maxDisparity );
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

void bb2_wrapper::showCameraInfo( )
{
	// Print the camera info to the screen for the user to validate:

	cout << fixed << setprecision( 2 );

	float baseline;
	te = triclopsGetBaseline( triclops, &baseline );
	_HANDLE_TRICLOPS_ERROR( "triclopsGetBaseline()", te );
	cout << "Baseline: " << baseline << " meters" << endl;

	float focalLength;
	te = triclopsGetFocalLength( triclops, &focalLength );
	_HANDLE_TRICLOPS_ERROR( "triclopsGetFocalLength()", te );
	cout << "Focal length: " << focalLength << " pixels" << endl;

	float centerRow;
	float centerCol;
	te = triclopsGetImageCenter( triclops, &centerRow, &centerCol );
	_HANDLE_TRICLOPS_ERROR( "triclopsGetImageCenter()", te );
	cout << "The center (along y direction): " << centerRow << " pixels" << endl;
	cout << "The center (along x direction): " << centerCol << " pixels" << endl;

	cout << "The CalFile: " << szCalFile << endl;
}

bool bb2_wrapper::StereoMatch( )
{
	if(runStatus)
	{		
		// Pointers to positions in the color buffer that correspond to the beginning
		// of the red, green and blue sections
		unsigned char* redColor = NULL;
		unsigned char* greenColor = NULL;
		unsigned char* blueColor = NULL; 

		redColor = rowIntColor;
		if ( flycaptureImage.iNumImages == 2 )
		{
			greenColor = redColor + ( 4 * imageCols );
			blueColor = redColor + ( 4 * imageCols );
		}

		if ( flycaptureImage.iNumImages == 3 )
		{
			greenColor = redColor + ( 4 * imageCols );
			blueColor = redColor + ( 2 * 4 * imageCols );
		}

		// Use the row interleaved images to build up a packed TriclopsInput.
		// A packed triclops input will contain a single image with 32 bpp.
		te = triclopsBuildPackedTriclopsInput( imageCols, imageRows, imageRowInc * 4,
			timeStampSeconds, timeStampMicroSeconds, greenColor, &colorData );
		_HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );

		// Rectify the color image
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
			int		       r, g, b;			// 彩色图像对应像素点值，校正后的图像
			int		       nPoints = 0;		// 成功匹配的点数
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
							// rgb things
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

							// Put the current pixel info into a PointCloud struct
							pc.x = x; pc.y = y; pc.z = z; pc.r = r; pc.g = g; pc.b = b; pc.row = i; pc.col = j;

							// If the current Point has the right indexes, then fill an interest point with this point info.
							fillInterestPointsDepth( );
							fillInterestPoints3D( );
							fillAvrgDepth( );
							//fillInterestArea( );

							// I guess this is only a qualitative representation of the relationship 
							// between depth and disparity, therefore cannot be directly used for 
							// calculating z.
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
			
			//printf( "Points count: %d\n", nPoints );
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

Mat bb2_wrapper::getDepthImage16( ) const
{
	Mat result( depthImage16.nrows, depthImage16.ncols, CV_16UC1 );

	for ( int r = 0; r < depthImage16.nrows; r++ )
	{
		unsigned short* rowPointer = depthImage16.data + r*depthImage16.rowinc/sizeof(unsigned short);
		for ( int c = 0; c < depthImage16.ncols; c++ )
		{
			result.at< ushort >( r, c ) = rowPointer[c];
		}
	}

	return result;
}

IplImage* bb2_wrapper::getStereo( ) const
{
	return cvImgStereo;
}

PointCloud bb2_wrapper::getPointCloud( ) const
{
	return pc;
}

void bb2_wrapper::fillInterestPointsDepth( )
{
	// First line
	if ( pc.col == stereoWidth / 4 - stereoWidth / 5 && pc.row == stereoHight / 4 )
	{
		interestPointsDepth[0] = pc.z;
	}

	if ( pc.col == stereoWidth / 4 && pc.row == stereoHight / 4 )
	{
		interestPointsDepth[1] = pc.z;
	}
	if ( pc.col == stereoWidth / 2 && pc.row == stereoHight / 4 )
	{
		interestPointsDepth[2] = pc.z;
	}
	if ( pc.col == ( stereoWidth - stereoWidth / 4 ) && pc.row == stereoHight / 4 )
	{
		interestPointsDepth[3] = pc.z;
	}

	if ( pc.col == stereoWidth - stereoWidth / 4 + stereoWidth / 5 && pc.row == stereoHight / 4 )
	{
		interestPointsDepth[4] = pc.z;
	}

	// Second line
	if ( pc.col == stereoWidth / 4 - stereoWidth / 5 && pc.row == stereoHight / 2 )
	{
		interestPointsDepth[5] = pc.z;
	}
	if ( pc.col == stereoWidth / 4 && pc.row == stereoHight / 2 )
	{
		interestPointsDepth[6] = pc.z;
	}
	if ( pc.col == ( stereoWidth / 2 ) && pc.row == stereoHight / 2 )
	{
		interestPointsDepth[7] = pc.z;
	}

	if ( pc.col == stereoWidth - stereoWidth / 4 && pc.row == stereoHight / 2 )
	{
		interestPointsDepth[8] = pc.z;
	}
	if ( pc.col == stereoWidth - stereoWidth / 4 + stereoWidth / 5 && pc.row == stereoHight / 2 )
	{
		interestPointsDepth[9] = pc.z;
	}

	// Third line
	if ( pc.col == stereoWidth / 4 - stereoWidth / 5 && pc.row == stereoHight - stereoHight / 4 )
	{
		interestPointsDepth[10] = pc.z;
	}
	if ( pc.col == stereoWidth / 4 && pc.row == stereoHight - stereoHight / 4 )
	{
		interestPointsDepth[11] = pc.z;
	}
	if ( pc.col == ( stereoWidth / 2 ) && pc.row == stereoHight - stereoHight / 4 )
	{
		interestPointsDepth[12] = pc.z;
	}

	if ( pc.col == stereoWidth - stereoWidth / 4 && pc.row == stereoHight - stereoHight / 4 )
	{
		interestPointsDepth[13] = pc.z;
	}
	if ( pc.col == stereoWidth - stereoWidth / 4 + stereoWidth / 5 && pc.row == stereoHight - stereoHight / 4 )
	{
		interestPointsDepth[14] = pc.z;
	}
}

void bb2_wrapper::fillInterestPoints3D( )
{
	// First line
	if ( pc.col == stereoWidth / 4 - stereoWidth / 5 && pc.row == stereoHight / 4 )
	{
		interestPoints3D[0].x = pc.x;
		interestPoints3D[0].y = pc.y;
		interestPoints3D[0].z = pc.z;
	}

	if ( pc.col == stereoWidth / 4 && pc.row == stereoHight / 4 )
	{
		interestPoints3D[1].x = pc.x;
		interestPoints3D[1].y = pc.y;
		interestPoints3D[1].z = pc.z;
	}
	if ( pc.col == stereoWidth / 2 && pc.row == stereoHight / 4 )
	{
		interestPoints3D[2].x = pc.x;
		interestPoints3D[2].y = pc.y;
		interestPoints3D[2].z = pc.z;
	}
	if ( pc.col == ( stereoWidth - stereoWidth / 4 ) && pc.row == stereoHight / 4 )
	{
		interestPoints3D[3].x = pc.x;
		interestPoints3D[3].y = pc.y;
		interestPoints3D[3].z = pc.z;
	}

	if ( pc.col == stereoWidth - stereoWidth / 4 + stereoWidth / 5 && pc.row == stereoHight / 4 )
	{
		interestPoints3D[4].x = pc.x;
		interestPoints3D[4].y = pc.y;
		interestPoints3D[4].z = pc.z;
	}

	// Second line
	if ( pc.col == stereoWidth / 4 - stereoWidth / 5 && pc.row == stereoHight / 2 )
	{
		interestPoints3D[5].x = pc.x;
		interestPoints3D[5].y = pc.y;
		interestPoints3D[5].z = pc.z;
	}
	if ( pc.col == stereoWidth / 4 && pc.row == stereoHight / 2 )
	{
		interestPoints3D[6].x = pc.x;
		interestPoints3D[6].y = pc.y;
		interestPoints3D[6].z = pc.z;
	}
	if ( pc.col == ( stereoWidth / 2 ) && pc.row == stereoHight / 2 )
	{
		interestPoints3D[7].x = pc.x;
		interestPoints3D[7].y = pc.y;
		interestPoints3D[7].z = pc.z;
	}

	if ( pc.col == stereoWidth - stereoWidth / 4 && pc.row == stereoHight / 2 )
	{
		interestPoints3D[8].x = pc.x;
		interestPoints3D[8].y = pc.y;
		interestPoints3D[8].z = pc.z;
	}
	if ( pc.col == stereoWidth - stereoWidth / 4 + stereoWidth / 5 && pc.row == stereoHight / 2 )
	{
		interestPoints3D[9].x = pc.x;
		interestPoints3D[9].y = pc.y;
		interestPoints3D[9].z = pc.z;
	}

	// Third line
	if ( pc.col == stereoWidth / 4 - stereoWidth / 5 && pc.row == stereoHight - stereoHight / 4 )
	{
		interestPoints3D[10].x = pc.x;
		interestPoints3D[10].y = pc.y;
		interestPoints3D[10].z = pc.z;
	}
	if ( pc.col == stereoWidth / 4 && pc.row == stereoHight - stereoHight / 4 )
	{
		interestPoints3D[11].x = pc.x;
		interestPoints3D[11].y = pc.y;
		interestPoints3D[11].z = pc.z;
	}
	if ( pc.col == ( stereoWidth / 2 ) && pc.row == stereoHight - stereoHight / 4 )
	{
		interestPoints3D[12].x = pc.x;
		interestPoints3D[12].y = pc.y;
		interestPoints3D[12].z = pc.z;
	}

	if ( pc.col == stereoWidth - stereoWidth / 4 && pc.row == stereoHight - stereoHight / 4 )
	{
		interestPoints3D[13].x = pc.x;
		interestPoints3D[13].y = pc.y;
		interestPoints3D[13].z = pc.z;
	}
	if ( pc.col == stereoWidth - stereoWidth / 4 + stereoWidth / 5 && pc.row == stereoHight - stereoHight / 4 )
	{
		interestPoints3D[14].x = pc.x;
		interestPoints3D[14].y = pc.y;
		interestPoints3D[14].z = pc.z;
	}
}

void bb2_wrapper::showInterestPointsDepth( ) const
{
	cout << "视野中标记点处的深度值： " << endl;
	for ( int i = 0; i < 5; i++ )
	{
		cout << interestPointsDepth[i] << "m" << '\t';	
	}
	cout << endl;

	for ( int i = 5; i < 10; i++ )
	{
		cout << interestPointsDepth[i] << "m" << '\t';	
	}
	cout << endl;

	for ( int i = 10; i < 15; i++ )
	{
		cout << interestPointsDepth[i] << "m" << '\t';	
	}
	cout << endl;
	cout << "--------------------------------------------";
	cout << endl;
}

void bb2_wrapper::showInterestPoints3D( ) const
{
	cout << fixed << setprecision( 2 );
	cout << "视野中标记点处的三维坐标： " << endl;
	
	for ( int i = 0; i < 5; i++ )
	{
		cout << "(" << interestPoints3D[i].x << "m, " << interestPoints3D[i].y << "m, " << interestPoints3D[i].z << "m)" << '\t';
	}
	cout << endl << endl;

	for ( int i = 5; i < 10; i++ )
	{
		cout << "(" << interestPoints3D[i].x << "m, " << interestPoints3D[i].y << "m, " << interestPoints3D[i].z << "m)" << '\t';
	}
	cout << endl << endl;

	for ( int i = 10; i < 15; i++ )
	{
		cout << "(" << interestPoints3D[i].x << "m, " << interestPoints3D[i].y << "m, " << interestPoints3D[i].z << "m)" << '\t';
	}
	cout << endl << endl;
	cout << "-----------------------------------------------------------------------------------------------------------------------";
	cout << endl;
}

void bb2_wrapper::fillAvrgDepth( )
{
	if ( pc.row > 10 && pc.row < 470 && pc.col > 10 && pc.col < 210 )
	{
		avrgDepth[0] += pc.z;
	}

	if ( pc.row > 10 && pc.row < 470 && pc.col >= 210 && pc.col < 430 )
	{
		avrgDepth[1] += pc.z;
	}

	if ( pc.row > 10 && pc.row < 470 && pc.col >= 430 && pc.col < 630 )
	{
		avrgDepth[2] += pc.z;
	}
}

void bb2_wrapper::showAvrgDepth( )
{
	cout << "三个窗口的平均深度值： ";
	cout << fixed << setprecision( 2 );

	cout << avrgDepth[0] / 200 / 460 << "m"<< '\t';	
	cout << avrgDepth[1] / 220 / 460 << "m"<< '\t';	
	cout << avrgDepth[2] / 200 / 460 << "m"<< '\t';	
	
	cout << endl;
	cout << "----------------------------------------------";
	cout << endl;

	for ( int i  = 0; i < 3; i++ )
	{
		avrgDepth[i] = 0;
	}
}

void bb2_wrapper::fillInterestArea( )
{
	if ( pc.row >= 215 && pc.row < 265 && pc.col >= 295 && pc.col < 345 )
	{
		int index = ( pc.row - 215 ) * 50 + ( pc.col - 295 );
		interestArea[index].x = pc.x;
		interestArea[index].y = pc.y;
		interestArea[index].z = pc.z;
	}
}

vector< CvPoint3D32f > bb2_wrapper::showInterestArea( ) const
{
	return interestArea;
}
