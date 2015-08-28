// By Xiaojia Xie
// Aug. 26, 2015
// Function: sends controls to MCU to control the car
// Decription:
// Lengh of instruction: 10 bytes
// Frame header: fffe
// Control segment: two float number, one for velocity, and the other for direction.
// Range: [-1, 1]

/*
// Prepare controls
float spd =  0.3f;
float ang = -0.1f;

// Pre-define the control header	
char send[10];
send[0] = 0xff
send[1] = 0xfe;

// Speed control
const int *p = (int *)&spd;
send[5] = (*p>>24)&0xff;
send[4] = (*p>>16)&0xff;
send[3] = (*p>>8)&0xff;
send[2] = (*p>>0)&0xff;

// Angle control
p = (int *)&ang;
send[9] = (*p>>24)&0xff;
send[8] = (*p>>16)&0xff;
send[7] = (*p>>8)&0xff;
send[6] = (*p>>0)&0xff;
*/

#include <math.h>
#include <Windows.h>
#include <vector>
using std::vector;
#include <iostream>
using std::cout;
using std::endl;

#include "stdafx.h"
#include "SerialPort.h"

void genControls( vector< float >& angles, vector< float >& speeds )
{
	// Controls length
	const int numOfControls = 10;

	// Controls ranges
	const float maxAngle =  0.7f;
  const float minAngle = -0.7f;
  const float maxSpeed =  0.7f;

	// Specify angles
	angles[0] =  0.8f;
  angles[1] =  0.1f;
  angles[2] = -0.2f;
  angles[3] = -0.3f;
  angles[4] =  0.4f;

  angles[5] =  0.5f;
  angles[6] = -0.4f;
  angles[7] = -1.0f;
  angles[8] =  0.2f;
  angles[9] =  0.1f;

	// Limit the angle controls to a safe range
  for ( int i = 0; i < numOfControls; i++ )
  {
      if ( angles[i] > maxAngle )
      {
        angles[i] = maxAngle;
      }

      if ( angles[i] < minAngle )
      {
        angles[i] = minAngle;
      }
  }

	// Compute speeds according to specified angles
  for ( int i = 0; i < numOfControls; i++ )
  {
    const float p = 10.0f;
    speeds[i] = maxSpeed / sqrt( fabs( p * angles[i] ) + 1.0f );
  }
}

void genSend( float angle, float speed, char* send )
{
	send[0] = 0xff;
	send[1] = 0xfe;

	// Speed control
	const int *p = (int *)&speed;
	send[5] = (*p>>24)&0xff;
	send[4] = (*p>>16)&0xff;
	send[3] = (*p>>8)&0xff;
	send[2] = (*p>>0)&0xff;

	// Angle control
	p = (int *)&angle;
	send[9] = (*p>>24)&0xff;
	send[8] = (*p>>16)&0xff;
	send[7] = (*p>>8)&0xff;
	send[6] = (*p>>0)&0xff;
}

void writeToPort( char* send )
{
	try
	{
		CSerialPort port;
		port.Open( 4, 115200, CSerialPort::NoParity, 8, CSerialPort::OneStopBit, CSerialPort::XonXoffFlowControl );
		
		std::cout << "Done sending!" << std::endl;
		port.Write( send, 10 );

		port.Flush(  );

		port.Close(  );
	}
	catch( CSerialException *pEx )
	{
		TRACE( _T( "Handle Exception, Message:%s\n" ), pEx->GetErrorMessage(  ) );
		pEx->Delete(  );
	}
}

void makeStop( )
{
	float speed = 0.0f;
	float angle = 0.0f;

	char send[10];
	genSend( angle, speed, send );
	writeToPort( send );

	cout << "Stopped!" << endl;
}

int main(  )
{
	// Generate controls
	const int numOfControls( 10 );
	vector< float > angles( numOfControls );
	vector< float > speeds( numOfControls );
	genControls( angles, speeds );

	// Signal to be sent to serial port
	char send[10];

	for ( int i = 0; i < numOfControls; i++ )
	{
		genSend( angles[i], speeds[i], send );
		
		writeToPort( send );

		Sleep( 1000 );
	}

	makeStop( );
	return 0;
}

/*
Output example:
Speed       Angle      Output
1.0f        1.0f       FF FE 3F 80 00 00 3F 80 00 00
1.0f        -1.0f      FF FE 3F 80 00 00 BF 80 00 00
-1.0f       1.0f       FF FE BF 80 00 00 3F 80 00 00
-1.0f       1.0f       FF FE BF 80 00 00 BF 80 00 00
*/
