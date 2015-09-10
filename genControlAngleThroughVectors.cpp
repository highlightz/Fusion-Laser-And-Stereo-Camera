#include <math.h>

#include <iostream>
#include <vector>
using namespace std;

const double pi = 3.14;

double genControlAngle( double angle, double targetX, double targetY )
{
	// Input
	// -----------------------------------
	double totalAngle = angle * pi / 180;
	double target_x = targetX;
	double target_y = targetY;
	// ------------------------------------

	totalAngle = pi / 2 - totalAngle;
	
	double directionVectorCurrX = cos( totalAngle );
	double directionVectorCurrY = sin( totalAngle );
	
	//cout << directionVectorCurrX << "," << directionVectorCurrY << endl;
	
	double curr_x = 0;
	double curr_y = 0;
	double targetVectorX = target_x - curr_x;
	double targetVectorY = target_y - curr_y;
	// Normalization
	double normalizedTargetVectorX = targetVectorX / sqrt( targetVectorX * targetVectorX + targetVectorY * targetVectorY );
	double normalizedTargetVectorY = targetVectorY / sqrt( targetVectorX * targetVectorX + targetVectorY * targetVectorY );

	//cout << normalizedTargetVectorX << "," << normalizedTargetVectorY << endl;

	double dotMux = directionVectorCurrX * normalizedTargetVectorX + directionVectorCurrY * normalizedTargetVectorY;
	double modCurr = sqrt( directionVectorCurrX * directionVectorCurrX + directionVectorCurrY * directionVectorCurrY );
	double modNormTarget = sqrt( normalizedTargetVectorX * normalizedTargetVectorX + normalizedTargetVectorY * normalizedTargetVectorY );
	double cos_theta = dotMux / modCurr / modNormTarget;
	double theta = acos( cos_theta );

	//cout << endl << theta * 180 / pi << endl;

	double controlAngle = -( atan2( normalizedTargetVectorY, normalizedTargetVectorX ) * 180 / pi - atan2( directionVectorCurrY, directionVectorCurrX ) * 180 / pi );

	return controlAngle;
}

int main( )
{
	cout << "Control Angle: " << genControlAngle( 0, 6, -6 ) << endl;
	return 0;
}
