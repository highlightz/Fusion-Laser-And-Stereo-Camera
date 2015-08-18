double angleOfSafety = dg.process( distance );

// x_next and y_next are referenced to laser coordinate frame
dg.genWaypoint( distance, x_next, y_next );

odometry anOdom = libviso2.getOdom( );

// Calculate the angel to waypoint
const pi = 3.141592;
double angleToWaypoint = 0.0;
if ( anOdom.x == - y_next )
{
  angleToWaypoint = pi / 2;  // Or -pi/2
}
else
{
  double k = ( x_next - anOdom.z ) / ( - y_next - anOdom.x );
  angleToWaypoint = atan( k );
}

double angleFromLaserToWaypoint = angleToWaypoint - ( 90 - anOdom.yaw );

const double weightOfLocal = 0.6;
double controlAngle = weightOfLocal * angleOfSafety + ( 1 - weightOfLocal ) * angleFromLaserToWaypoint;


