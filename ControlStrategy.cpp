#include <math.h>
#include <vector>
using std::vector;

#include "DirectionGenerator.h"

namespace control_module
{
  const double pi = 3.141592;
  const double rad_to_deg = 180 / pi;
  
  const double v_max = 2.5;  // unit: m/s
                             // This value is set before running the car according to its physical performance.
                             
  const double proportional_gain = 1.0;  // To be tuned to generate reasonable control speed
  
  // This class can compute the safety angle based on laser scan data; 
  // Also, it has method of generating next waypoint;
  // refer to 'DirectionGenerator.h' for more details.
  DirectionGenerator dg;
  
  // This function reads visual odometry measures and laser scan,
  // based on these data, it decides two controls
  void control( odometry anOdom,  // Get odometry data from VO by calling getOdom( ) of libviso2_wrapper 
  	            vector< long > laser_distance,  // Get laser scan data from hokuyo_wrapper
  						  double& controlAngle,  // unit: degree
  						  double& controlSpeed )  // unit: m/s
  {
  	// Step 1: SHOULD DO
  	// Process laser scan data to generate a best traversing angle (unit: degree),
  	// which refers to laser coordinate frame
  	double SafeAngleDeg = dg.process( laser_distance );
  	//double SafeAngleDeg = 20;  // For offline testing
  
  	// Step 2: WANT TO DO THIS
  	// x_next and y_next (unit:meter) are referenced to laser coordinate frame
  	double x_next = 0.0;
  	double y_next = 0.0;
  	dg.genWaypoint( laser_distance, x_next, y_next );
  	double angleToWaypoint = atan2( x_next - anOdom.z, -y_next - anOdom.x );
  	double angleToWaypointDeg = angleToWaypoint * rad_to_deg;  // Refers to VO frame
  	// Transforming to laser frame
  	double angleFromLaserToWaypoint = angleToWaypointDeg - ( 90 - anOdom.yaw_rad * rad_to_deg );
  
  	// Step 3: WOULD DO THIS
  	const double weightOfLocal = 0.0;
  	controlAngle = weightOfLocal * SafeAngleDeg + ( 1 - weightOfLocal ) * angleFromLaserToWaypoint;
  
  	// Step 4: velocity control planning
  	controlSpeed = v_max / sqrt( abs( proportional_gain * controlAngle ) + 1 );
  }
}
