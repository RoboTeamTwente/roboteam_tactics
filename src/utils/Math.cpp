#include "ros/ros.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/Math.h"
#include <cmath>

namespace rtt {

double cleanAngle(double angle){
	if (angle <= -M_PI){
		return fmod(angle-M_PI, (2*M_PI))+M_PI;
	} else if(angle > M_PI){
		return fmod(angle+M_PI, (2*M_PI))-M_PI;
	} else {
		return angle;
	}
}

roboteam_utils::Vector2 worldToRobotFrame(roboteam_utils::Vector2 requiredv, double rotation){
    roboteam_utils::Vector2 robotRequiredv;
    robotRequiredv.x=requiredv.x*cos(-rotation)-requiredv.y*sin(-rotation);
    robotRequiredv.y=requiredv.x*sin(-rotation)+requiredv.y*cos(-rotation);
	return robotRequiredv;
}

double computeAngle(roboteam_utils::Vector2 robotPos, roboteam_utils::Vector2 faceTowardsPos) {
	roboteam_utils::Vector2 differenceVector = faceTowardsPos - robotPos;
	return differenceVector.angle();
}





} // rtt
