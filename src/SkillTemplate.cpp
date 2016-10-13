#include "ros/ros.h"

namespace rtt {

class GoToPos : public Skill {

private: 
	roboteam_msgs::SteeringGoal prevGoal;

public:
	GoToPos(Aggregator& aggregator) : 
			Skill{aggregator} {	
	}

	Status Update (){
		return Status::Running;
	}
}

} //rtt

int main() {
	return 0;
}