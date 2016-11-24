#include <inttypes.h>

#include "roboteam_tactics/conditions/IsRefStage.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/LastRef.h"

#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/RefereeStage.h"


namespace rtt {

IsRefStage::IsRefStage(std::string name, bt::Blackboard::Ptr blackboard)
        : Condition(name, blackboard) {}

bt::Node::Status IsRefStage::Update() {
	roboteam_msgs::RefereeData ref = LastRef::get();
	roboteam_msgs::RefereeStage refstage = ref.stage;
	int testStage = int(GetDouble("stage"));
	if(refstage.stage == testStage){
		ROS_INFO("ja, stage: %ld", (long)refstage.stage);
		return Status::Success;
	} else {
		ROS_INFO("nee, stage: %ld", (long)refstage.stage);
		return Status::Failure;
	}
}

} // rtt
