#include <inttypes.h>

#include "roboteam_tactics/conditions/IsRefStage.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/LastRef.h"

#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/RefereeStage.h"

#include "roboteam_utils/RefLookup.h"
#include "roboteam_tactics/utils/debug_print.h"
#define RTT_CURRENT_DEBUG_TAG IsRefStage

namespace rtt {

IsRefStage::IsRefStage(std::string name, bt::Blackboard::Ptr blackboard)
        : Condition(name, blackboard) {}

bt::Node::Status IsRefStage::Update() {
	roboteam_msgs::RefereeData ref = LastRef::get();
	roboteam_msgs::RefereeStage refstage = ref.stage;
	int testStage;
	
	if (private_bb->HasDouble("stage")){
		testStage = int(GetDouble("stage"));
		
	} else if(private_bb->HasString("stage")){
		testStage= refstagelookup.at(GetString("stage"));
	
	} else {
		RTT_DEBUG("no good blackboard \r\n");
		return Status::Failure;
	}
	
	
	
	if(refstage.stage == testStage){
		ROS_INFO("ja, stage: %ld", (long)refstage.stage);
		return Status::Success;
	} else {
		ROS_INFO("nee, stage: %ld", (long)refstage.stage);
		return Status::Failure;
	}
}

} // rtt
