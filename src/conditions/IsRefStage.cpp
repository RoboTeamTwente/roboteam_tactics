#include <inttypes.h>

#include "roboteam_tactics/conditions/IsRefStage.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"

#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/RefereeStage.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#include "roboteam_utils/RefLookup.h"
#include "roboteam_tactics/utils/debug_print.h"
#define RTT_CURRENT_DEBUG_TAG IsRefStage

namespace rtt {

RTT_REGISTER_CONDITION(IsRefStage);

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
        std::cout << "Refstage!\n";
		return Status::Success;
	} else {
        std::cout << "Not refstage!\n";
		return Status::Failure;
	}
}

} // rtt
