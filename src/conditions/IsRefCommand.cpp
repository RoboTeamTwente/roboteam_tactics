#include <inttypes.h>

#include "roboteam_tactics/conditions/IsRefCommand.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/LastRef.h"

#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/RefereeStage.h"
#include "roboteam_msgs/RefereeCommand.h"

#include "roboteam_utils/RefLookup.h"
#include "roboteam_tactics/utils/debug_print.h"
#define RTT_CURRENT_DEBUG_TAG IsRefCommand

namespace rtt {

IsRefCommand::IsRefCommand(std::string name, bt::Blackboard::Ptr blackboard)
        : Condition(name, blackboard) {}

bt::Node::Status IsRefCommand::Update() {
	roboteam_msgs::RefereeData ref = LastRef::get();
	roboteam_msgs::RefereeCommand refcommand = ref.command;
	int testCommand;
	
	
	if (private_bb->HasDouble("command")){
		testCommand = int(GetDouble("command"));
		
	} else if(private_bb->HasString("command")){
		testCommand= refcommandlookup.at(GetString("command"));
	
	} else {
		RTT_DEBUG("no good blackboard \r\n");
		return Status::Failure;
	}
	
	
	
	if(refcommand.command == testCommand){
		ROS_INFO("ja, command: %ld", (long)refcommand.command);
		return Status::Success;
	} else {
		ROS_INFO("nee, command: %ld", (long)refcommand.command);
		return Status::Failure;
	}
	
}

} // rtt
