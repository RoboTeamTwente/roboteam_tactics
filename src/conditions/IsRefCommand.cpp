#include <inttypes.h>

#include "roboteam_tactics/conditions/IsRefCommand.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"

#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/RefereeStage.h"
#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#include "roboteam_utils/RefLookup.h"
#include "roboteam_tactics/utils/debug_print.h"
#define RTT_CURRENT_DEBUG_TAG IsRefCommand

namespace rtt {

RTT_REGISTER_CONDITION(IsRefCommand);

IsRefCommand::IsRefCommand(std::string name, bt::Blackboard::Ptr blackboard)
        : Condition(name, blackboard) {}

bt::Node::Status IsRefCommand::Update() {
	// roboteam_msgs::RefereeData ref = LastRef::get();
	// roboteam_msgs::RefereeCommand refcommand = ref.command;
	RefState testCommand;

	if (HasDouble("command") || HasInt("command")){
        int possibleRefState = (int) GetDouble("command");

        // If both are available, we prefer the integer command
        if (private_bb->HasInt("command")) {
            possibleRefState = GetInt("command");
        }

        if (auto refStateOpt = toRefState(possibleRefState)) {
            testCommand = *refStateOpt;
        } else {
            ROS_ERROR("Invalid refstate supplied: %d", possibleRefState);
            return Status::Failure;
        }
	} else if(HasString("command")){
        if (auto refStateOpt = stringToRefState(GetString("command"))) {
            testCommand = *refStateOpt;
        } else {
            ROS_ERROR("Invalid refstate string supplied: %s", GetString("command").c_str());
            return Status::Failure;
        }
	} else {
		RTT_DEBUGLN("no good blackboard");
		return Status::Failure;
	}
	
	bool previous = HasBool("previous") && GetBool("previous");
	
    if (previous) {
        if(LastRef::getPreviousRefCommand() == testCommand) {
            return Status::Success;
        }
    } else if(LastRef::hasReceivedFirstCommand() && LastRef::getState() == testCommand){
		return Status::Success;
	} 

    return Status::Failure;
}

} // rtt
