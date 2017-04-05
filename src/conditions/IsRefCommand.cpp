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
	roboteam_msgs::RefereeData ref = LastRef::get();
	roboteam_msgs::RefereeCommand refcommand = ref.command;
	int testCommand;

	if (private_bb->HasDouble("command")){
		testCommand = int(GetDouble("command"));
	} else if(private_bb->HasString("command")){
		testCommand= refcommandlookup.at(GetString("command"));
	} else {
		RTT_DEBUGLN("no good blackboard");
		return Status::Failure;
	}
	
	bool previous = HasBool("previous") && GetBool("previous");
	
    if (previous) {
        if(LastRef::getPreviousRefCommand() == testCommand) {
            return Status::Success;
        }
    } else if(refcommand.command == testCommand){
		return Status::Success;
	} 

    return Status::Failure;
}

} // rtt
