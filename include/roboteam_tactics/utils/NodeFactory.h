#include "ros/ros.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/generated/allskills_factory.h"
#include "roboteam_tactics/generated/alltrees_factory.h"
//#include "roboteam_tactics/generated/alltactics_factory.h"
#include "roboteam_tactics/generated/allskills_set.h"
#include "roboteam_tactics/generated/alltrees_set.h"
#include "roboteam_tactics/generated/alltactics_set.h"

namespace rtt {

std::shared_ptr<bt::Node> generate_node(ros::NodeHandle& n, std::string className, std::string nodeName, bt::Blackboard::Ptr bb);

}