#include <map>

#include "uuid_msgs/UniqueID.h"
#include "unique_id/unique_id.h"

#include "roboteam_tactics/bt.hpp"

namespace rtt {

std::map<boost::uuids::uuid, bt::Node::Status> feedbacks;

}
