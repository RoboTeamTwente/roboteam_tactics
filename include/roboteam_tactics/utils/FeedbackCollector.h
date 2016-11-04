#pragma once

#include <map>
#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>

#include "uuid_msgs/UniqueID.h"
#include "unique_id/unique_id.h"

#include "roboteam_tactics/bt.hpp"

namespace rtt {

extern std::map<boost::uuids::uuid, bt::Node::Status> feedbacks;

}

