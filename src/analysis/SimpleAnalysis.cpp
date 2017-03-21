#include "roboteam_msgs/World.h"

namespace rtt {

bool countOurRobots(roboteam_msgs::World const & world) {
	return world.us.size();
}

} // namespace rtt
