#include "roboteam_tactics/utils/BtDebug.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

void send_rqt_bt_trace(const std::string& name, const uint8_t& debugType, const uint8_t& status, const roboteam_msgs::Blackboard& bb) {
    roboteam_msgs::BtDebugInfo msg;

    msg.name = name;
    msg.type = debugType;
    msg.status.status = status;
    msg.blackboard = bb;

    GlobalPublisher<roboteam_msgs::BtDebugInfo>::get_publisher().publish(msg);
}

} // rtt
