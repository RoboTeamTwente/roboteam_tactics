#pragma once

#include <cstdint>

#include "roboteam_msgs/BtDebugInfo.h"

namespace rtt {

// Sends an rqt bt trace to rqt regardless of RTT_ENABLE_BT_RQT_TRACE using the global publisher defined with GlobalPublisher<roboteam_msgs::BtDebugInfo>::get_publisher()
void send_rqt_bt_trace(const int& id, const std::string& name, const uint8_t& debugType, const uint8_t& status, const roboteam_msgs::Blackboard& bb);

#ifdef RTT_ENABLE_BT_RQT_TRACE

#define RTT_SEND_RQT_BT_TRACE(id, name, type, status, bb) ::rtt::send_rqt_bt_trace(id, name, type, status, bb);

#define CREATE_GLOBAL_RQT_BT_TRACE_PUBLISHER \
rtt::GlobalPublisher<roboteam_msgs::BtDebugInfo> RTT_GLOBAL_BT_DEBUG_INFO_PUBLISHER("bt_debug_info", 1000);

#else

#define RTT_SEND_RQT_BT_TRACE(id, name, type, status, msg)

#define CREATE_GLOBAL_RQT_BT_TRACE_PUBLISHER

#endif

} // rtt
