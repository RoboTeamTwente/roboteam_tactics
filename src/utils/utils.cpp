#include <ros/ros.h>

#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_msgs/World.h"

namespace rtt {

/**
 * Gets all the nodes subscribed to topic topic, relative
 * to the current node.
 */
std::vector<std::string> getNodesSubscribedTo(std::string topic) {
    // Adapted from ros::master::getNodes()
    // Do the xmlrpc request to master
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();

    if (!ros::master::execute("getSystemState", args, result, payload, true)) {
        // Return an empty list if something went wrong
        return {};
    }

    // See: http://wiki.ros.org/ROS/Master_API
    // and then getSystemState for the format
    std::vector<std::string> nodes;
    XmlRpc::XmlRpcValue subscribers = payload[1];
    for (int i = 0; i < subscribers.size(); ++i) {
        XmlRpc::XmlRpcValue topicInfo = subscribers[i];
        XmlRpc::XmlRpcValue topicName = topicInfo[0];
        XmlRpc::XmlRpcValue topicSubscribers = topicInfo[1];

        if (topicName == topic) {
            for (int j = 0; j < topicSubscribers.size(); ++j) {
                nodes.push_back(topicSubscribers[j]);
            }
            break;
        }
    }

    return nodes;
}

std::string getMyNamespace() {
    std::string ns = ros::this_node::getNamespace();

    if (ns.size() >= 2) {
        if (ns.substr(0, 2) == "//") {
            ns = ns.substr(1, ns.size() - 1);
        }
    }

    if (ns.size() >= 1) {
        if (ns.at(ns.size() - 1) != '/') {
            ns += "/";
        }
    }
    
    return ns;
}

boost::optional<std::pair<roboteam_msgs::WorldRobot, bool>> getBallHolder() {
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    bb->SetBool("our_team", true);
    for (auto& bot : LastWorld::get().us) {
        bb->SetInt("me", bot.id);
        IHaveBall ihb("", bb);
        if (ihb.Update() == bt::Node::Status::Success) {
            return boost::optional<std::pair<roboteam_msgs::WorldRobot, bool>>(std::make_pair(bot, true));
        }
    }
    bb->SetBool("our_team", false);
    for (auto& bot : LastWorld::get().them) {
        bb->SetInt("me", bot.id);
        IHaveBall ihb("", bb);
        if (ihb.Update() == bt::Node::Status::Success) {
            return boost::optional<std::pair<roboteam_msgs::WorldRobot, bool>>(std::make_pair(bot, false));
        }
    }
    return boost::optional<std::pair<roboteam_msgs::WorldRobot, bool>>();
}

} // rtt

