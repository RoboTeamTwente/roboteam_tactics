#include "roboteam_tactics/treegen/NodeFactory.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"
#include "NodeWrapper.hpp"

template <class MsgType>
MsgType DecodeMsg(int len, char* data) {
    MsgType msg;
    ros::serialization::IStream stream((unsigned char*)data, len);
    ros::serialization::Serializer<MsgType>::read(stream, msg);
    return msg;
}

extern "C" {

    void RosInit() {
        int argc = 0;
        ros::init(argc, NULL, "CppSlave");

        new rtt::GlobalPublisher<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS);
        new rtt::GlobalPublisher<roboteam_msgs::RoleDirective>(rtt::TOPIC_ROLE_DIRECTIVE);
        new rtt::GlobalPublisher<roboteam_msgs::BtDebugInfo>("bt_debug_info", 1000);
    }

    void RosShutdown() {
        ros::shutdown();
    }

    void SetWorld(int len, char* data) {
        auto world = DecodeMsg<roboteam_msgs::World>(len, data);
        rtt::LastWorld::set(world);
    }

    void SetField(int len, char* data) {
        auto field = DecodeMsg<roboteam_msgs::GeometryFieldSize>(len, data);
        rtt::LastWorld::set_field(field);
    }

    void SetRef(int len, char* data) {
        auto refdata = DecodeMsg<roboteam_msgs::RefereeData>(len, data);
        rtt::LastRef::set(refdata);
    }

    CNode* NodeNew(char* className, char* nodeName, int len, char* bbdata) {
        auto bbmsg = DecodeMsg<roboteam_msgs::Blackboard>(len, bbdata);
        auto bb = std::make_shared<bt::Blackboard>();
        bb->fromMsg(bbmsg);
        auto ptr = rtt::generate_rtt_node(className, nodeName, bb);
        CNode *n = new CNode;
        n->CxxNode = ptr;
        return n;
    }
        
    void NodeInitiate(CNode* cn) {
        cn->CxxNode->Initialize();
    }

    void NodeUpdate(CNode* cn) {
        bt::Node::Status status = cn->CxxNode->Update();
        cn->CxxNode->setStatus(status);
    }

    void NodeTerminate(CNode* cn) {
        cn->CxxNode->Terminate(cn->CxxNode->getStatus());
    }

    int NodeStatus(CNode* cn) {
        return (int)cn->CxxNode->getStatus();
    }
}
