#include "roboteam_tactics/utils/BTRunner.h"
#include "ros/ros.h"

std::string bt::Node::status_desc;

namespace rtt {
    
bt::Node::Status BTRunner::run_once() {
    bt::Node::Status s = tree.Update();
    print();
    return s;
}

void BTRunner::run_until(bt::Node::Status status) {
    while (run_once() != status) {}
}
    
void BTRunner::print() {
    std::string msg = bt::Node::status_desc;
    bt::Node::status_desc = "";
    if (print_debug) {
        ROS_INFO("BT Update: %s", msg.c_str());
    }
}    
    
}
