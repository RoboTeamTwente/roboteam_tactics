#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <unistd.h>
#include <ros/ros.h>

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/generated/allskills_factory.h"
#include "roboteam_tactics/utils/NodeFactory.h"
#include "roboteam_tactics/utils/BTRunner.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_msgs/World.h"

static volatile bool may_update = false;

void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

void msgCallBackGoToPos(const roboteam_msgs::WorldConstPtr& world) {
	rtt::LastWorld::set(*world);
    may_update = true;
}

void msgCallbackFieldGeometry(const roboteam_msgs::GeometryDataConstPtr& geometry) {
    rtt::LastWorld::set_field(geometry->field);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "TestX", ros::init_options::AnonymousName);
	ros::NodeHandle n;

    auto bb = std::make_shared<bt::Blackboard>();

    std::vector<std::string> arguments(argv + 1, argv + argc);

    if (arguments.at(0) == "help") {
        std::string msg = R"###(
[TestX]

How to use:
- To just test a skill:
  "rosrun roboteam_tactics TestX [classname] argType1:argName1=argValue1 argType2:argName2=argValue2"
  Here [classname] is the name of the classname you want to test (e.g. GoToPos).
  argType# can be any of the following:
    - string
    - int
    - double
    - bool

  If the type is string the value should be surrounded by quotes (e.g. "This is a string value!").
  A double value has a decimal point (not a comma).
  A bool is "true" or "false" without quotes.
- To display this help:
  "rosrun roboteam_tactics TestX help"
)###";
        std::cout << msg << "\n";
        return 0;
    }

    std::string testClass = arguments.at(0);
    std::cout << "Test class: " << testClass << "\n";

    for (size_t i = 1; i < arguments.size(); i++) {
        std::vector<std::string> typeSplit;

        auto arg = arguments.at(i);

        auto nameSplit = split(arg, '=');
        auto name = nameSplit.at(0);
        auto rest = nameSplit.at(1);

        // Aggregate all the splitted = into one string
        // This happens if you try to set a value that contains multiple equals
        // (Then you only want to split on the first)
        for (size_t i = 2; i < nameSplit.size(); i++) {
            rest += nameSplit.at(i);
        }

        std::string argType;
        if (name.find(":") != std::string::npos) {
            // Name contains type - lets take it out
            auto typeSplit = split(name, ':');
            argType = typeSplit.at(0);
            name = typeSplit.at(1);
        } else {
            // Derive type
            if (rest == "true") {
                argType = "bool";
            } else if (rest == "false") {
                argType = "bool";
            } else if (rest.find(".") != std::string::npos) {
                argType = "double";
            } else if (rtt::is_digits(rest)) {
                argType = "int";
            } else {
                argType = "string";
            }
        }


        // Uncomment to see the arguments
        // std::cout << "\n[Arg]\n";
        // std::cout << "Type: " << argType << "\n";
        // std::cout << "Name: " << name << "\n";
        // std::cout << "Value: " << rest << "\n";

        if (argType == "string") {
            bb->SetString(name, rest);
        } else if (argType == "int") {
            bb->SetInt(name, std::stoi(rest));
        } else if (argType == "double") {
            bb->SetDouble(name, std::stod(rest));
        } else if (argType == "bool") {
            bb->SetBool(name, rest == "true");
        } else {
            std::cout << "Unknown arg type: " << argType << "\n";
        }
    }

    rtt::print_blackboard(bb);
    ros::Subscriber world_sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, msgCallBackGoToPos);
    ros::Subscriber geom_sub = n.subscribe<roboteam_msgs::GeometryData> ("vision_geometry", 1000, msgCallbackFieldGeometry);

    while (!may_update) ros::spinOnce();
    std::shared_ptr<bt::Node> node = rtt::generate_node(n, testClass, "", bb);

    bt::BehaviorTree* is_bt = dynamic_cast<bt::BehaviorTree*>(&(*node));

    ros::Rate fps60(60);

    if (is_bt) {
        rtt::BTRunner runner(*is_bt, false);

        runner.run_until([&](bt::Node::Status previousStatus) {
            ros::spinOnce();
            fps60.sleep();

            return ros::ok() && previousStatus != bt::Node::Status::Success && previousStatus != bt::Node::Status::Failure;
        });
    } else {
        node->Initialize();

        bt::Node::Status status;
        while (ros::ok()) {
            ros::spinOnce();
            status = node->Update();
            if (status == bt::Node::Status::Success || status == bt::Node::Status::Failure) {
                break;
            }
            fps60.sleep();
        }

        std::cout << "Terminating..." << std::endl;

        node->Terminate(status);
    }

    // Busy loop to make sure all the messages have been sent.
    // Not sure if neccesary.
    for (int i = 0; i < 10; i++) {
        fps60.sleep();
        ros::spinOnce();
    }

    // Gracefully close all the publishers.
    n.shutdown();

    std::cout << "Test of " << testClass << " completed!\n";
	return 0;
}
