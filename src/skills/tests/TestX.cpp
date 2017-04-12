#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <ros/ros.h>

#include "roboteam_msgs/GeometryData.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/RoleDirective.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/constants.h"

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/treegen/NodeFactory.h"
#include "roboteam_tactics/utils/BTRunner.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

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

void msgCallbackRef(const roboteam_msgs::RefereeDataConstPtr& refdata) {
    rtt::LastRef::set(*refdata);
    //ROS_INFO("set ref, timestamp: %d",refdata->packet_timestamp);
}

int main(int argc, char **argv) {
    std::vector<std::string> arguments(argv + 1, argv + argc);

    if (arguments.size() == 0) {
        arguments.push_back("help");
    }

    if (arguments.at(0) == "show" && arguments.size() >= 2) {
        if (arguments.at(1) == "skills") {
            rtt::factories::print_all<rtt::Skill>("skills");
        } else if (arguments.at(1) == "conditions") {
            rtt::factories::print_all<rtt::Condition>("conditions");
        } else if (arguments.at(1) == "tactics") {
            rtt::factories::print_all<rtt::Tactic>("tactics");
        } else if (arguments.at(1) == "trees") {
            rtt::factories::print_all<bt::BehaviorTree>("trees");
        } else if (arguments.at(1) == "all") {
            rtt::factories::print_all<rtt::Skill>("skills");
            rtt::factories::print_all<rtt::Condition>("conditions");
            rtt::factories::print_all<rtt::Tactic>("tactics");
            rtt::factories::print_all<bt::BehaviorTree>("trees");
        } else {
            std::cout << "second argument from show not recognized (\"" << arguments.at(1) << "\")\n";
            arguments = { "help "};
        }
    }

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
- To display all skills, conditions, tactics, or trees:
  rosrun roboteam_tactics TestX show [skills/conditions/tactics/trees/all]
- To display this help:
  "rosrun roboteam_tactics TestX help"
)###";
        std::cout << msg << "\n";
        return 0;
    }

	ros::init(argc, argv, "TestX", ros::init_options::AnonymousName);
	ros::NodeHandle n;

    auto bb = std::make_shared<bt::Blackboard>();

    std::string testClass = arguments.at(0);
    std::cout << "Test class: " << testClass << "\n";

    for (size_t i = 1; i < arguments.size(); i++) {
        std::vector<std::string> typeSplit;

        auto arg = arguments.at(i);

        if (arg.find(":=") != std::string::npos) {
            // If := is in the string, it's a ROS argument, so we skip it
            continue;
        }

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
        // TODO: Factor the logic here into a few common functions
        // (one for going string -> T and one for going T -> string)
        // and use them here, paramset, paramget, and in some of Dennis's bb code

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

    //rtt::print_blackboard(bb);

    // Create subscribers for world & geom messages
    rtt::WorldAndGeomCallbackCreator cb;
    
    CREATE_GLOBAL_RQT_BT_TRACE_PUBLISHER;

    rtt::GlobalPublisher<roboteam_msgs::RobotCommand> globalRobotCommandPublisher(rtt::TOPIC_COMMANDS);
    rtt::GlobalPublisher<roboteam_msgs::RoleDirective> globalRoleDirectivePublisher(rtt::TOPIC_ROLE_DIRECTIVE);

    // Create subscriber for referee messages
    ros::Subscriber ref_sub = n.subscribe<roboteam_msgs::RefereeData> ("vision_refbox", 1000, msgCallbackRef);

    // Wait for the first geom & world message
    rtt::LastWorld::wait_for_first_messages();

    std::shared_ptr<bt::Node> node = rtt::generate_rtt_node<>(testClass, "", bb);

    if (!node) {
        ROS_ERROR("Test for \"%s\" could not be constructred. Aborting.", testClass.c_str());
        return 1;
    }

    if (rtt::factories::isTactic(testClass)) {
        std::cout << "Testing a tactic! Please ensure that 6 role nodes are available...\n";
        std::cout << "(For example by using mini_rolenodes.launch from roboteam_utils)\n";
        
        auto& directivePub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
        ros::Rate fps60(60);
        while ((int) directivePub.getNumSubscribers() < 6) {
            ros::spinOnce();
            fps60.sleep();

            if (!ros::ok()) {
                std::cout << "Interrupt received, exiting...";
                return 0;
            }
        }

        std::cout << "Spotted 6 role directives, carrying on!\n";

        rtt::RobotDealer::initialize_robots(5, {0, 1, 2, 3, 4});

        // TODO: Maybe at the end ensure that the role nodes stop the execution?
        // And this can then be prevented with a command line switch
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    double updateRate = 30;
    ros::param::set("role_iterations_per_second", updateRate);
    ros::Rate fps(updateRate);

    if (rtt::factories::isTree(testClass)) {
        // Notify the tree debugger that we're running a tree.
        RTT_SEND_RQT_BT_TRACE(testClass, roboteam_msgs::BtDebugInfo::TYPE_ROLE, roboteam_msgs::BtStatus::STARTUP, bb->toMsg());

        bt::BehaviorTree* bt = dynamic_cast<bt::BehaviorTree*>(&(*node));

        bt->Initialize();

        rtt::BTRunner runner(*bt, false);
		runner.run_until([&](bt::Node::Status previousStatus) {
            ros::spinOnce();
            fps.sleep();
            return ros::ok() && previousStatus != bt::Node::Status::Success && previousStatus != bt::Node::Status::Failure;
        });

        // TODO: This is a hack, and if the thing above this is just a while loop
        // we can terminate or tick just fine.
        bt->Terminate(bt::Node::Status::Failure);
    } else {
        node->Initialize();

        bt::Node::Status status;
        while (ros::ok()) {
            ros::spinOnce();

            status = node->Update();

            if (status == bt::Node::Status::Success || status == bt::Node::Status::Failure) {
                break;
            }

            fps.sleep();
        }

        std::cout << "Terminating. Final status: " << bt::statusToString(status) << "\n";

        node->Terminate(status);
    }

    // Gracefully close all the publishers.
    n.shutdown();

    std::cout << "Test of " << testClass << " completed!\n";
	return 0;
}
