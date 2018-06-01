#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <signal.h>

#include "roboteam_msgs/GeometryData.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/RoleFeedback.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/constants.h"

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/treegen/NodeFactory.h"
#include "roboteam_tactics/utils/BTRunner.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/RobotDealer.h"

namespace {

bool mustShutdown = false;
unsigned sigCounter = 0;

// We have our own sigint handler here. Normally when you CTRL+C, ros shuts
// down all the publishers, subscribers, and all other cool things needed
// to use the ROS infrastructure. However, when someone CTRL+C's when TestX
// is running, before closing TestX we'd like to send termination messages
// (stop instructions/robotocommands) to the robots so they don't keep driving
// after TestX is closed. So we need to trap the SigInt ourselves, so we
// can shut down gracefully.

void mySigintHandler(int sig) {
    if (sig == SIGINT) {
        mustShutdown = true;
        sigCounter++;
    }

    ROS_INFO_STREAM_NAMED("TestX", "Sig: " << sig);
    if (sigCounter >= 3) {
        ROS_WARN_STREAM_NAMED("TestX", "Received at least 3 SIGINTs, so stopping ungracefully");
    	std::exit(0);
    }
}

void feedbackCallback(const roboteam_msgs::RoleFeedbackConstPtr &msg) {

    auto uuid = unique_id::fromMsg(msg->token);

    if (msg->status == roboteam_msgs::RoleFeedback::STATUS_FAILURE) {
        rtt::feedbacks[uuid] = bt::Node::Status::Failure;
        // std::cout << "Received a feedback on token " << uuid << ": failure.\n";
    } else if (msg->status == roboteam_msgs::RoleFeedback::STATUS_INVALID) {
        rtt::feedbacks[uuid] = bt::Node::Status::Invalid;
        // std::cout << "Received a feedback on token " << uuid << ": invalid.\n";
    } else if (msg->status == roboteam_msgs::RoleFeedback::STATUS_SUCCESS) {
        rtt::feedbacks[uuid] = bt::Node::Status::Success;
        // std::cout << "Received a feedback on token " << uuid << ": success.\n";
    }
}

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

void stopRolenode(int const id) {
    auto & pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    roboteam_msgs::RoleDirective rd;
    rd.robot_id = id;
    rd.tree = rd.STOP_EXECUTING_TREE;

    pub.publish(rd);
}

void stopRobot(int const id) {
    auto & pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();

    roboteam_msgs::RobotCommand rc;
    rc.id = id;

    pub.publish(rc);
}

void reportNodeFailure(std::string const & testClass) {
    using namespace rtt::factories;
    auto& treeRepo = getRepo<Factory<bt::BehaviorTree>>();

    ROS_ERROR("Test for \"%s\" could not be constructred. Aborting.", testClass.c_str());

    if (treeRepo.size() == 0) {
        ROS_ERROR("Also, no trees have been found. If the build is configured properly you need to run refresh_b3_projects.sh in roboteam_tactics to make sure the build system links in all the trees. If you're unsure if you've added your trees check if you've added your Behavior3 project or tree to the proper CMakeLists.txt in src/trees/json or src/trees/projects in roboteam_tactics."); 
    }
}

} // anonymous namespace

int main(int argc, char **argv) {
    std::vector<std::string> arguments(argv + 1, argv + argc);

    if (arguments.size() == 0) {
        arguments.push_back("help");
    }

    if (arguments.at(0) == "show" && arguments.size() >= 2) {
        if (arguments.at(1) == "skills") {
            rtt::factories::print_all<rtt::Skill>("skills");
            return 0;
        } else if (arguments.at(1) == "conditions") {
            rtt::factories::print_all<rtt::Condition>("conditions");
            return 0;
        } else if (arguments.at(1) == "tactics") {
            rtt::factories::print_all<rtt::Tactic>("tactics");
            return 0;
        } else if (arguments.at(1) == "trees") {
            rtt::factories::print_all<bt::BehaviorTree>("trees");
            return 0;
        } else if (arguments.at(1) == "all") {
            rtt::factories::print_all<rtt::Skill>("skills");
            rtt::factories::print_all<rtt::Condition>("conditions");
            rtt::factories::print_all<rtt::Tactic>("tactics");
            rtt::factories::print_all<bt::BehaviorTree>("trees");
            return 0;
        } else {
            ROS_WARN_STREAM_NAMED("TestX", "Second argument from show not recognized (\"" << arguments.at(1) << "\")");
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

    // Install our own signal handler to ensure that publishers
    // are closed at the end of main instead of when ctrl-c is pressed
    signal(SIGINT, mySigintHandler);

    ros::Subscriber feedbackSub = n.subscribe<roboteam_msgs::RoleFeedback>(
        rtt::TOPIC_ROLE_FEEDBACK,
        10,
        &feedbackCallback
        );

    auto bb = std::make_shared<bt::Blackboard>();

    std::string testClass = arguments.at(0);

    ROS_INFO_STREAM_NAMED("TestX", "Test class : " << testClass);

    /* For each argument, figure out its type, parse it, and add it to the blackboard */
    for (size_t i = 1; i < arguments.size(); i++) {
        std::vector<std::string> typeSplit;

        // Get argument
        auto arg = arguments.at(i);

        // If := is in the string, it's a ROS argument, so we skip it
        if (arg.find(":=") != std::string::npos) {
            continue;
        }

        // Split argument
        auto nameSplit = split(arg, '=');
        auto name = nameSplit.at(0);
        auto rest = nameSplit.at(1);

        // Aggregate all the splitted = into one string
        // This happens if you try to set a value that contains multiple equals
        // (Then you only want to split on the first)
        for (size_t i = 2; i < nameSplit.size(); i++) {
            rest += nameSplit.at(i);
        }

        // ====== Figure out the type of the argument ====== //
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
            ROS_WARN_STREAM_NAMED("TestX", "implicitly derived type '" << argType << "' for argument '" << rest << "'");
        }
        // ================================================= //

        // Update blackboard with the new value
        if (argType == "string") {
            bb->SetString(name, rest);
        } else if (argType == "int") {
            bb->SetInt(name, std::stoi(rest));
        } else if (argType == "double") {
            bb->SetDouble(name, std::stod(rest));
        } else if (argType == "bool") {
            bb->SetBool(name, rest == "true");
        } else {
            ROS_WARN_STREAM_NAMED("TestX", "Unknown argument type '" << argType << "' for argument '" << rest << "'");
        }
    }

    // Create subscribers for world & geom messages
    rtt::WorldAndGeomCallbackCreator cb;

    CREATE_GLOBAL_RQT_BT_TRACE_PUBLISHER;

    rtt::GlobalPublisher<roboteam_msgs::RobotCommand> globalRobotCommandPublisher(rtt::TOPIC_COMMANDS);
    rtt::GlobalPublisher<roboteam_msgs::RoleDirective> globalRoleDirectivePublisher(rtt::TOPIC_ROLE_DIRECTIVE);

    // Create subscriber for referee messages
    ros::Subscriber ref_sub = n.subscribe<roboteam_msgs::RefereeData> ("vision_refbox", 1000, msgCallbackRef);

    // Wait for the first geom & world message
    ROS_INFO_STREAM_NAMED("TestX", "Waiting for first world & geom message...");
    rtt::LastWorld::wait_for_first_messages();
    ROS_INFO_STREAM_NAMED("TestX", "Received first messages, proceeding");

    std::shared_ptr<bt::Node> node = rtt::generate_rtt_node<>(testClass, "", bb);

    if (!node) {
        reportNodeFailure(testClass);
        return 1;
    }

    // If we test a Strategy, wait for a RoleNode to come online
    if (rtt::factories::isTactic(testClass)) {
        ROS_INFO_STREAM_NAMED("TestX", "Testing a tactic! Please ensure that at least 1 RoleNode is available...");

        auto& directivePub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
        ros::Rate fps10(10);
        while ((int) directivePub.getNumSubscribers() < 1) {
            ros::spinOnce();
            fps10.sleep();

            ROS_DEBUG_STREAM_NAMED("TestX", "mustShutdown = " << mustShutdown);

            if (mustShutdown) {
                ROS_INFO_STREAM_NAMED("TestX", "Interrupt received, exiting...");
                return 0;
            }
        }

        ROS_INFO_STREAM_NAMED("TestX", "Spotted 1 RoleDirective listener, carrying on!");

        // TODO: Maybe at the end ensure that the role nodes stop the execution?
        // And this can then be prevented with a command line switch
    }

    // Emiel : Not sure why this is here.. Maybe to give newly created RoleNodes the time to initialize?
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // === Get the Hz
    double updateRate = 30;
    ros::param::get("role_iterations_per_second", updateRate);
    if (updateRate == 0) {
        ROS_ERROR_STREAM_NAMED("TestX", "role_iterations_per_second == 0. Aborting!");
        return 1;
    }
    ros::Rate fps(updateRate);
    ROS_INFO_STREAM_NAMED("TestX", "Updating at " << updateRate << "Hz");
    // ===

    int keeper_id = 0;
    if (ros::param::has("keeper_id")) {
        ros::param::get("keeper_id", keeper_id);
    }
    rtt::RobotDealer::setKeeper(keeper_id);


    // === If we're testing a tree (Strategy / Role)
    if (rtt::factories::isTree(testClass)) {
        int bot_id = -1;
        if (bb->HasInt("ROBOT_ID")) {
            bot_id = bb->GetInt("ROBOT_ID");
        }
        // Notify the tree debugger that we're running a tree. Emiel : why assume TYPE_ROLE? What about TYPE_STRATEGY?
        RTT_SEND_RQT_BT_TRACE(bot_id, testClass, roboteam_msgs::BtDebugInfo::TYPE_ROLE, roboteam_msgs::BtStatus::STARTUP, bb->toMsg());
    }
    // ===


    /* ======== Run the node ======== */
    node->Initialize();

    bt::Node::Status status = bt::Node::Status::Invalid;
    while(!mustShutdown){
        ros::spinOnce();
        fps.sleep();

        status = node->Update();
        if(status != bt::Node::Status::Running)
            break;
    }
    std::string statusString = "Unknown";
    if(status == bt::Node::Status::Running) statusString = "Running";
    if(status == bt::Node::Status::Success) statusString = "Success";
    if(status == bt::Node::Status::Failure) statusString = "Failure";
    if(status == bt::Node::Status::Invalid) statusString = "Invalid";

    node->Terminate(status);
    /* ============================== */

    // Give ros some time to send the stop messages
    ros::Rate rate(60);
    for (int i = 0; i < 15; i++) {
        rate.sleep();
        ros::spinOnce();
    }

    // Gracefully close all the publishers.
    n.shutdown();

    ROS_INFO_STREAM_NAMED("TestX", "Completed '" << testClass << "' with status '" << statusString << "'");

	return 0;
}
