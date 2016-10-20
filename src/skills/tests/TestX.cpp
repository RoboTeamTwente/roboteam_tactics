#include "ros/ros.h"

#include "roboteam_tactics/generated/allskills_factory.h"

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
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "TestX");
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
        auto typeSplit = split(arguments.at(i), ':');
        auto argType = typeSplit.at(0);
        auto nameSplit = split(typeSplit.at(1), '=');
        auto name = nameSplit.at(0);
        auto rest = nameSplit.at(1);
        
        
        // Aggregate all the splitted = into one string
        // This happens if you try to set a value that contains multiple equals
        // (Then you only want to split on the first)
        for (size_t i = 2; i < nameSplit.size(); i++) {
            rest += nameSplit.at(i);
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

    auto skill = rtt::make_skill(n, testClass, "", bb);
    ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackGoToPos, _1));

    while (ros::ok()) {
        ros::spinOnce();
        if (skill->Update() == bt::Node::Status::Success) {
            break;
        }
    }

    std::cout << "Test of " << testClass << " completed!\n";
	return 0;
}
