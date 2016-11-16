#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

using namespace roboteam_utils;

Vector2 getPointOfInterest(std::string name) {
    if (name.empty()) {
        ROS_ERROR("getPointOfInterest: name was empty");
        return Vector2(0, 0);
    } else if (name == "ball") {
        return Vector2(LastWorld::get().ball.pos);
    } else if (name == "our goal" || name == "their goal") {
        // Get the side of the field
        std::string our_side = "right";
        ros::param::get("our_side", our_side);
        
        // Get the length of the field
        double field_length = LastWorld::get_field().field_length;
        
        // If we want their goal, set the modifier to -1
        // This flips the point to the other side
        int mod = 1;
        if (name == "their goal") {
            mod = -1;
        }

        if (our_side == "right") {
            return Vector2(field_length / 2 / mod, 0);
        } else {
            return Vector2(field_length / -2 / mod, 0);
        }
    } else if (name == "center dot") {
        return Vector2(0, 0);
    } 

    // We're looking for a robot!
    // Try to parse the id. If it fails, return the zero vector
    int robotID = 0;
    try {
        robotID = std::stoi(name);
    } catch (const std::invalid_argument &e) {
        ROS_ERROR_STREAM("name passed to getPointOfInterest is not a valid robot id! Robot id passed: "
                << name
                << "\n");
        return Vector2(0, 0);
    } catch (const std::out_of_range &e) {
        ROS_ERROR_STREAM("name passed to getPointOfInterest is not a valid robot id! Robot id passed: "
                << name
                << "\n");
        return Vector2(0, 0);
    }

    // Try to find a bot based on the suffix
    boost::optional<roboteam_msgs::WorldRobot> possibleBot;
    if (name.back() == 'T') {
        possibleBot = lookup_bot(robotID, false);
    } else {
        possibleBot = lookup_bot(robotID, true);
    }

    // If it's not there, return the zero vector
    if (!possibleBot) {
        ROS_ERROR_STREAM("Bot " << robotID << " was not found in LastWorld in getPointOfInterest.");
        return Vector2(0, 0);
    }

    // Otherwise return the bot position
    return Vector2(possibleBot->pos);
}

DistanceXToY::DistanceXToY(std::string name, bt::Blackboard::Ptr blackboard)
        : Condition(name, blackboard) {
            std::cout << "Received bb:\n";
            rtt::print_blackboard(blackboard);
        }

bt::Node::Status DistanceXToY::Update() {
	roboteam_msgs::World world = LastWorld::get();
    
    const int ROBOT_ID = blackboard->GetInt("ROBOT_ID");
    const double checkDistance = GetDouble("distance");
    const std::string mode = GetString("mode");
    const std::string X = GetString("X");
    const std::string Y = GetString("Y");

    // ROS_INFO_STREAM("DistanceXToY update. X " << X << " Y " << Y << ". ROBOT_ID: " << blackboard->GetInt("ROBOT_ID"));

    Vector2 vecX;
    if (X == "me") {
        vecX = getPointOfInterest(std::to_string(ROBOT_ID));
    } else {
        vecX = getPointOfInterest(X);
    }

    Vector2 vecY;
    if (Y == "me") {
        vecY = getPointOfInterest(std::to_string(ROBOT_ID));
    } else {
        vecY = getPointOfInterest(Y);
    }

    double dist = vecX.dist(vecY);
    bool result = false;

    if (mode == "lt") {
        result = dist < checkDistance;
    } else if (mode == "gt") {
        result = dist > checkDistance;
    } else if (mode == "eq") {
        result = dist == checkDistance;
    } else if (mode == "leq") {
        result = dist <= checkDistance;
    } else if (mode == "geq") {
        result = dist >= checkDistance;
    } else {
        ROS_ERROR_STREAM("Unknown mode given to DistanceXToY: " << mode << ". Name: " << name);
        return Status::Failure;
    }

    if (result) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

} // rtt
