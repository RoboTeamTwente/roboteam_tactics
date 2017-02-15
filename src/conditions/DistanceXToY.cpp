#include <limits>

#include <ros/param.h>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/FieldCircularArc.h"
#include "roboteam_msgs/FieldLineSegment.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"

#define RTT_CURRENT_DEBUG_TAG DistanceXToY

namespace rtt {

using namespace roboteam_utils;
using namespace roboteam_msgs;

Vector2 getPointOfInterest(std::string name, int const ROBOT_ID) {
    if (name.empty()) {
        ROS_ERROR("getPointOfInterest: name was empty");
        return Vector2(0, 0);
    } else if (name == "me") {
        if (auto bot = lookup_our_bot(ROBOT_ID)) {
            return bot->pos;   
        }
        
        // Error!
        return Vector2(0, 0);
    } else if (name == "ball") {
        return Vector2(LastWorld::get().ball.pos);
    } else if (name == "our goal" || name == "their goal") {
        // Get the side of the field
        std::string our_side = "right";
        get_PARAM_OUR_SIDE(our_side);

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
    } else if (name == "closest opponent") {
        auto const world = LastWorld::get();

        if (auto myBotOpt = lookup_bot(ROBOT_ID, true, &world)) {
            double minDist = std::numeric_limits<double>::max();
            Vector2 minPos(0, 0);

            auto myBot = *myBotOpt;
            Vector2 myPos = myBot.pos;

            for (auto const & robot : world.them) {
                double dist = Vector2(robot.pos).dist2(myPos);
                if (dist < minDist) {
                    minDist = dist;
                    minPos = robot.pos;
                }
            }

            return minPos;
        }
        
        // Error! myBot could not be found!
        return Vector2(0, 0);
    }

    // See if we are looking for a robot.
    // Try to parse the id. If it fails, return the zero vector
    int robotID = 0;
    try {
        robotID = std::stoi(name);
    } catch (const std::invalid_argument &e) {
        return Vector2(0, 0);
    } catch (const std::out_of_range &e) {
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

/*
 * Calculates the distance between an arc and a point.
 * Gives a negative distance when the point is inside the arc.
 */
Vector2 distPointToArc(FieldCircularArc arc, Vector2 point, double safetyMargin) {
    Vector2 centerToPoint(point.x - arc.center.x, point.y - arc.center.y);
    Vector2 closestPointOnArc;
    double a1 = cleanAngle(arc.a1);
    double a2 = cleanAngle(arc.a2);

    if (isBetweenAngles(a1, a2, centerToPoint.angle())) {
        closestPointOnArc = centerToPoint.scale((arc.radius + safetyMargin) / centerToPoint.length()) + Vector2(arc.center);
    } else {
        double clockwiseAngle = getClockwiseAngle(a1, centerToPoint.angle());
        double counterClockwiseAngle = getCounterClockwiseAngle(a2, centerToPoint.angle());
        if (clockwiseAngle <= counterClockwiseAngle) {
            closestPointOnArc = Vector2(arc.center) + Vector2(arc.radius + safetyMargin, 0.0).rotate(a1);
        } else {
            closestPointOnArc = Vector2(arc.center) + Vector2(arc.radius + safetyMargin, 0.0).rotate(a2);
        }
    }
    Vector2 posDiff = closestPointOnArc - point;
    return posDiff;
}

Vector2 distPointToLine(FieldLineSegment line, Vector2 point, double safetyMarginLine) {
    Vector2 vector(line.end.x - line.begin.x, line.end.y - line.begin.y);
    Vector2 start(line.begin.x, line.begin.y);
    start.x = start.x + safetyMarginLine;

    Vector2 closestPointOnVector = vector.closestPointOnVector(start, point);
    Vector2 posDiff = closestPointOnVector - point;
    return posDiff;
}


Vector2 getDistToDefenseArea(std::string name, Vector2 point, double safetyMargin) {
    FieldLineSegment line;
    FieldCircularArc top_arc;
    FieldCircularArc bottom_arc;

    std::string our_side = get_our_side();

    double safetyMarginLine = safetyMargin;

    GeometryFieldSize field = LastWorld::get_field();
    if ((name == "our defense area" && our_side == "right") || (name == "their defense area" && our_side == "left")) {
        line = field.right_penalty_line;
        top_arc = field.top_right_penalty_arc;
        bottom_arc = field.bottom_right_penalty_arc;
        safetyMarginLine = safetyMarginLine * -1; // on the right side of the field we need to subtract the safety margin instead of add it.
    } else if ((name == "our defense area" && our_side == "left") || (name == "their defense area" && our_side == "right")) {
        line = field.left_penalty_line;
        top_arc = field.top_left_penalty_arc;
        bottom_arc = field.bottom_left_penalty_arc;
    }

    Vector2 distToLine = distPointToLine(line, point, safetyMarginLine);
    Vector2 distToTopArc = distPointToArc(top_arc, point, safetyMargin);
    Vector2 distToBottomArc = distPointToArc(bottom_arc, point, safetyMargin);

    Vector2 shortestDistance = distToLine;
    if (distToTopArc.length() < shortestDistance.length()) {
        shortestDistance = distToTopArc;
    }
    if (distToBottomArc.length() < shortestDistance.length()) {
        shortestDistance = distToBottomArc;
    }

    return shortestDistance;
}

bool isWithinDefenseArea(std::string whichArea, Vector2 point) {
    GeometryFieldSize field = LastWorld::get_field();
    std::string our_side;
    ros::param::get("our_side", our_side);
    Vector2 distToDefenseArea = getDistToDefenseArea(whichArea, point, 0.0);
    if (whichArea == "our defense area") {
        if (our_side == "left") {
            if (distToDefenseArea.x > 0.0 && point.x >= -field.field_length/2) return true;
            else return false;
        }
        if (our_side == "right") {
            if (distToDefenseArea.x < 0.0 && point.x <= field.field_length/2) return true;
            else return false;
        }
    }
    if (whichArea == "their defense area") {
        if (our_side == "left") {
            if (distToDefenseArea.x < 0.0 && point.x <= field.field_length/2) return true;
            else return false;
        }
        if (our_side == "right") {
            if (distToDefenseArea.x > 0.0 && point.x >= -field.field_length/2) return true;
            else return false;
        }
    }
    ROS_WARN("DistanceXToY/isWithinDefenseArea you probably entered a wrong name");
    return false;
}

double getDistToSide(std::string name, Vector2 point, double marginOutsideField) {
    GeometryFieldSize field = LastWorld::get_field();
    if (name == "top") {
        return (field.field_width/2 + marginOutsideField - point.y);
    }
    if (name == "bottom") {
        return (-(field.field_width/2 + marginOutsideField) - point.y);
    }
    if (name == "left") {
        return (-(field.field_length/2 + marginOutsideField) - point.x);
    }
    if (name == "right") {
        return (field.field_length/2 + marginOutsideField - point.x);
    }
    ROS_WARN("getDistToSide: no valid name given");
    return 0.0;
}


RTT_REGISTER_CONDITION(DistanceXToY);

DistanceXToY::DistanceXToY(std::string name, bt::Blackboard::Ptr blackboard)
        : Condition(name, blackboard) { }

bt::Node::Status DistanceXToY::Update() {
	roboteam_msgs::World world = LastWorld::get();

    const int ROBOT_ID = blackboard->GetInt("ROBOT_ID");
    const double checkDistance = GetDouble("distance");
    const std::string mode = GetString("mode");
    const std::string X = GetString("X");
    const std::string Y = GetString("Y");

    RTT_DEBUGLN("Check if %s -> %s %s %f\n", X.c_str(), Y.c_str(), mode.c_str(), checkDistance);

    auto vecX = getPointOfInterest(X, ROBOT_ID);
    auto vecY = getPointOfInterest(Y, ROBOT_ID);

    double dist;

    if ((X == "our defense area" || X == "their defense area") && X != Y) {
        dist = getDistToDefenseArea(X, vecY, 0.2).length();
    } else if ((Y == "our defense area" || Y == "their defense area") && X != Y) {
        dist = getDistToDefenseArea(Y, vecX, 0.2).length();
    } else {
        dist = vecX.dist(vecY);
    }


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

    RTT_DEBUGLN("%s => Dist: %f, result: %d", name.c_str(), dist, result);

    if (result) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

} // rtt
