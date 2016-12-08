#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/Math.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/FieldCircularArc.h"
#include "roboteam_msgs/FieldLineSegment.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG DistanceXToY

namespace rtt {

using namespace roboteam_utils;
using namespace roboteam_msgs;


Vector2 getPointOfInterest(std::string name) {
    if (name.empty()) {
        ROS_ERROR("getPointOfInterest: name was empty");
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

    std::string our_field_side;
    get_PARAM_OUR_SIDE(our_field_side);

    double safetyMarginLine = safetyMargin;

    GeometryFieldSize field = LastWorld::get_field();
    if ((name == "our defense area" && our_field_side == "right") || (name == "their defense area" && our_field_side == "left")) {
        line = field.right_penalty_line;
        top_arc = field.top_right_penalty_arc;
        bottom_arc = field.bottom_right_penalty_arc;
        safetyMarginLine = safetyMarginLine * -1; // on the rights side of the field we need to subtract the safety margin instead of add it.
    } else if ((name == "our defense area" && our_field_side == "left") || (name == "their defense area" && our_field_side == "right")) {
        line = field.left_penalty_line;
        top_arc = field.top_left_penalty_arc;
        bottom_arc = field.bottom_left_penalty_arc;
    }

    Vector2 distToLine = distPointToLine(line, point, safetyMarginLine);
    Vector2 distToTopArc = distPointToArc(top_arc, point, safetyMargin);
    Vector2 distToBottomArc = distPointToArc(bottom_arc, point, safetyMargin);

    Vector2 shortestDistance(100.0, 100.0);
    if (distToLine.length() < shortestDistance.length()) {
        shortestDistance = distToLine;
    }
    if (distToTopArc.length() < shortestDistance.length()) {
        shortestDistance = distToTopArc;
    }
    if (distToBottomArc.length() < shortestDistance.length()) {
        shortestDistance = distToBottomArc;
    }
    return shortestDistance;
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

    RTT_DEBUGLN("Checking distance from %s to %s...", X.c_str(), Y.c_str());

    Vector2 vecX;
    Vector2 vecY;

    if (X == "me") {
        vecX = getPointOfInterest(std::to_string(ROBOT_ID));
    } else {
        vecX = getPointOfInterest(X);
    }

    if (Y == "me") {
        vecY = getPointOfInterest(std::to_string(ROBOT_ID));
    } else {
        vecY = getPointOfInterest(Y);
    }

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

    if (result) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

} // rtt
