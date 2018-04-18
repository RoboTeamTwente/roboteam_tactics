#include <limits>

#include <ros/param.h>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/FieldCircularArc.h"
#include "roboteam_msgs/FieldLineSegment.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_tactics/conditions/IsBallInDefenseArea.h"


#define RTT_CURRENT_DEBUG_TAG DistanceXToY

namespace rtt {

using namespace roboteam_utils;
using namespace roboteam_msgs;

Vector2 DistanceXToY::getPointOfInterest(std::string name, int const ROBOT_ID) {
    if (name.empty()) {
        ROS_ERROR("getPointOfInterest: name was empty");
        return Vector2(0, 0);
    } else if (name == "me") {
        if (auto bot = getWorldBot(ROBOT_ID)) {
            return bot->pos;   
        }
        
        // Error!
        return Vector2(0, 0);
    } else if (name == "ball") {
        return Vector2(LastWorld::get().ball.pos);
    } else if (name == "our goal" || name == "their goal") {
        // Get the side of the field
        // std::string our_side = "right";
        // get_PARAM_OUR_SIDE(our_side);

        // Get the length of the field
        double field_length = LastWorld::get_field().field_length;

        if (name == "our goal") {
            return Vector2(field_length / -2, 0);
        } else if (name == "their goal") {
            return Vector2(field_length / 2, 0);
        }
    } else if (name == "center dot") {
        return Vector2(0, 0);
    } else if (name == "closest opponent") {
        auto const world = LastWorld::get();

        if (auto myBotOpt = getWorldBot(ROBOT_ID)) {
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
    } else if (name == "fixed point") {
    	if (!HasDouble("px") || !HasDouble("py")) {
    		ROS_ERROR("DistanceXToY (Name: %s) set to \"fixed point\" mode, but doubles px and py not specified.",
    				this->name.c_str());
    		return Vector2();
    	}
    	return { GetDouble("px"), GetDouble("py") };
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
    boost::optional<roboteam_msgs::WorldRobot> possibleBot = getWorldBot(robotID, name.back() != 'T');

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


Vector2 getDistToDefenseArea(bool ourDefenseArea, Vector2 point, double safetyMargin) {
    FieldLineSegment line;
    FieldLineSegment top_line;
    FieldLineSegment bottom_line;
    // FieldCircularArc top_arc;
    // FieldCircularArc bottom_arc;
    
    double safetyMarginLine = safetyMargin;//safetyMargin;

    GeometryFieldSize field = LastWorld::get_field();

    if (ourDefenseArea) {
        line = field.left_penalty_line;
        top_line = field.top_left_penalty_stretch;
        bottom_line = field.bottom_left_penalty_stretch;
        // top_arc = field.top_left_penalty_arc;
        // bottom_arc = field.bottom_left_penalty_arc;
    } else {
        line = field.right_penalty_line;
        top_line = field.top_right_penalty_stretch;
        bottom_line = field.bottom_right_penalty_stretch;
        // top_arc = field.top_right_penalty_arc;
        // bottom_arc = field.bottom_right_penalty_arc;
        safetyMarginLine = safetyMarginLine * -1; // on the right side of the field we need to subtract the safety margin instead of add it.
    }


    Vector2 distToLine = distPointToLine(line, point, 0.0);
    Vector2 distToTopLine = distPointToLine(top_line, point, 0.0);
    Vector2 distToBottomLine = distPointToLine(bottom_line, point, 0.0);
    // Vector2 distToTopArc = distPointToArc(top_arc, point, safetyMargin);
    // Vector2 distToBottomArc = distPointToArc(bottom_arc, point, safetyMargin);

    if(isWithinDefenseArea(ourDefenseArea, point, 0.001)) {
        // if within actual defense area -> just add margin
        distToLine.x += safetyMarginLine;
        distToTopLine.y += safetyMargin;
        distToBottomLine.y -= safetyMargin;
    } else {
        // if not -> stretch outward (i.e. add margin in opposite direction)
        distToLine = distToLine + Vector2(-safetyMargin, 0.0).rotate(distToLine.angle());
        distToTopLine = distToTopLine + Vector2(-safetyMargin, 0.0).rotate(distToTopLine.angle());
        distToBottomLine = distToBottomLine + Vector2(-safetyMargin, 0.0).rotate(distToBottomLine.angle());
    }

    Vector2 shortestDistance = distToLine;
    if (distToTopLine.length() < shortestDistance.length()) {
        shortestDistance = distToTopLine;
    }
    if (distToBottomLine.length() < shortestDistance.length()) {
        shortestDistance = distToBottomLine;
    }
    // if (distToTopArc.length() < shortestDistance.length()) {
    //     shortestDistance = distToTopArc;
    // }
    // if (distToBottomArc.length() < shortestDistance.length()) {
    //     shortestDistance = distToBottomArc;
    // }

    return shortestDistance;
}

// Written by Jelle: get distance as a double that is negative when inside defense area
double getDistToDefenseArea2(bool ourDefenseArea, Vector2 point) { 
    FieldLineSegment line;
    FieldLineSegment top_line;
    FieldLineSegment bottom_line;
    // FieldCircularArc top_arc;
    // FieldCircularArc bottom_arc;

    GeometryFieldSize field = LastWorld::get_field();

    if (ourDefenseArea) {
        line = field.left_penalty_line;
        top_line = field.top_left_penalty_stretch;
        bottom_line = field.bottom_left_penalty_stretch;
        // top_arc = field.top_left_penalty_arc;
        // bottom_arc = field.bottom_left_penalty_arc;
    } else {
        line = field.right_penalty_line;
        top_line = field.top_right_penalty_stretch;
        bottom_line = field.bottom_right_penalty_stretch;
        // top_arc = field.top_right_penalty_arc;
        // bottom_arc = field.bottom_right_penalty_arc;
    }

    Vector2 distToLine = distPointToLine(line, point, 0.0);
    Vector2 distToTopLine = distPointToLine(top_line, point, 0.0);
    Vector2 distToBottomLine = distPointToLine(bottom_line, point, 0.0);

    // drawer.setColor(0, 100, 100);
    // drawer.drawLine("distToBottomLine", point, distToBottomLine);
    // drawer.setColor(100, 100, 0);
    // drawer.drawLine("distToTopLine", point, distToTopLine);
    // Vector2 distToTopArc = distPointToArc(top_arc, point, safetyMargin);
    // Vector2 distToBottomArc = distPointToArc(bottom_arc, point, safetyMargin);

    double shortestDistance = distToLine.length() * -signum(distToLine.x);
    if (!ourDefenseArea) {
        shortestDistance = -shortestDistance;
    }
    if (distToTopLine.length() < fabs(shortestDistance)) {
        shortestDistance = distToTopLine.length() * -signum(distToTopLine.y);
    }
    if (distToBottomLine.length() < fabs(shortestDistance)) {
        shortestDistance = distToBottomLine.length() * signum(distToBottomLine.y);
    }
    // if (ourDefenseArea) {
    //     // For the arcs the sign of the distance is determined from the angle of the distance vector.
    //     if (distToTopArc.length() < fabs(shortestDistance)) {
    //         shortestDistance = distToTopArc.length() * signum(cleanAngle(distToTopArc.angle() - 0.75*M_PI));
    //     }
    //     if (distToBottomArc.length() < fabs(shortestDistance)) {
    //         shortestDistance = distToBottomArc.length() * signum(cleanAngle(distToBottomArc.angle() - 0.25*M_PI));
    //     }
    // } else { // On their defense area (right side)
    //     shortestDistance = -shortestDistance;
    //     if (distToTopArc.length() < fabs(shortestDistance)) {
    //         shortestDistance = distToTopArc.length() * signum(cleanAngle(distToTopArc.angle() + 0.75*M_PI));
    //     }
    //     if (distToBottomArc.length() < fabs(shortestDistance)) {
    //         shortestDistance = distToBottomArc.length() * signum(cleanAngle(distToBottomArc.angle() + 0.25*M_PI));
    //     }
    // }

    
    return shortestDistance;
}

// bool isWithinDefenseArea(std::string whichArea, Vector2 point) {
//     GeometryFieldSize field = LastWorld::get_field();
//     Vector2 distToDefenseArea = getDistToDefenseArea(whichArea, point, 0.0);
//     if (whichArea == "our defense area") {
//         if (distToDefenseArea.x > 0.0 && point.x >= -field.field_length/2) return true;
//         else return false;
//     } else if (whichArea == "their defense area") {
//         if (distToDefenseArea.x < 0.0 && point.x <= field.field_length/2) return true;
//         else return false;
//     }
//     ROS_WARN("DistanceXToY/isWithinDefenseArea you probably entered a wrong name");
//     return false;
// }

// bool isWithinDefenseArea(std::string whichArea, Vector2 point, std::string our_side, roboteam_msgs::GeometryFieldSize field) {

//     if (whichArea == "our defense area" || whichArea == "both") {
//         if ((Vector2(field.top_left_penalty_arc.center) - point).length() < field.top_left_penalty_arc.radius) return true;
//         else if ((Vector2(field.bottom_left_penalty_arc.center) - point).length() < field.bottom_left_penalty_arc.radius) return true;
//         else if (point.x < field.left_penalty_line.begin.x && fabs(point.y) < field.left_penalty_line.begin.x) return true;
//     }

//     if (whichArea == "their defense area" || whichArea == "both") {

//         if ((Vector2(field.top_right_penalty_arc.center) - point).length() < field.top_right_penalty_arc.radius) return true;
//         else if ((Vector2(field.bottom_right_penalty_arc.center) - point).length() < field.bottom_right_penalty_arc.radius) return true;
//         else if (point.x > field.right_penalty_line.begin.x && fabs(point.y) < field.right_penalty_line.begin.x) return true;
//     }

//     return false;
// }

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

    RTT_DEBUGLN("Check if %s -> %s %s %f", X.c_str(), Y.c_str(), mode.c_str(), checkDistance);

    auto vecX = getPointOfInterest(X, ROBOT_ID);
    auto vecY = getPointOfInterest(Y, ROBOT_ID);
    double dist;

    if (X == "our defense area" && X != Y) {
        dist = getDistToDefenseArea(true, vecY, 0.2).length();
    } else if (X == "their defense area" && X != Y) {
        dist = getDistToDefenseArea(false, vecY, 0.2).length();
    } else if (Y == "our defense area" && X != Y) {
        dist = getDistToDefenseArea(true, vecX, 0.2).length();
    } else if (Y == "their defense area" && X != Y) {
        dist = getDistToDefenseArea(false, vecX, 0.2).length();
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
