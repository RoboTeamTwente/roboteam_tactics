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
double distPointToArc(FieldCircularArc arc, Vector2 point) {
    // Normalize the point
    point.x -= arc.center.x;
    point.y -= arc.center.y;

    double dist;

    double angle = cleanAngle(point.angle());
    double arcStart = cleanAngle(arc.a1);
    double arcStop = cleanAngle(arc.a2);

    //ROS_INFO("start: %f, stop: %f, angle: %f", arcStart, arcStop, angle);

    if ((arcStart <= arcStop && (angle >= arcStart && angle <= arcStop)) ||
        (arcStart > arcStop && (angle >= arcStart || angle <= arcStop))) {
        //ROS_INFO("In arc");
        // The point is in between the arc angles.
        dist = point.length() - arc.radius;
    } else {
        Vector2 a1 = Vector2(arc.radius, 0);
        a1 = a1.rotate(arcStart);
        Vector2 a2 = Vector2(arc.radius, 0);
        a2 = a2.rotate(arcStop);

        double dist1 = point.dist(a1);
        double dist2 = point.dist(a2);

        if (dist1 < dist2) {
            dist = dist1;
        } else {
            dist = dist2;
        }
    }

    return dist;
}

double distPointToLine(FieldLineSegment line, Vector2 point) {
    // Normalize the point.
    point.x -= line.begin.x;
    point.y -= line.begin.y;

    Vector2 end = Vector2(line.end);
    end.x -= line.begin.x;
    end.y -= line.begin.y;

    point = point.rotate(-end.angle());
    end = end.rotate(-end.angle());

    //ROS_INFO("point -- x: %f, y: %f", point.x, point.y);
    //ROS_INFO("end -- x: %f, y: %f", end.x, end.y);

    double dist;

    if (point.x < 0) {
        dist = point.dist(Vector2());
    } else if (point.x > end.x) {
        dist = point.dist(end);
    } else {
        dist = point.y - end.y;
    }

    return fabs(dist);
}


double getDistToDefenseArea(std::string name, Vector2 point) {
    FieldLineSegment front_line;
    FieldLineSegment back_line;
    FieldCircularArc top_arc;
    FieldCircularArc bottom_arc;

    GeometryFieldSize field = LastWorld::get_field();

    std::string our_side = "right";
    get_PARAM_OUR_SIDE(our_side);

    if ((name == "our defense area" && our_side == "right") ||
        (name == "their defense area" && our_side == "left")) {

        front_line = field.right_penalty_line;
        back_line = field.right_line;
        top_arc = field.top_right_penalty_arc;
        bottom_arc = field.bottom_right_penalty_arc;

    } else if ((name == "our defense area" && our_side == "left") ||
        (name == "their defense area" && our_side == "right")) {

        front_line = field.left_penalty_line;
        back_line = field.left_line;
        top_arc = field.top_left_penalty_arc;
        bottom_arc = field.bottom_left_penalty_arc;
    }

    // Make the back line as wide as the outer edges of the defense arcs.
    back_line.begin.y = -(fabs(top_arc.center.y) + top_arc.radius);
    back_line.end.y = (fabs(bottom_arc.center.y) + bottom_arc.radius);

    double top_dist = distPointToArc(top_arc, point);
    double bottom_dist = distPointToArc(bottom_arc, point);
    double front_dist = distPointToLine(front_line, point);
    double back_dist = distPointToLine(back_line, point);

    double dist;

    if (top_dist < 0 || bottom_dist < 0) {

        if (back_dist < fabs(top_dist) && back_dist < fabs(bottom_dist)) {
            dist = back_dist;
        } else if (fabs(top_dist) < fabs(bottom_dist)) {
            dist = top_dist;
        } else {
            dist = bottom_dist;
        }

        // Check wether we are in the non-arc area of the defense area.
        // (the rectangle bit in the middle)
        // This check assumes the defense and back line are parralel, and vertical.
        // TODO: Maybe there is a more elegant way? At the moment this works.
    } else if (fabs(back_line.begin.x) >= fabs(point.x) && fabs(front_line.begin.x) <= fabs(point.x)
            && ((point.y <= front_line.begin.y && point.y >= front_line.end.y) || (point.y >= front_line.begin.y && point.y <= front_line.end.y))) {

        if (back_dist < front_dist) {
            dist = -back_dist;
        } else {
            dist = -front_dist;
        }
    } else {
        // We are not inside the defense area. Return positive values.

        if (top_dist < bottom_dist && top_dist < front_dist && top_dist < back_dist) {
            dist = top_dist;
        } else if (bottom_dist < front_dist && bottom_dist < back_dist) {
            dist = bottom_dist;
        } else if (front_dist < back_dist) {
            dist = front_dist;
        } else {
            dist = back_dist;
        }
    }

    return dist;
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
        dist = getDistToDefenseArea(X, vecY);
    } else if ((Y == "our defense area" || Y == "their defense area") && X != Y) {
        dist = getDistToDefenseArea(Y, vecX);
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
