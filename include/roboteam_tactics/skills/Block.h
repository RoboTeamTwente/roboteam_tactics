#pragma once

#include <map>

#include "ros/ros.h"

#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"

namespace rtt {

#define BLOCK_BALL_ID 987654
    
using Vector = Vector2;
using Position = Position;

enum class BlockType { RELATIVE, ABSOLUTE, CIRCLE, COVER, GOALAREA };

const std::map<BlockType, const char*> block_type_names {
    { BlockType::RELATIVE, "RELATIVE" },
    { BlockType::ABSOLUTE, "ABSOLUTE" },
    { BlockType::CIRCLE, "CIRCLE" },
    { BlockType::COVER, "COVER" },
    { BlockType::GOALAREA, "GOALAREA" }
};

class BlockPos {
public:
    virtual Position block_pos(const Position& current,
                             const Vector& opponent,
                             const Vector& to_block) const = 0;
    virtual ~BlockPos() {}
};

/**
 * \class Block
 * \brief See YAML
 */
/*
 * Descr: Take up position between a robot and some point, determined by parameters.
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the blocking robot
 *   - TGT_ID:
 *       Type: Int
 *       Descr: The id of the opponent robot to block
 *   - BLOCK_ID:
 *       Type: Int
 *       Can be:
 *         - BLOCK_BALL_ID: (=987654) To stand between TGT_ID and the ball
 *         - Any positive integer: To stand between TGT_ID and the opponent 
 *       Used when: block_x and block_y are not set
 *       Descr: When the block target is not an absolute location, this number indicates what to block
 *   - block_x:
 *       Type: Double
 *       Used when: BLOCK_ID != BLOCK_BALL_ID
 *       Descr: The x-coordinate of the location to block the target from
 *   - block_y:
 *       Type: Double
 *       Used when: BLOCK_ID != BLOCK_BALL_ID
 *       Descr: The y-coordinate of the location to block the target from
 *   - block_type:
 *       Type: String
 *       Can be:
 *         - RELATIVE: >
 *             Block at a position a certain fraction (given by block_arg) between the two points.
 *             For example, when block_arg is 0.5, the robot will stand halfway between the opponent
 *             and the point it should block. By default, the robot will face the opponent.
 *         - CIRCLE: >
 *             Blocks along a circle centered around the opponent, with a radius given by block_arg.
 *             By default, the robot will face the opponent.
 *         - COVER: >
 *             Blocks close to the opponent, trying to keep it from receiving the ball.
 *             By default, the robot will face the location it is blocking.
 *         - GOALAREA: >
 *             Blocks along the line surrounding the goal. By default, the robot will face
 *             away from the goal.
 *       Used when: block_x and block_y are not set.
 *       Descr: Determines how the actual block position is calculated.
 *   - block_arg:
 *        Type: Double
 *        Used when: block_type is used and is either RELATIVE or CIRCLE
 *        Descr: Parameter for the block_type. See its documentation for details.
 *   - invert_direction:
 *        Type: Bool
 *        Used when: block_type is used
 *        Descr: When true, the robot will face the direction opposite to the one it normally would given the block_type.
 *   - selfBlock:
 *        Type: Bool
 *        Default: false
 *        Descr: When true, TGT_ID is considered to be one of our robots, rather than an opponent. For testing.
 *             
 */
class Block : public Skill {
public:
    Block(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() override;
    void extra_update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["TGT_ID"] = BBArgumentType::Int;
        params["BLOCK_ID"] = BBArgumentType::Int;
        //params["block_x"] = BBArgumentType::Double;
        //params["block_y"] = BBArgumentType::Double;
        params["block_type"] = BBArgumentType::String; // must be an element of BlockType
        //params["block_arg"] = BBArgumentType::Double; // ignored for CloseCover
        params["invert_direction"] = BBArgumentType::Bool;
        return params;
    }
    std::string node_name() override { return "Block"; }
private:
    BlockPos* pos;
    std::unique_ptr<GoToPos> goToPos;
};


/**
 * \class RelativeBlock
 * \brief Blocks at the point a given distance between the opponent and target, facing the opponent.
 * For example, if the opponent is at (0, 0), the target at (0, 10) and the factor 0.4, then the block_pos
 * will be (0, 6).
 */
class RelativeBlock : public BlockPos {
public:
    RelativeBlock(double factor) : factor(factor) {}
    Position block_pos(const Position& current,
                             const Vector& opponent,
                             const Vector& to_block) const override {
        Vector norm = opponent - to_block;
        double angle = norm.angle();
        norm = norm.scale(factor) + to_block;
        return Position(norm.x, norm.y, angle);
    }
private:
    double factor;
};

/**
 * \class AbsoluteBlock
 * \brief Blocks at the point a given distance between the opponent and target, facing the opponent.
 * For example, if the opponent is at (0, 0), the target at (0, 10) and the argument 1, then the block_pos
 * will be (0, 1).
 */
class AbsoluteBlock : public BlockPos {
public:
    AbsoluteBlock(double distance) : distance(distance) {}
    Position block_pos(const Position& current,
                             const Vector& opponent,
                             const Vector& to_block) const override {
        Vector norm = opponent-to_block;
        double angle = norm.angle();
        norm = norm.scale(1/norm.length())*-distance + opponent;
        return Position(norm.x, norm.y, angle);
    }
private:
    double distance;
};

/**
 * \class CircleBlock
 * \brief Blocks at a fixed distance from the target, facing the opponent.
 */
class CircleBlock : public BlockPos {
public:
    CircleBlock(double radius) : radius(radius), fallback(RelativeBlock(.5)) {}
    Position block_pos(const Position& current,
                             const Vector& opponent,
                             const Vector& to_block) const {
        Vector norm = opponent - to_block;
        double dist = norm.length();
        double angle = norm.angle();
        if (dist < radius) {
            return fallback.block_pos(current, opponent, to_block);
        }
        double scalar = radius / dist;
        norm = norm.scale(scalar) + to_block;
        return Position(norm.x, norm.y, angle);
    }  
private:
    double radius;
    RelativeBlock fallback;
};

/**
 * \class GoalareaBlock
 * \brief Blocks at the line surrounding the goal area
 */

class GoalareaBlock : public BlockPos {
public:
    GoalareaBlock(std::string block_arg) : block_arg(block_arg), fallback(RelativeBlock(.5)) {}
    Position block_pos(const Position& current,
                             const Vector& opponent,
                             const Vector& to_block) const {
        
        ROS_INFO("block arg was: %s",block_arg.c_str());

        Vector2 point(current.x,current.y);;    
        
        float safetyMarginGoalAreas=0.0;
        Vector2 newTargetPos = to_block;
    
        std::string our_side;
		ros::param::get("our_side", our_side);

		Vector2 distToOurDefenseArea = getDistToDefenseArea(true, to_block, safetyMarginGoalAreas);
		Vector2 distToTheirDefenseArea = getDistToDefenseArea(false, to_block, safetyMarginGoalAreas);
		ROS_INFO("distToOurDefensArea: x:%f, y:%f",distToOurDefenseArea.x,distToOurDefenseArea.y);
		
		Vector togoalarea;
        if ((newTargetPos.x < 0.0 && our_side == "left") || 
        	(newTargetPos.x > 0.0 && our_side == "right")) {
            togoalarea= distToOurDefenseArea;
        } else {
            togoalarea= distToTheirDefenseArea;
	  	}
	  	
	  	newTargetPos = newTargetPos + togoalarea;
	  	Vector fromgoalarea=togoalarea.scale(-1); // point vector the other way;
	  	
	  	
		Position targetpospos(newTargetPos.x, newTargetPos.y, fromgoalarea.angle());
		ROS_INFO("new target: x: %f, y: %f",targetpospos.x,targetpospos.y);
        return targetpospos;
    }  
private:
    std::string block_arg;
    RelativeBlock fallback;
};

/**
 * \class CloseCover
 * \brief Blocks very close to the opponent, facing the target.
 */
class CloseCover : public BlockPos {
public:
    Position block_pos(const Position& current,
                             const Vector& opponent,
                             const Vector& to_block) const {
        CircleBlock impl(.45); // +/- 2 * bot radius + 5 cm
        return impl.block_pos(current, to_block, opponent);
    }  
};

}
