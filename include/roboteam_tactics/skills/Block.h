#pragma once

#include <map>

#include "ros/ros.h"

#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_tactics/skills/AvoidRobots.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"

namespace rtt {

#define BLOCK_BALL_ID 987654
    
using Vector = roboteam_utils::Vector2;
using Position = roboteam_utils::Position;

enum class BlockType { RELATIVE, CIRCLE, COVER, GOALAREA };

const std::map<BlockType, const char*> block_type_names {
    { BlockType::RELATIVE, "RELATIVE" },
    { BlockType::CIRCLE, "CIRCLE" },
    { BlockType::COVER, "COVER" },
    { BlockType::GOALAREA, "GOALAREA" }
};

/**
 * @class BlockPos
 * @brief Determines where to position a blocking robot.
 */
class BlockPos {
public:
    /**
     * @param current The blocking robot's current position.
     * @param opponent The location of the opponent.
     * @param to_block The location to block the opponent from.
     */
    virtual Position block_pos(const Position& current,
                             const Vector& opponent,
                             const Vector& to_block) const = 0;
    virtual ~BlockPos() {}
};

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
    std::unique_ptr<AvoidRobots> avoidBots;
 
    unsigned int my_id, tgt_id, block_id;
    bool constant, invert;
};


/**
 * @class RelativeBlock
 * @brief Blocks at the point a given distance between the opponent and target, facing the opponent.
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
 * @class CircleBlock
 * @brief Blocks at a fixed distance from the target, facing the opponent.
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
 * @class GoalareaBlock
 * @brief Blocks at the line surrounding the goal area
 */

class GoalareaBlock : public BlockPos {
public:
    GoalareaBlock(string block_arg) : block_arg(block_arg), fallback(RelativeBlock(.5)) {}
    Position block_pos(const Position& current,
                             const Vector& opponent,
                             const Vector& to_block) const {
        
        ROS_INFO("block arg was: %s",block_arg.c_str());

        Vector2 point(current.x,current.y);;    
        
        float safetyMarginGoalAreas=0.0;
        roboteam_utils::Vector2 newTargetPos = to_block;
    
        std::string our_side;
		ros::param::get("our_side", our_side);

		roboteam_utils::Vector2 distToOurDefenseArea = getDistToDefenseArea("our defense area", to_block, safetyMarginGoalAreas);
		roboteam_utils::Vector2 distToTheirDefenseArea = getDistToDefenseArea("their defense area", to_block, safetyMarginGoalAreas);
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
    string block_arg;
    RelativeBlock fallback;
};

/**
 * @class CloseCover
 * @brief Blocks very close to the opponent, facing the target.
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
