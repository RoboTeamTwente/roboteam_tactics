#include "roboteam_tactics/treegen/LeafRegister.h"
#include <exception>
#include <cmath>
#include <sstream>

#include "roboteam_tactics/skills/Block.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_tactics/utils/ScopedBB.h"

namespace rtt {

RTT_REGISTER_SKILL(Block);

Block::Block(std::string name, bt::Blackboard::Ptr bb) : Skill(name, bb) {
    //assert_valid<Block>(name);
}

void normalize(Position& pos) {
    if (pos.x > 6)
        pos.x = 6;
    if (pos.x < -6)
        pos.x = -6;
    if (pos.y > 4)
        pos.y = 4;
    if (pos.y < -4)
        pos.y = -4;
}

bt::Node::Status Block::Update() {
    
    std::string type = GetString("block_type");
    if (type == block_type_names.at(BlockType::RELATIVE)) {
        pos = new RelativeBlock(GetDouble("block_arg"));
    } else if (type ==  block_type_names.at(BlockType::CIRCLE)) {
        pos = new CircleBlock(GetDouble("block_arg"));
    } else if (type ==  block_type_names.at(BlockType::GOALAREA)) {
        pos = new GoalareaBlock(GetString("block_arg"));
    }else if (type ==  block_type_names.at(BlockType::COVER)) {
        pos = new CloseCover();
    } else {
        char err[30];
        sprintf(err, "Invalid block_type: %s\n", GetString("block_type").c_str());
        ROS_ERROR("%s", err);
        throw std::invalid_argument(err);
    }
    
    roboteam_msgs::WorldRobot me, tgt;

    {
        auto maybeMe = getWorldBot(GetInt("ROBOT_ID"));
        auto maybeTgt = getWorldBot(GetInt("TGT_ID"), HasBool("selfBlock") && GetBool("selfBlock"));
        if (!maybeMe || !maybeTgt) return Status::Failure;

        me = *maybeMe;
        tgt = *maybeTgt;
    }
    
    Position mypos(me.pos.x, me.pos.y, me.angle);
    Position tgtpos(tgt.pos.x, tgt.pos.y, tgt.angle);
    Vector block;

    if (GetInt("BLOCK_ID") == BLOCK_BALL_ID) {
        block = Vector(LastWorld::get().ball.pos);
    } else if (HasDouble("block_x") && HasDouble("block_y")) {
        block = Vector(GetDouble("block_x"), GetDouble("block_y"));
    } else {
        roboteam_msgs::WorldRobot blk;
        {
            auto maybeBlk = getWorldBot(GetInt("BLOCK_ID"), false);
            if (!maybeBlk) return Status::Failure;
            blk = *maybeBlk;
        }
        block = Vector(blk.pos);
    }

    if (block.dist(tgtpos.location()) < .4) return bt::Node::Status::Failure;

    Position goal = pos->block_pos(mypos, tgtpos.location(), block);
    normalize(goal);
    if (GetBool("invert_direction"))
        goal.rot -= M_PI;
    //if (mypos.location().dist(goal.location()) < .1) return bt::Node::Status::Running;

    if (!goal.real()) return bt::Node::Status::Running;

    ScopedBB(*blackboard, "")
        .setDouble("xGoal", goal.x)
        .setDouble("yGoal", goal.y)
        .setDouble("angleGoal", goal.rot)
        .setBool("dribbler", false)
        .setBool("avoidRobots", true);
/*
    private_bb->SetInt("ROBOT_ID", my_id);
    private_bb->SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
    private_bb->SetDouble("xGoal", goal.x);
    private_bb->SetDouble("yGoal", goal.y);
    private_bb->SetDouble("angleGoal", goal.rot);
    private_bb->SetBool("dribbler", false);
    private_bb->SetBool("avoidRobots", true);
*/    

    goToPos = std::make_unique<GoToPos>("", blackboard);

    //ROS_INFO("Goal: (%f, %f, %f)", private_bb->GetDouble("xGoal"), private_bb->GetDouble("yGoal"), private_bb->GetDouble("angleGoal"));
    bt::Node::Status avoid_status = goToPos->Update();
    if (avoid_status != bt::Node::Status::Running) {
        goToPos.reset();
        goToPos = std::unique_ptr<GoToPos>();
    }
    return avoid_status == bt::Node::Status::Invalid || avoid_status == bt::Node::Status::Failure ? avoid_status : bt::Node::Status::Running;
}

void Block::extra_update() {
    if (goToPos) {
        goToPos->Update();
    }
}

}
