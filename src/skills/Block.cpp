#include <exception>
#include <cmath>

#include "roboteam_tactics/skills/Block.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

Block::Block(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr bb) : Skill(n, name, bb) {
    assert_valid<Block>(name);
    std::string type = GetString("block_type");
    if (type == block_type_names.at(BlockType::RELATIVE)) {
        pos = new RelativeBlock(GetDouble("block_arg"));
    } else if (type ==  block_type_names.at(BlockType::CIRCLE)) {
        pos = new CircleBlock(GetDouble("block_arg"));
    } else if (type ==  block_type_names.at(BlockType::COVER)) {
        pos = new CloseCover();
    } else {
        char err[30];
        sprintf(err, "Invalid block_type: %s", GetString("block_type").c_str());
        ROS_ERROR("%s", err);
        throw std::invalid_argument(err);
    }
    my_id = blackboard->GetInt("ROBOT_ID");
    tgt_id = blackboard->GetInt("TGT_ID");
    block_id = blackboard->GetInt("BLOCK_ID");
    constant = block_id < 0;
    invert = blackboard->GetBool("invert_direction");
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
    if (!avoidBots) {
        roboteam_msgs::WorldRobot me, tgt;

        {
            auto maybeMe = lookup_bot(my_id, true);
            auto maybeTgt = lookup_bot(tgt_id, false);
            if (!maybeMe || !maybeTgt) return Status::Failure;

            me = *maybeMe;
            tgt = *maybeTgt;
        }
        
        Position mypos(me.pos.x, me.pos.y, me.angle);
        Position tgtpos(tgt.pos.x, tgt.pos.y, tgt.angle);
        Vector block;
            
        if (block_id == BLOCK_BALL_ID) {
            block = Vector(LastWorld::get().ball.pos);
        } else if (!constant) {
            block = Vector(GetDouble("block_x"), GetDouble("block_y"));
        } else {
            roboteam_msgs::WorldRobot blk;
            {
                auto maybeBlk = lookup_bot(block_id, false);
                if (!maybeBlk) return Status::Failure;
                blk = *maybeBlk;
            }
            block = Vector(blk.pos);
        }
        
        if (block.dist(tgtpos.location()) < .4) return bt::Node::Status::Failure;
        
        Position goal = pos->block_pos(mypos, tgtpos.location(), block);
        normalize(goal);
        if (invert)
            goal.rot -= M_PI;
        if (mypos.location().dist(goal.location()) < .1) return bt::Node::Status::Running;
        
        if (!goal.real()) return bt::Node::Status::Running;
        
        private_bb->SetInt("ROBOT_ID", my_id);
        private_bb->SetDouble("xGoal", goal.x);
        private_bb->SetDouble("yGoal", goal.y);
        private_bb->SetDouble("angleGoal", goal.rot);
        private_bb->SetBool("endPoint", true);
        private_bb->SetBool("dribbler", false);

        avoidBots = std::make_unique<AvoidRobots>(n, "", private_bb);
    }
    
    ROS_INFO("Goal: (%f, %f, %f)", private_bb->GetDouble("xGoal"), private_bb->GetDouble("yGoal"), private_bb->GetDouble("angleGoal"));
    
    bt::Node::Status gtpStatus = avoidBots->Update();
    std::string desc = describe_status(gtpStatus);
    ROS_INFO("gtpStatus=%s", desc.c_str());
    if (gtpStatus != bt::Node::Status::Running) {
        ROS_INFO("invalidated");
        avoidBots.reset();
        avoidBots = std::unique_ptr<AvoidRobots>();
    }

    return gtpStatus == bt::Node::Status::Invalid || gtpStatus == bt::Node::Status::Failure ? gtpStatus : bt::Node::Status::Running;
}
    
}
