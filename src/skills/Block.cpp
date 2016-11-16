#include <exception>
#include <cmath>

#include "roboteam_tactics/skills/Block.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"

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
    gtp_valid = false;
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
    if (!gtp_valid) {
        roboteam_msgs::World world = LastWorld::get();
        roboteam_msgs::WorldRobot *me = nullptr, *tgt = nullptr;
        for (auto& bot : world.us) {
            if (bot.id == my_id) {
                me = &bot;
                break;
            }
        }    
        for (auto& bot : world.them) {
            if (bot.id == tgt_id) {
                tgt = &bot;
                break;
            }
        }    
        if (me == nullptr || tgt == nullptr) return bt::Node::Status::Invalid;
        
        Position mypos(me->pos.x, me->pos.y, me->angle);
        Position tgtpos(tgt->pos.x, tgt->pos.y, tgt->angle);
        Vector block;
            
        if (block_id == BLOCK_BALL_ID) {
            block = Vector(world.ball.pos.x, world.ball.pos.y);
        } else if (!constant) {
            block = Vector(GetDouble("block_x"), GetDouble("block_y"));
        } else {
            roboteam_msgs::WorldRobot* blk = nullptr;
            for (auto bot : world.them) {
                if (bot.id == block_id) {
                    blk = &bot;
                    break;
                }
            }
            if (blk == nullptr) return bt::Node::Status::Invalid;
            block = Vector(blk->pos.x, blk->pos.y);
        }
        
        if (block.dist(tgtpos.location()) < .4) return bt::Node::Status::Failure;
        
        Position goal = pos->block_pos(mypos, tgtpos.location(), block);
        normalize(goal);
        if (invert)
            goal.rot -= M_PI;
        if (mypos.location().dist(goal.location()) < .1) return bt::Node::Status::Running;
        
        // ROS_INFO("mypos=(%f, %f, %f), tgtpos=(%f, %f, %f), block=(%f, %f), goal=(%f, %f, %f)",
            // mypos.x, mypos.y, mypos.rot, tgtpos.x, tgtpos.y, tgtpos.rot,
            // block.x, block.y, goal.x, goal.y, goal.rot);
        
        if (!goal.real()) return bt::Node::Status::Running;
        
        private_bb->SetInt("ROBOT_ID", my_id);
        private_bb->SetDouble("xGoal", goal.x);
        private_bb->SetDouble("yGoal", goal.y);
        private_bb->SetDouble("angleGoal", goal.rot);
        private_bb->SetBool("endPoint", true);
        private_bb->SetBool("dribbler", false);
        goToPos = std::make_unique<GoToPos>(n, "", private_bb);
    }
    
    //ROS_INFO("Goal: (%f, %f, %f)", private_bb->GetDouble("xGoal"), private_bb->GetDouble("yGoal"), private_bb->GetDouble("angleGoal"));
    
    bt::Node::Status gtpStatus = goToPos->Update();
    ROS_INFO("gtpStatus=%d", gtpStatus);
    if (gtpStatus != bt::Node::Status::Running) {
        gtp_valid = false;
    }
    return gtpStatus == bt::Node::Status::Invalid || gtpStatus == bt::Node::Status::Failure ? gtpStatus : bt::Node::Status::Running;
}
    
}
