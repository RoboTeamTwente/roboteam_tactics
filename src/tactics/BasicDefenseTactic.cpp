#include "roboteam_msgs/World.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/GeometryFieldSize.h"

#include "roboteam_utils/Vector2.h"


#include "roboteam_tactics/tactics/BasicDefenseTactic.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG BasicDefenseTactic

namespace rtt {

RTT_REGISTER_TACTIC(BasicDefenseTactic);

BasicDefenseTactic::BasicDefenseTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void BasicDefenseTactic::Initialize() {
    tokens.clear();

    RTT_DEBUG("Initializing\n");
    
    roboteam_msgs::World world = rtt::LastWorld::get();

    // Assign the remaining robots the secondary keeper role
    std::vector<int> robots = RobotDealer::get_available_robots();
    // double yStep = LastWorld::get_field().field_width / (robots.size() + 1);
    // double startY = LastWorld::get_field().field_width / 2 - yStep;

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // const Vector2 theirGoal = rtt::LastWorld::get_their_goal_center();

    int goalareaDefenders=0;

    std::vector<roboteam_msgs::WorldRobot> dangerousbots;

    // find most dangerous attacker
    // TODO: determine to most dangerous attackers
    for (const auto& bot : world.them) {
        std::cout<<"id: "<<bot.id<<", x:"<<bot.pos.x<<", y:"<<bot.pos.y<<std::endl;
        

    }
    ROS_INFO("end of opponents");

    for (const int ROBOT_ID : robots) {
        ROS_INFO("found a robot");

        if(goalareaDefenders!=2){
            goalareaDefenders+=1;

            string position;
            if(goalareaDefenders==1){position="top";}
            else {position="bottom";}

            bt::Blackboard bb;

            bb.SetInt("ROBOT_ID", ROBOT_ID);
            bb.SetInt("KEEPER_ID",0);
            bb.SetInt("numberOfRobots", 2);
            bb.SetString("position",position);

            // Create message
            roboteam_msgs::RoleDirective wd;
            wd.robot_id = ROBOT_ID;
            wd.tree = "DefendGoalarea";
            
            boost::uuids::uuid token = unique_id::fromRandom();
            //tokens.push_back(token);
            wd.token = unique_id::toMsg(token);

            wd.blackboard = bb.toMsg();
           
            // Send to rolenode
            pub.publish(wd);
            

        } else {

            //int tgt;
            //tgt = dangerFinder.getImmediateUpdate().mostDangerous->id;
            

            bt::Blackboard bb;

            bb.SetInt("ROBOT_ID", ROBOT_ID);
            bb.SetInt("KEEPER_ID",0);
            bb.SetInt("TGT_ID",3);
            bb.SetString("block_type","ABSOLUTE");
            bb.SetDouble("block_arg", 0.5);
            bb.SetDouble("block_x",4.5);
            bb.SetDouble("block_y",0.0);

        
            // Create message
            roboteam_msgs::RoleDirective wd;
            wd.robot_id = ROBOT_ID;
            wd.tree = "Block";
            
            boost::uuids::uuid token = unique_id::fromRandom();
            //tokens.push_back(token);
            wd.token = unique_id::toMsg(token);
            


            wd.blackboard = bb.toMsg();

            // Send to rolenode
            pub.publish(wd);

        }

        


        RTT_DEBUG("Claiming: %i\n", ROBOT_ID);
    }

    claim_robots(robots);
}

bt::Node::Status BasicDefenseTactic::Update() {
    // Defense tactic is never done
    return bt::Node::Status::Running;
}


} // rtt

