#include <ros/console.h>    // Used for ROS logging

#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "unique_id/unique_id.h"

#include "roboteam_msgs/RoleDirective.h"

#include "roboteam_tactics/tactics/Emiel_test1.h"

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"


namespace rtt {

    RTT_REGISTER_TACTIC(Emiel_test1);

    /* Constructor */
    Emiel_test1::Emiel_test1(std::string name, bt::Blackboard::Ptr blackboard)
            : Tactic(name, blackboard)  // Initialize superclass
    {
        ROS_INFO_STREAM_NAMED("Emiel_test1", "[New] name=" << name);
    }



    /* Initialize : Called once when creating this play */
    void Emiel_test1::Initialize(){
        ROS_INFO_STREAM_NAMED("Emiel_test1", "[Initialize]");




        // === Check if there are robots available ===
        std::vector<int> robotsAvailable = getAvailableRobots();
        int nRobotsAvailable = robotsAvailable.size();

        ROS_INFO_STREAM_NAMED("Emiel_test1", "[Initialize] Robots available : " << nRobotsAvailable);

        if(nRobotsAvailable < 1){
            ROS_WARN_STREAM_NAMED("Emiel_test1", "[Initialize] Not enough robots available!");
            return;
        }

        // Print all the robots that are available
        for(int i = 0; i < nRobotsAvailable; i++){
            ROS_INFO_STREAM_NAMED("Emiel_test1", "[Initialize] Robot " << robotsAvailable.at(i) << " available");
        }
        // ===========================================



        // === Try to claim the first robot ===
        int robotId = robotsAvailable.front();
        ROS_INFO_STREAM_NAMED("Emiel_test1", "[Initialize] Trying to claim robotId " << robotId);
        bool robotClaimed = claim_robot(robotId);
        if(!robotClaimed){
            ROS_ERROR_STREAM_NAMED("Emiel_test1", "[Initialize] Could not claim robot, aborting...");
            return;
        }
        // ====================================




        // === Setup the Blackboard ===
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", robotId);                 // This is mandatory
        bb.SetInt("KEEPER_ID", robotId);                // This is mandatory, and has the wrong value at the moment!
        bb.SetDouble("GoToPos_Left_xGoal", -10.0);      // See rtt_emiel/new_tree GoToPos_Left
        bb.SetDouble("GoToPos_Left_yGoal", -10.0);      // See rtt_emiel/new_tree GoToPos_Left
        bb.SetDouble("GoToPos_Right_xGoal", 2.0);       // See rtt_emiel/new_tree GoToPos_Right
        bb.SetDouble("GoToPos_Right_yGoal", 2.0);       // See rtt_emiel/new_tree GoToPos_Right
        // ============================


        // === Setup the RoleDirective message === //
        /* The RoleDirective message is sent to a

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = robotId;

        rd.tree = "rtt_emiel/new_tree";
        rd.blackboard = bb.toMsg();

        // Get the default roledirective publisher
        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
        pub.publish(rd);
        ROS_INFO_STREAM_NAMED("Emiel_test1", "[Initialize] Published! " << robotId);

    }



    /* Update: Called on every frame / tick() */
    bt::Node::Status Emiel_test1::Update(){
        ROS_INFO_STREAM_NAMED("Emiel_test1", "[Update] returning Status::Success");

        // Check if the parameter 'aParam' is set in the blackboard
        if(HasInt("aParam")){
            ROS_INFO_STREAM_NAMED("Emiel_test1", "[Update] aParam set! : " << GetInt("aParam"));
        }else{
            ROS_INFO_STREAM_NAMED("Emiel_test1", "[Update] aParam NOT set!");
        }

        return Status::Success;
    }



    /* Terminate : Called once when removing this play */
    void Emiel_test1::Terminate(bt::Node::Status s){
        ROS_INFO_STREAM_NAMED("Emiel_test1", "[Terminate]");
    }

} // rtt