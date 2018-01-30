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
        ROS_INFO_STREAM_NAMED("Emiel_test1", "New instance. name=" << name);
    }



    /* Initialize : Called once when creating this play */
    void Emiel_test1::Initialize(){
        ROS_INFO_STREAM_NAMED("Emiel_test1", "Initializing..");


        // === Check if there are robots available ===
        std::vector<int> robotsAvailable = getAvailableRobots();
        int nRobotsAvailable = robotsAvailable.size();

        // Print all the robots that are available
        std::stringstream result;
        std::copy(robotsAvailable.begin(), robotsAvailable.end(), std::ostream_iterator<int>(result, ","));
        ROS_INFO_STREAM_NAMED("Emiel_test1", "Robots " << result.str().c_str() << " available");

        if(nRobotsAvailable < 1){
            ROS_WARN_STREAM_NAMED("Emiel_test1", "Not enough robots available!");
            return;
        }
        // ===========================================


        // === Try to claim the first robot ===
        int robotId = robotsAvailable.front();
        bool robotClaimed = claim_robot(robotId);
        if(!robotClaimed){
            ROS_ERROR_STREAM_NAMED("Emiel_test1", "Could not claim robot " << robotId << ", aborting...");
            return;
        }
        ROS_INFO_STREAM_NAMED("Emiel_test1", "Robot " << robotId << " claimed");
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


        // === Setup and publish the RoleDirective message ===
        roboteam_msgs::RoleDirective rd;        // Create the message
        rd.robot_id = robotId;                  // Set the robot_id

        rd.tree = "rtt_emiel/new_tree";         // Set the role tree for the robot
        rd.blackboard = bb.toMsg();             // Convert the blackboard to a message, and add it


        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();    // Get the default roledirective publisher
        pub.publish(rd);                                                                    // Publish the message
        ROS_INFO_STREAM_NAMED("Emiel_test1", "Published RoleDirective for robot " << robotId);
        // ===================================================

        timeSinceInitialized = now();
    }



    /* Update: Called on every frame / tick() */
    /* The tree is terminated when the tree returns something other than Status::Running (If I recall correctly */
    bt::Node::Status Emiel_test1::Update(){
        ROS_INFO_STREAM_ONCE_NAMED("Emiel_test1", "Updating");  // Print only once, to prevent clogging the console

        // Check if three seconds have passed, for no other reason than demonstrating
        if (time_difference_milliseconds(timeSinceInitialized, now()).count() < 3000) {
            return Status::Running;
        }


        // Check if the parameter 'aParam' is set in the blackboard
        if(HasInt("aParam")){
            ROS_INFO_STREAM_NAMED("Emiel_test1", "aParam set! : " << GetInt("aParam"));
        }else{
            ROS_INFO_STREAM_NAMED("Emiel_test1", "aParam NOT set!");
        }


        ROS_INFO_STREAM_NAMED("Emiel_test1", "Three seconds have passed!");
        return Status::Success;
    }



    /* Terminate : Called once when removing this play */
    void Emiel_test1::Terminate(bt::Node::Status s){
        ROS_INFO_STREAM_NAMED("Emiel_test1", "Terminating");
    }

} // rtt