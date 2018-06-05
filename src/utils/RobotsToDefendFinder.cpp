#include <vector>

#include "roboteam_msgs/World.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

#include "roboteam_tactics/utils/RobotsToDefendFinder.h"


namespace rtt {

    std::vector<int> RobotsToDefendFinder::GetRobotsToDefend(double minDangerScore, bool excludeKicker){

        // Get the last world
        roboteam_msgs::World world = LastWorld::get();

        // Get the position of the ball
        Vector2 ballPos(world.ball.pos);


        // === Get their robot that is the closest to the ball === //
        int oppKicker = -1; // Holds the kicker ID
        double oppClosestDistance = 9999; // Holds the closest distance

        for(size_t i = 0; i < world.them.size(); i++){
            // Get the distance between the ball and the current opponent
            double distanceToBall = (Vector2(world.them.at(i).pos) - ballPos).length();
            // If this distance is closer than previous distances, store it
            if(distanceToBall < oppClosestDistance ){
                oppClosestDistance = distanceToBall;
                oppKicker = i;
            }
        }

        // === Find the most dangerous opponents, excluding the kicker ===
        // Vector to hold the new robots to defend
        std::vector<int> newRobotsToDefend;
        // For each opponent in the world
        for (size_t i = 0; i < world.dangerList.size(); i++) {
            // Get the opponent
            roboteam_msgs::WorldRobot opp = world.dangerList.at(i);
            // If the robot is the kicker, ignore
            if(opp.id == oppKicker && excludeKicker)
                continue;
            // Get danger score of the robot
            float dangerScore = world.dangerScores.at(i);

            // If the danger score of the robot is too low, ignore it
            if(dangerScore < minDangerScore)
                continue;

            // The robot should be defended
            newRobotsToDefend.push_back(opp.id);
        }

        return newRobotsToDefend;
    }

}
