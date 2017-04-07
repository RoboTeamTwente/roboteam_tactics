#pragma once

#include <boost/optional.hpp>
#include <vector>
#include <algorithm>
#include <mutex>
#include <thread>

#include "roboteam_utils/LastWorld.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/DangerFinder.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"

namespace rtt {
    
using Robot = roboteam_msgs::WorldRobot;
using DFService = roboteam_msgs::DangerFinder;

/**
 * \typedef DangerFactor
 * \brief A type of function which determines how dangerous a certain robot is right now by some metric
 */
typedef std::function<double(const Robot&, std::string*)> DangerFactor;

/**
 * \brief DangerFactor which gives high scores to robots which have a clear line of sight to our goal
 */
extern const DangerFactor canSeeOurGoal;

/**
 * \brief DangerFactor which gives high scores if it seems likely that an opponent might try to pass the
 * ball to this robot from the opposite side of the field.
 */
extern const DangerFactor potentialCrossRecipient;

/**
 * \brief DangerFactor which gives a very high score to the robot which has the ball, and low scores to all others.
 */
extern const DangerFactor hasBall;

/**
 * \brief DangerFactor which scores opponents relative to their distance to our goal
 */
extern const DangerFactor distance;

/**
 * \brief DangerFactor which scores opponents based on their orientation to our goal.
 * An opponent facing our goal will have a high score.
 */
extern const DangerFactor orientation;

/**
 * \brief The DangerFactors used by default
 */
extern const std::vector<DangerFactor> DEFAULT_FACTORS;

/**
 * \brief The essential coordinates of the left-side goal
 */
const std::vector<Vector2> GOAL_POINTS_LEFT({
    Vector2(-4.5, .35),
    Vector2(-4.5, 0),
    Vector2(-4.5, -.35)
});
  
/**
 * \brief The essential coordinates of the right-side goal
 */
const std::vector<Vector2> GOAL_POINTS_RIGHT({
    Vector2(4.5, .35),
    Vector2(4.5, 0),
    Vector2(4.5, -.35)
});

/**
 * \struct DangerResult
 * \brief The result of a single round of danger evaluation
 */
typedef struct {
    boost::optional<Robot> charging;                //< An optional robot with the ball, coming towards our goal
    boost::optional<Robot> mostDangerous;          //< The most dangerous opponent, if there is any
    boost::optional<Robot> secondMostDangerous;   //< The second most dangerous opponent, if there is more than one
    std::vector<Robot> dangerList;                 //< All opponents, sorted from least to most dangerous
} DangerResult;

/**
 * \class DangerFinder
 * \brief Evaluates all robots in a background thread, determining how 'dangerous' each is according
 * to an extendable set of metrics.
 */
class DangerFinder {
    public:
    DangerFinder();
    
    /**
     * \brief Starts the background thread.
     * \param delay The amount of milliseconds to pause between evaluations
     */
    virtual void run(unsigned int delay = 100);
    
    /**
     * \brief Stops the background thread. If an evaluation is currently running, it will be allowed to finish.
     */
    virtual void stop();
    
    /**
     * \brief Checks whether or not the background thread is running
     */
    virtual bool isRunning() const;
    
    /**
     * \brief Gets the most recent DangerResult. If the background thread is not running and has never run, this will be empty.
     */
    virtual DangerResult currentResult();
    
    /**
     * \brief Evaluates the current world state and returns a DangerResult. This does not affect the background
     * thread, whether it is running or not.
     */
    virtual DangerResult getImmediateUpdate() const;
    
    private:
    void runImpl(unsigned int delay);
    DangerResult update() const;
    
    DangerResult result;
    bool running;
    volatile bool _stop;
    std::mutex lock;
    std::thread runner;
};

class RemoteDangerFinder : public DangerFinder {
    public:
    RemoteDangerFinder();
    void run(unsigned int delay) override;
    void stop() override;
    bool isRunning() const override;
    DangerResult currentResult() override;
    DangerResult getImmediateUpdate() const override;
};

extern RemoteDangerFinder dangerFinder;

namespace df_impl {
    std::vector<Vector2> ourGoal();

    bool weAreLeft();

    double dangerScore(const Robot& bot, const std::vector<DangerFactor>& factors = DEFAULT_FACTORS, 
                    bool includeCross = true, unsigned int preferred = 999);
    
    void dumpScores(const roboteam_msgs::World& world);    
    
    boost::optional<Robot> chargingBot();
    boost::optional<Robot> mostDangerousBot(unsigned int preferred = 999);
    boost::optional<Robot> secondMostDangerousBot(unsigned int preferred = 999);
    std::vector<Robot> sortedOpponents(const roboteam_msgs::World& world, unsigned int preferred);
}

std::vector<Vector2> ourGoal();
bool weAreLeft();
boost::optional<Robot> chargingBot();
boost::optional<Robot> mostDangerousBot();
boost::optional<Robot> secondMostDangerousBot();
   
}
