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
extern const DangerFactor can_see_our_goal;

/**
 * \brief DangerFactor which gives high scores if it seems likely that an opponent might try to pass the
 * ball to this robot from the opposite side of the field.
 */
extern const DangerFactor potential_cross_recipient;

/**
 * \brief DangerFactor which gives a very high score to the robot which has the ball, and low scores to all others.
 */
extern const DangerFactor has_ball;

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
    boost::optional<Robot> most_dangerous;          //< The most dangerous opponent, if there is any
    boost::optional<Robot> second_most_dangerous;   //< The second most dangerous opponent, if there is more than one
    std::vector<Robot> danger_list;                 //< All opponents, sorted from least to most dangerous
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
    virtual bool is_running() const;
    
    /**
     * \brief Gets the most recent DangerResult. If the background thread is not running and has never run, this will be empty.
     */
    virtual DangerResult current_result();
    
    /**
     * \brief Evaluates the current world state and returns a DangerResult. This does not affect the background
     * thread, whether it is running or not.
     */
    virtual DangerResult get_immediate_update() const;
    
    private:
    void _run(unsigned int delay);
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
    bool is_running() const override;
    DangerResult current_result() override;
    DangerResult get_immediate_update() const override;
};

extern RemoteDangerFinder danger_finder;

namespace df_impl {
    std::vector<Vector2> our_goal();

    bool we_are_left();

    double danger_score(const Robot& bot, const std::vector<DangerFactor>& factors = DEFAULT_FACTORS, 
                    bool include_cross = true, unsigned int preferred = 999);
    
    void dump_scores(const roboteam_msgs::World& world);    
    
    boost::optional<Robot> charging_bot();
    boost::optional<Robot> most_dangerous_bot(unsigned int preferred = 999);
    boost::optional<Robot> second_most_dangerous_bot(unsigned int preferred = 999);
    std::vector<Robot> sorted_opponents(const roboteam_msgs::World& world, unsigned int preferred);
}

std::vector<Vector2> our_goal();
bool we_are_left();
boost::optional<Robot> charging_bot();
boost::optional<Robot> most_dangerous_bot();
boost::optional<Robot> second_most_dangerous_bot();
   
}
