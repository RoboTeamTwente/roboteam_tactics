#include "roboteam_tactics/learning/LearnGoToPos.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Position.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/RoleDirective.h"


constexpr int TIME_LIMIT_MILLIS = 3000;         //< Time in ms to allow before applying a score penalty
constexpr rtt::Position TGT_POS(2.0, 2.0, M_PI);//< Position to drive to
constexpr rtt::Position INIT_POS;				//< Position to return to after tests
constexpr size_t MAX_ITERATIONS = 10;			//< Max number of iterations to train
constexpr double INIT_P_POS = 2.0;				//< Initial pGainPosition
constexpr double INIT_P_ROT = 4.0;				//< Initial pGainRotation
constexpr double MIN_GAIN = .5;					//< Minimum possible gain value
constexpr double MAX_GAIN = 15;					//< Maximum possible gain value
constexpr double GAIN_STEP = 1.0;				//< Initial gain step size
constexpr double CONVERGENCE_THRESHOLD = 0.001; //< Divergence value at which to consider the training to have converged

namespace rtt {

bt::Node::Status runGTP(const Position& pos, boost::optional<GTPLearner::Data> params) {
	bt::Blackboard bb;
	bb.SetInt("ROBOT_ID", 0);
	bb.SetDouble("LearningGTP_xGoal", pos.x);
	bb.SetDouble("LearningGTP_yGoal", pos.y);
	bb.SetDouble("LearningGTP_angleGoal", pos.rot);
	bb.SetBool("LearningGTP_avoidRobots", true);
	GoToPos gtp("LearningGTP", std::make_shared<bt::Blackboard>(bb));
	gtp.Initialize();

	if(params) {
		gtp.setPGains((*params)[0], (*params)[1]);
	}

	bt::Node::Status status = bt::Node::Status::Running;
	while (status == bt::Node::Status::Running && ros::ok()) {
		status = gtp.Update();
		ros::spinOnce();
	}
	return status;
}

const GTPLearner::ScoreFunction gtpScorer = [](const GTPLearner::Data& data) {
	time_point startTime = now();
	auto status = runGTP(TGT_POS, data);
	milliseconds time = time_difference_milliseconds(startTime, now());
	if (status != bt::Node::Status::Success) {
		return -99999999.0;
	}

	auto bot = getWorldBot(0);
	if (!bot) throw std::runtime_error("Could not get bot 0 in LearnGoToPos");

	Position pos(bot->pos.x, bot->pos.y, bot->angle);

	double score = fabs(pos.x - TGT_POS.x)
				 + fabs(pos.y - TGT_POS.y)
				 + fabs(pos.rot - TGT_POS.rot);

	if (time.count() > TIME_LIMIT_MILLIS) {
		score += (double) (time.count() - TIME_LIMIT_MILLIS) / 1000.0;
		// Arriving 1 second past the limit is equally bad as
		// missing the target by 10 cm.
	}

	runGTP(INIT_POS, boost::none);
	return -score;
};

}

int main(int argc, char* argv[]) {
	std::cout << "Setting up LearnGoToPos... ";
	ros::init(argc, argv, "LearnGoToPos");
	ros::NodeHandle n;
	rtt::WorldAndGeomCallbackCreator callbacks;
	rtt::GlobalPublisher<roboteam_msgs::RobotCommand> globalRobotCommandPublisher(rtt::TOPIC_COMMANDS);
	rtt::GlobalPublisher<roboteam_msgs::RoleDirective> globalRoleDirectivePublisher(rtt::TOPIC_ROLE_DIRECTIVE);
	rtt::LastWorld::wait_for_first_messages();
	ros::param::set("robot0/robotType", "grsim");

	std::cout << "done. Initial pGains: pos=" << INIT_P_POS << ", rot=" << INIT_P_ROT << "\n";

	rtt::GTPLearner::Data params = { INIT_P_POS, INIT_P_ROT };
	rtt::GTPLearner learner(rtt::gtpScorer, params, GAIN_STEP, MIN_GAIN, MAX_GAIN);
	do {
		std::cout << "Running iteration " << learner.getIterationCount()
				  << ", last score = " << learner.getLastScore()
				  << ". Params: {" << learner.getCurrentValues()[0]
			      << ", " << learner.getCurrentValues()[1] << "}\n";
		learner.singleIteration();
	} while (fabs(learner.getLastDivergence()) > CONVERGENCE_THRESHOLD
			  && learner.getIterationCount() < MAX_ITERATIONS && ros::ok());
	bool converged = learner.getIterationCount() < MAX_ITERATIONS;
	rtt::GTPLearner::Data results = learner.getCurrentValues();
	std::cout << std::boolalpha;
	std::cout << "Learner results:\n\tConverged (" << MAX_ITERATIONS << " max): " << converged << "\n\tpGainPosition: "
			  << results[0] << "\n\tpGainRotation: " << results[1] << "\n\tIterations: " << learner.getIterationCount()
			  << "\n\tLast Score: " << learner.getLastScore() << "\n\tLast Divergence: " << learner.getLastDivergence()
			  << "\n\n";
	return 0;
}
