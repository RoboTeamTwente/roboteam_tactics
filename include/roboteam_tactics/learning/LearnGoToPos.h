#pragma once

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_utils/Optimization.h"

namespace rtt {

typedef GradientDescent<double, 2> GTPLearner;

extern const GTPLearner::ScoreFunction gtpScorer;

}

int main(int argc, char* argv[]);
