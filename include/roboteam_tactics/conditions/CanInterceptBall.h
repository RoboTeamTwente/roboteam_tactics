#pragma once

#include <cmath>

#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"

#define ACCEL_CHEAT .275
#define ACCEL_ELLIPSE_A .43
#define ACCEL_ELLIPSE_B .67
#define MAX_ICPT_TIME 10.0
#define ICPT_DELTA 0.001
#define ICPT_RADIUS 0.005

struct intercept_data {
    bool success;
    rtt::Vector2 icpt_pos;
    double time;
};

namespace rtt {

class CanInterceptBall : public Condition {
    public:

    CanInterceptBall(std::string name, bt::Blackboard::Ptr blackboard);

    Status Update() override;

    static VerificationMap required_parameters() {
        VerificationMap params;
        params["me"] = BBArgumentType::Int;
        return params;
    }

    struct intercept_data calc_intercept();

    static struct intercept_data calc_intercept(
                        const Position& bot_pos,
                        const Position& bot_vel,
                        const Vector2& ball_pos,
                        const Vector2& ball_vel) {

        auto bot_func = [bot_pos, bot_vel](double t) {
            double x = bot_pos.x + bot_vel.x * t + ACCEL_CHEAT * t * t;
            double y = bot_pos.y + bot_vel.y * t + ACCEL_CHEAT * t * t;
            return Vector2(x, y);
        };

        auto bot_range = [bot_vel](double t, double angle) {
            double ax = (ACCEL_ELLIPSE_A * ACCEL_ELLIPSE_B) / sqrtl(powl(ACCEL_ELLIPSE_B, 2) + powl(ACCEL_ELLIPSE_A, 2) * powl(tanl(angle), 2));
            double ay = sqrtl(1 - powl(ax/ACCEL_ELLIPSE_A, 2)) * ACCEL_ELLIPSE_B;
            double x = bot_vel.x * cosl(angle) * t - (ax/2.0) * t * t;
            double y = bot_vel.y * sinl(angle) * t - (ay/2.0) * t * t;
            return Vector2(x*cosl(angle), y*sinl(angle)).length();
        };

        auto ball_func = [ball_pos, ball_vel](double t) {
            double x = ball_pos.x + ball_vel.x * t;
            double y = ball_pos.y + ball_vel.y * t;
            return Vector2(x, y);
        };

        double time = -1.0;
        Vector2 icpt_pos;
        Vector2 bot = bot_pos.location();
        for (double t = ICPT_DELTA; t < MAX_ICPT_TIME; t += ICPT_DELTA) {
            Vector2 ball = ball_func(t);
            double range = bot_range(t, (ball).angle());
            //DEBUG_INFO_ICPT("At t=%f: bot(%f, %f) ball(%f, %f) dist=%f, range=%f angle=%f\n\n", t, bot.x, bot.y, ball.x, ball.y,
            //    ball.dist(bot), range, ball.angle());
            // DEBUG_INFO_ICPT("At t=%f: bot(%f, %f) ball(%f, %f) dist=%f, range=%f angle=%f\n\n", t, bot.x, bot.y, ball.x, ball.y,
                // ball.dist(bot), range, ball.angle());
            if (fabs(ball.dist(bot) - range) < ICPT_RADIUS) {
                time = t;
                icpt_pos = ball;
                break;
            }
        }

        return {time > 0, icpt_pos, time};
    }

    std::string node_name() { return "CanInterceptBall"; }

    private:

};

}
