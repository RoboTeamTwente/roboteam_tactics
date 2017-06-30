//
// Academic Student License -- for use by students to meet course
// requirements and perform academic research at degree granting
// institutions only.  Not for government, commercial, or other
// organizational use.
// File: intercept2.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 28-Jun-2017 12:21:15
//

// Include Files
#include "intercept2.h"

// Function Declarations
static double findTimeToClosestPoint(const double botPos[2], const double
    botVel[2], double botAcc, const double from[2], const double to[2], double
    frac);

// Function Definitions

//
// Arguments    : const double botPos[2]
//                const double botVel[2]
//                double botAcc
//                const double from[2]
//                const double to[2]
//                double frac
// Return Type  : double
//
static double findTimeToClosestPoint(const double botPos[2], const double
    botVel[2], double botAcc, const double from[2], const double to[2], double
    frac)
{
    double time;
    double dist;
    double scale;
    int k;
    double absxk;
    double t;

    // 'findTimeToClosestPoint:3' tgtPos = to * frac + from * (1 - frac)
    // 'findTimeToClosestPoint:4' time = timeToPos(botPos, botVel, botAcc, tgtPos); 
    // 'timeToPos:3' diff = tgtPos - botPos;
    // dir = acos(dot(tgtPos, botPos) / (norm(tgtPos) * norm(botPos)));
    // 'timeToPos:5' dist = norm(diff);
    dist = 0.0;
    scale = 2.2250738585072014E-308;
    for (k = 0; k < 2; k++) {
        absxk = std::abs((to[k] * frac + from[k] * (1.0 - frac)) - botPos[k]);
        if (absxk > scale) {
            t = scale / absxk;
            dist = 1.0 + dist * t * t;
            scale = absxk;
        } else {
            t = absxk / scale;
            dist += t * t;
        }
    }

    dist = scale * std::sqrt(dist);

    // 'timeToPos:6' t = (botVel(1) + sqrt(botVel(1)^2 + 2 * botAcc * ...
    // 'timeToPos:7'     (dist+botPos(1)) - 2 * botAcc * botPos(1))) / botAcc;
    time = (botVel[0] + std::sqrt((botVel[0] * botVel[0] + 2.0 * botAcc * (dist
               + botPos[0])) - 2.0 * botAcc * botPos[0])) / botAcc;

    // 'timeToPos:8' if ~isreal(t)
    return time;
}

//
// Arguments    : const double botPos[2]
//                const double botVel[2]
//                double botAcc
//                const double ballPos[2]
//                const double ballVel[2]
//                double ballAcc
//                boolean_T *valid
//                double iPos[2]
//                double *iTime
// Return Type  : void
//
void intercept2(const double botPos[2], const double botVel[2], double botAcc,
                const double ballPos[2], const double ballVel[2], double ballAcc,
                boolean_T *valid, double iPos[2], double *iTime)
{
    double farBall[2];
    double fx;
    double dx;
    double frac;
    double a;
    double fb;
    boolean_T exitg2;
    int i0;
    double fc;
    double c;
    double e;
    double d;
    boolean_T exitg1;
    boolean_T res;
    double m;
    double b_dx;
    double toler;
    double s;
    double r;

    // 'intercept2:3' farTime = 3;
    // 'intercept2:4' farBall = ballPosAfterTime(ballPos, ballVel, ballAcc, farTime); 
    // 'ballPosAfterTime:3' pos = [ initPos(1) + initVel(1) * t + (decceleration * t^2)/2,... 
    // 'ballPosAfterTime:4'         initPos(2) + initVel(2) * t + (decceleration * t^2)/2 
    // 'ballPosAfterTime:5'       ];
    farBall[0] = (ballPos[0] + ballVel[0] * 3.0) + ballAcc * 9.0 / 2.0;
    farBall[1] = (ballPos[1] + ballVel[1] * 3.0) + ballAcc * 9.0 / 2.0;

    // 'intercept2:6' frac = fzero(@(f) findTimeToClosestPoint(botPos, botVel,... 
    // 'intercept2:7'     botAcc, ballPos, farBall, f) - farTime*f,...
    // 'intercept2:8'     0);
    fx = findTimeToClosestPoint(botPos, botVel, botAcc, ballPos, farBall, 0.0);
    if (fx == 0.0) {
        frac = 0.0;
    } else {
        dx = 0.02;
        a = 0.0;
        frac = 0.0;
        fb = fx;
        exitg2 = false;
        while ((!exitg2) && ((fx > 0.0) == (fb > 0.0))) {
            dx *= 1.4142135623730951;
            a = 0.0 - dx;
            fx = findTimeToClosestPoint(botPos, botVel, botAcc, ballPos, farBall,
                0.0 - dx) - 3.0 * (0.0 - dx);
            if ((fx > 0.0) != (fb > 0.0)) {
                exitg2 = true;
            } else {
                frac = dx;
                fb = findTimeToClosestPoint(botPos, botVel, botAcc, ballPos,
                    farBall, dx) - 3.0 * dx;
            }
        }

        fc = fb;
        c = frac;
        e = 0.0;
        d = 0.0;
        exitg1 = false;
        while ((!exitg1) && ((fb != 0.0) && (a != frac))) {
            if ((fb > 0.0) == (fc > 0.0)) {
                c = a;
                fc = fx;
                d = frac - a;
                e = d;
            }

            if (std::abs(fc) < std::abs(fb)) {
                a = frac;
                frac = c;
                c = a;
                fx = fb;
                fb = fc;
                fc = fx;
            }

            m = 0.5 * (c - frac);
            dx = std::abs(frac);
            if (dx >= 1.0) {
                b_dx = dx;
            } else {
                b_dx = 1.0;
            }

            toler = 4.4408920985006262E-16 * b_dx;
            if ((std::abs(m) <= toler) || (fb == 0.0)) {
                exitg1 = true;
            } else {
                if ((std::abs(e) < toler) || (std::abs(fx) <= std::abs(fb))) {
                    d = m;
                    e = m;
                } else {
                    s = fb / fx;
                    if (a == c) {
                        dx = 2.0 * m * s;
                        fx = 1.0 - s;
                    } else {
                        fx /= fc;
                        r = fb / fc;
                        dx = s * (2.0 * m * fx * (fx - r) - (frac - a) * (r -
                                   1.0));
                        fx = (fx - 1.0) * (r - 1.0) * (s - 1.0);
                    }

                    if (dx > 0.0) {
                        fx = -fx;
                    } else {
                        dx = -dx;
                    }

                    if ((2.0 * dx < 3.0 * m * fx - std::abs(toler * fx)) && (dx <
                         std::abs(0.5 * e * fx))) {
                        e = d;
                        d = dx / fx;
                    } else {
                        d = m;
                        e = m;
                    }
                }

                a = frac;
                fx = fb;
                if (std::abs(d) > toler) {
                    frac += d;
                } else if (frac > c) {
                    frac -= toler;
                } else {
                    frac += toler;
                }

                fb = findTimeToClosestPoint(botPos, botVel, botAcc, ballPos,
                    farBall, frac) - 3.0 * frac;
            }
        }
    }

    // 'intercept2:10' iPos = farBall * frac + ballPos;
    for (i0 = 0; i0 < 2; i0++) {
        iPos[i0] = farBall[i0] * frac + ballPos[i0];
    }

    // 'intercept2:11' iTime = findTimeToClosestPoint(botPos, botVel, botAcc,... 
    // 'intercept2:12'                        ballPos, farBall, frac);
    *iTime = findTimeToClosestPoint(botPos, botVel, botAcc, ballPos, farBall,
        frac);

    // 'intercept2:13' valid = interceptionValid(iPos, iTime);
    // 'interceptionValid:3' x = iPos(1);
    // 'interceptionValid:4' y = iPos(2);
    // 'interceptionValid:5' res = x <= 9 && x >= -9 && y <= 6 && y >= -6 && iTime >= 0 && iTime < 3; 
    if ((iPos[0] <= 9.0) && (iPos[0] >= -9.0) && (iPos[1] <= 6.0) && (iPos[1] >=
         -6.0) && (*iTime >= 0.0) && (*iTime < 3.0)) {
        res = true;
    } else {
        res = false;
    }

    *valid = res;
}

//
// Arguments    : void
// Return Type  : void
//
void intercept2_initialize()
{
}

//
// Arguments    : void
// Return Type  : void
//
void intercept2_terminate()
{
    // (no terminate code required)
}

//
// File trailer for intercept2.cpp
//
// [EOF]
//
