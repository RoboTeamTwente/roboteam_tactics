//
// Academic Student License -- for use by students to meet course
// requirements and perform academic research at degree granting
// institutions only.  Not for government, commercial, or other
// organizational use.
// File: intercept2.h
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 28-Jun-2017 12:17:14
//
#ifndef INTERCEPT2_H
#define INTERCEPT2_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "intercept2_types.h"

// Function Declarations
extern void intercept2(const double botPos[2], const double botVel[2], double
                       botAcc, const double ballPos[2], const double ballVel[2],
                       double ballAcc, boolean_T *valid, double iPos[2], double *
                       iTime);
extern void intercept2_initialize();
extern void intercept2_terminate();

#endif

//
// File trailer for intercept2.h
//
// [EOF]
//
