#ifndef GENERATESTEPS_H
#define GENERATESTEPS_H

#include <array>
#include <math.h>


std::array<std::array<float, 4>, 2> genSteps(float fullStepDistance, bool forward=true, float xOffset=0, bool directionForward=true, float defaultHeight=0.6, int nSubsteps=3);

#endif