#include <array>
#include <math.h>
#include "generateSteps.h"

/*
 * Function to generate x, y (x is forward/backward, y[not z due to SolidWorks axis orientation] is upward/downward) coordinates
 * for a step. A full step is travelled in "n" steps/intervals. 
 * fullStepDistance: this is the diameter of a circle, where the actual step distance is the length of a segment of this circle
 * subtended at an angle theta (theta is 150 deg by default)
 * forward: this parameter specifies if the step is in the forward direction. True indicates forward
 * xOffset: this parameter gives the offset value in the x direction at the start of the step
 * directionForward: this parameter specifies the direction of motion of tachyon (forward or backward). True indicates forward
 * and vice versa
 * defaultHeight: this parameter specifies the current height of tachyon/the height that is to be maintained
 * nSubsteps: this parameter states the number of substeps
 */

std::array<std::array<float, 4>, 2> genSteps(float fullStepDistance, bool forward, float xOffset, bool directionForward, float defaultHeight, int nSubsteps)
{
    std::array<std::array<float, 4>, 2> xAndYVals;
    float xDir = forward? 1: -1;
    float lift = forward == directionForward? 1: 0; // zero lift if foot is supposed to be dragged

    float r = fullStepDistance / 2;
    float theta = (M_PI / 6) * 4; // 120 degrees
    float d = sqrt(pow(r, 2) + pow(r, 2) - (2 * pow(r, 2) * cos(theta)));
    float h = r * cos(theta / 2);
    float b = r - (d / 2);

    float alpha = M_PI_2 - (theta / 2);
    float step = theta / nSubsteps;

    for (int i = 0;i <= nSubsteps;i++)
    {
        float x = r - (r * cos(alpha));
        float y = r * sin(alpha);

        x = (x - b + xOffset) * xDir;
        y = ((y - h) * lift) - defaultHeight;

        xAndYVals[0][i] = x;
        xAndYVals[1][i] = y;
        alpha += step;
    }

    return xAndYVals;

}
