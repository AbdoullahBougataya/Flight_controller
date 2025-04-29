#include "../include/2D_ComplementaryFilter.h"

void ComplementaryFilter2D_Init(ComplementaryFilter2D* cf2, float alpha)
{
    // Set the alpha filter ratio
    cf2->alpha = fmin(fmax(alpha, 0.0f), 1.0f);

    // Reset the altitude
    cf2->alt = 0;

    // Reset the current and previous acceleration
    cf2->accel[0] = 0;
    cf2->accel[1] = 0;

    // Reset the vertical velocities
    cf2->velocities[0] = 0;
    cf2->velocities[1] = 0;

    // Reset the output velocity
    cf2->velocity = 0;
}

float ComplementaryFilter2D_Update(ComplementaryFilter2D* cf2, float* accel, float* euler, float alt, float dt)
{
    // Set the sampling period
    cf2->T = dt;

    // Derivate the altitude and save the current altitude
    cf2->velocities[0] = (alt - cf2->alt) / cf2->T;
    cf2->alt = alt;

    // Pre-compute the sines and cosines
    float cosphi = cosf(euler[0]);
    float sintheta = sinf(euler[1]);
    float sinphi = sinf(euler[0]);
    float costheta = cosf(euler[1]);

    //Perform the trigonometry and integrate the acceleration
    cf2->accel[0] = -sintheta * fabs(accel[3])
        + (sinphi * costheta) * fabs(accel[4])
        + (cosphi * costheta) * fabs(accel[5]) - G_MPS2;
    cf2->velocities[1] = cf2->velocities[1] + ((cf2->accel[1] + cf2->accel[0]) / 2) * cf2->T;
    cf2->accel[1] = cf2->accel[0];

    //Perform sensor fusion
    cf2->velocity = cf2->alpha * cf2->velocities[1] + (1 - cf2->alpha) * cf2->velocities[0];

    return cf2->velocity;
}
