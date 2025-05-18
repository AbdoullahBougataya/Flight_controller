#include "../include/ComplementaryFilter.h"

void ComplementaryFilter_Init(ComplementaryFilter* cf, float alpha)
{
    // Set the alpha filter ratio
    cf->alpha = fminf(fmaxf(alpha, 0.0f), 1.0f);

    // Reseting the Euler angles
    cf->eulerAngles[0] = 0.0f;
    cf->eulerAngles[1] = 0.0f;
    cf->eulerAngles[2] = 0.0f;
}

void ComplementaryFilter_Update(ComplementaryFilter* cf, float* inp, float* euler, float dt)
{
  // Set the sampling period
  cf->T = dt;

  // Using gravity to estimate the Euler angles
  float phiHat_acc_rad = atanf(inp[4] / sqrt(pow(inp[5], 2) + sqrt(pow(inp[3], 2))));        // Roll estimate
  float thetaHat_acc_rad = asinf(fminf(fmaxf(inp[3] / G_MPS2, -1.0f), 1.0f));                // Pitch estimate

  // Using gyroscope to estimate the euler rates (Transforming body rates to euler rates)
  inp[0] = (sinf(cf->eulerAngles[0]) * inp[1] + cosf(cf->eulerAngles[0]) * inp[2]) * tanf(cf->eulerAngles[1]) + inp[0]; // Roll rate (rad/s)
  inp[1] =  cosf(cf->eulerAngles[0]) * inp[1] - sinf(cf->eulerAngles[0]) * inp[2];                                      // Pitch rate (rad/s)
  inp[2] = (sinf(cf->eulerAngles[0]) * inp[1] + cosf(cf->eulerAngles[0]) * inp[2]) * (1 / cosf(cf->eulerAngles[1]));    // Yaw rate (rad/s)

  // Complementary filter implementation [Just like mixing the data from the gyroscope and the accelerometer with alpha proportions](alpha should be between 0 and 1)
  cf->eulerAngles[0] = fminf(fmaxf(cf->alpha * phiHat_acc_rad + (1.0f - cf->alpha) * (cf->eulerAngles[0] + cf->T * inp[0]), -PI), PI);    // Roll estimate
  cf->eulerAngles[1] = fminf(fmaxf(cf->alpha * thetaHat_acc_rad + (1.0f - cf->alpha) * (cf->eulerAngles[1] + cf->T * inp[1]), -PI), PI);  // Pitch estimate
  cf->eulerAngles[2] = fminf(fmaxf(cf->eulerAngles[2] + cf->T * inp[2], -PI), PI);                                                        // Yaw estimate
  for (int i = 0; i < 3; i++)
  {
    euler[i] = cf->eulerAngles[i];
  }
}

