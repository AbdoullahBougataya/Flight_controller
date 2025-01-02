#include"../include/Complimentary_filter.h"

void complimentaryFilter(float p, float q, float r, float x_acc, float y_acc, float z_acc, float deltaTime, float &phi, float &theta)
{
    // Using gravity to estimate the euler angles
    phiHat_acc_rad = atanf(filteredAccelGyro[4] / filteredAccelGyro[5]); // Roll estimate
    thetaHat_acc_rad = asinf(fminf(fmaxf(filteredAccelGyro[3] / G_MPS2, -1.0f), 1.0f)); // Pitch estimate
    phi = COMP_FLTR_ALPHA * atanf(filteredAccelGyro[4] / filteredAccelGyro[5]) + (1.0f - COMP_FLTR_ALPHA) * (phiHat_rad + deltaTime * (filteredAccelGyro[0] + tanf(thetaHat_rad) * (sinf(phiHat_rad) * filteredAccelGyro[1] + cosf(phiHat_rad) * filteredAccelGyro[2])));
}
