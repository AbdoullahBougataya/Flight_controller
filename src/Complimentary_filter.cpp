#include"../include/Complimentary_filter.h"

void complimentaryFilter(int p, int q, int r, double x_acc, double y_acc, double z_acc, int &phi, double &theta)
{
    // Using gravity to estimate the euler angles
    phiHat_acc_rad = atanf(filteredAccelGyro[4] / filteredAccelGyro[5]); // Roll estimate
    thetaHat_acc_rad = asinf(fminf(fmaxf(filteredAccelGyro[3] / G_MPS2, -1.0f), 1.0f)); // Pitch estimate
}
