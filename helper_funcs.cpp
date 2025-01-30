#include "helper_funcs.h"

// Wrap theta between -pi and pi
float wrap_theta(float theta) {
    theta = fmod(theta, 2 * PI); // Normalize theta within [-2PI, 2PI]
    if (theta > PI) {
        theta -= 2 * PI; // Adjust if theta is in (PI, 2PI]
    } else if (theta < -PI) {
        theta += 2 * PI; // Adjust if theta is in [-2PI, -PI)
    }
    // cout << theta << endl;
    return theta;
}

// Sign function
int sign(float x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

// Rotation matrix
Matrix2f rot_mat(float theta) {
    Matrix2f R;
    R << cos(theta), -sin(theta),
         sin(theta), cos(theta);
    return R;
}