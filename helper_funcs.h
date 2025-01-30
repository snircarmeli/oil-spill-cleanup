#ifndef HELPER_FUNCS_H
#define HELPER_FUNCS_H

#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem> // For file operations

#define PI 3.14159265358979323846
#define DEG2RAD PI / 180.0
#define RAD2DEG 180.0 / PI

using Eigen::Vector2f;
using Eigen::Matrix2f;


// Wrap theta between -pi and pi
float wrap_theta(float theta);

// Sign function
int sign(float x);

// Rotation matrix
Matrix2f rot_mat(float theta);

#endif