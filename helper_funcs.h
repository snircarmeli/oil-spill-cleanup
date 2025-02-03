#ifndef HELPER_FUNCS_H
#define HELPER_FUNCS_H

#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem> // For file operations
#include <algorithm> // for sort

#define PI 3.14159265358979323846
#define DEG2RAD PI / 180.0
#define RAD2DEG 180.0 / PI

using Eigen::Vector2f;
using Eigen::Matrix2f;

using std::min;
using std::max;
using std::string;


// Wrap theta between -pi and pi
float wrap_theta(float theta);

// Sign function
int sign(float x);

// Rotation matrix
Matrix2f rot_mat(float theta);

// Check if point k is on segment ij
bool on_segment(float xi, float yi, float xj, float yj, float xk, float yk);

// Check if two line segments intersect
bool check_intersection(float x_11, float y_11, float x_12, float y_12,
 float x_21, float y_21, float x_22, float y_22, float L);

bool canConvertToFloat(const string &s);

#endif // HELPER_FUNCS_H