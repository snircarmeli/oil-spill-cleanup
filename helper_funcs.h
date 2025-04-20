#ifndef HELPER_FUNCS_H
#define HELPER_FUNCS_H

#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem> // For file operations
#include <algorithm> // for sort
#include <cmath>
#include <limits>

#define PI 3.14159265358979323846
#define DEG2RAD PI / 180.0
#define RAD2DEG 180.0 / PI

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::VectorXi;

using std::min;
using std::max;
using std::string;
using std::sort;

using std::cout;
using std::endl;


// Wrap theta between -pi and pi
double wrap_theta(double theta);

// wrap eta between -pi / 2 and pi / 2
double wrap_eta(double eta);

// Sign function
int sign(double x);

// Rotation matrix
Matrix2d rot_mat(double theta);

// Check if point k is on segment ij
bool on_segment(double xi, double yi, double xj, double yj, double xk, double yk);

// Check if two line segments intersect
bool check_intersection(double x_11, double y_11, double x_12, double y_12,
 double x_21, double y_21, double x_22, double y_22, double L);

bool canConvertToFloat(const string &s);

// Function which calculate the derivative of the path in the global frame
MatrixXd path_der_global(MatrixXd path_points, double ts);

// Function which checks if jumps between two points are too big and interpolates them
MatrixXd check_path(MatrixXd path_points, double max_jump);

// Function that calculates the convex hull of a set of points
MatrixXd calculate_convex_hull(const MatrixXd &points);

// Function that calculates the minimal distance between two convex hulls
double calculate_convex_hull_distance(const MatrixXd &hull1, const MatrixXd &hull2);

// MatrixXd glob_path_points_2_local_frame_vel(MatrixXd path_points, double ts);

#endif // HELPER_FUNCS_H