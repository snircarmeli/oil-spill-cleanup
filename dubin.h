#ifndef DUBIN_H
#define DUBIN_H

#include <cmath>
#include <limits>
#include <iostream>
#include <Eigen/Dense>
#include <fstream> // For file operations


using Eigen::MatrixXf;
using std::string;
using std::endl;


// Error codes
#define EDUBOK        (0)   // No error
#define EDUBCOCONFIGS (1)   // Colocated configurations
#define EDUBPARAM     (2)   // Path parameterization error
#define EDUBBADRHO    (3)   // The rho value is invalid
#define EDUBNOPATH    (4)   // No connection between configurations with this word

// Path types
#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

// The three segment types a path can be made up of
#define L_SEG (0)
#define S_SEG (1)
#define R_SEG (2)

// The various types of solvers for each of the path types
typedef int (*DubinsWord)(float, float, float, float*);

// A complete list of the possible solvers that could give optimal paths
extern DubinsWord dubins_words[];

// Dubins path structure
typedef struct {
    float qi[3];       // The initial configuration
    float param[3];    // The lengths of the three segments
    float rho;         // Model forward velocity / model angular velocity
    int type;           // Path type. One of LSL, LSR, ...
} DubinsPath;

// Callback function for path sampling
typedef int (*DubinsPathSamplingCallback)(float q[3], float t, void* user_data);

// Function declarations
float fmodr(float x, float y);
float mod2pi(float theta);
int dubins_init(float q0[3], float q1[3], float rho, DubinsPath* path);
float dubins_path_length(DubinsPath* path);
int dubins_path_type(DubinsPath* path);
int dubins_path_sample(DubinsPath* path, float t, float q[3]);
int dubins_path_sample_many(DubinsPath* path, DubinsPathSamplingCallback cb,
 float stepSize, void* user_data);
int dubins_path_endpoint(DubinsPath* path, float q[3]);
int dubins_extract_subpath(DubinsPath* path, float t, DubinsPath* newpath);
int dubins_LSL(float alpha, float beta, float d, float* outputs);
int dubins_RSR(float alpha, float beta, float d, float* outputs);
int dubins_LSR(float alpha, float beta, float d, float* outputs);
int dubins_RSL(float alpha, float beta, float d, float* outputs);
int dubins_LRL(float alpha, float beta, float d, float* outputs);
int dubins_RLR(float alpha, float beta, float d, float* outputs);
void save_to_file(string file_path, string folder_name, MatrixXf path,
 MatrixXf path_R, MatrixXf path_L);

#endif // DUBIN_H