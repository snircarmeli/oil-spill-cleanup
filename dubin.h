#ifndef DUBIN_H
#define DUBIN_H

#include <cmath>
#include <limits>

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
typedef int (*DubinsWord)(double, double, double, double*);

// A complete list of the possible solvers that could give optimal paths
extern DubinsWord dubins_words[];

// Dubins path structure
typedef struct {
    double qi[3];       // The initial configuration
    double param[3];    // The lengths of the three segments
    double rho;         // Model forward velocity / model angular velocity
    int type;           // Path type. One of LSL, LSR, ...
} DubinsPath;

// Callback function for path sampling
typedef int (*DubinsPathSamplingCallback)(double q[3], double t, void* user_data);

// Function declarations
double fmodr(double x, double y);
double mod2pi(double theta);
int dubins_init(double q0[3], double q1[3], double rho, DubinsPath* path);
double dubins_path_length(DubinsPath* path);
int dubins_path_type(DubinsPath* path);
int dubins_path_sample(DubinsPath* path, double t, double q[3]);
int dubins_path_sample_many(DubinsPath* path, DubinsPathSamplingCallback cb, double stepSize, void* user_data);
int dubins_path_endpoint(DubinsPath* path, double q[3]);
int dubins_extract_subpath(DubinsPath* path, double t, DubinsPath* newpath);
int dubins_LSL(double alpha, double beta, double d, double* outputs);
int dubins_RSR(double alpha, double beta, double d, double* outputs);
int dubins_LSR(double alpha, double beta, double d, double* outputs);
int dubins_RSL(double alpha, double beta, double d, double* outputs);
int dubins_LRL(double alpha, double beta, double d, double* outputs);
int dubins_RLR(double alpha, double beta, double d, double* outputs);

#endif // DUBIN_H