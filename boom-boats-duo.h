#ifndef BOOM_BOATS_DUO_H
#define BOOM_BOATS_DUO_H

#include "boom-boat.h"
#include <Eigen/Dense>

// For JSON parameters parsing
#include "json/json.hpp"
using json = nlohmann::json;

using Eigen::MatrixXf;
using Eigen::Vector2f;
using Eigen::VectorXf;
using std::string;

class Boom {
private:
    MatrixXf links_states; // Matrix representing all links and their states: N rows x 6 columns (length, x, y, theta, x_dot, y_dot, omega)
    float L; // Default length of each link
    float mu_l; // Linear drag coefficient
    float mu_ct; // Cross-track drag coefficient
    float mu_r; // Rotational drag coefficient
    float I; // Moment of inertia of each link
    float m; // Mass of each link

    // Parameters for the spring-damper model between links
    float k; // Spring constant
    float c; // Damping coefficient

    json boom_params;


public:
    // Constructor
    Boom(size_t num_links, float L, float mu_l, float mu_ct, float mu_r, float I, float m, float k, float c);
    Boom(size_t num_links, float L);
    // Default constructor
    Boom();
    
    // Destructor
    ~Boom();

    // Assignment operator
    Boom &operator=(const Boom &boom);

    // Accessors
    void set_link_state(size_t index, const VectorXf &state); // sets the state of the link at the given index
    VectorXf get_link_state(size_t index) const; // returns the state of the link at the given index
    void set_links_states(MatrixXf &states); // set the state of all links
    float get_L() const;
    float get_I() const;
    float get_m() const;
    float get_mu_l() const;
    float get_mu_ct() const;
    float get_mu_r() const;
    float get_k() const;
    float get_c() const;   

    void load_boom_params(std::string filename);

    // Utility
    int get_num_links() const;
    void print_links_states() const;

    // Validation of state
    bool is_valid_state() const; // Check if boom doesn't intersect itself
    // State derivative function
    MatrixXf state_der(const MatrixXf &state, const Vector2f Boom_force1,
     const Vector2f Boom_force2) const;

    // // Propagation function
    // void propagate();
};

class BoomBoatsDuo {
private:
    BoomBoat boat1;
    BoomBoat boat2;
    Boom boom;
    float t; // Time [s]
    
    json boom_boats_duo_params;
    json simulation_params;

public:
    // Constructor
    BoomBoatsDuo(const BoomBoat &b1, const BoomBoat &b2, size_t num_links,
     float L, float mu_l, float mu_ct, float mu_r, float I, float m, float k,
      float c, Vector2f center, float orientation);
    BoomBoatsDuo(Vector2f center, float orientation, size_t num_links, float L);

    // Destructor
    ~BoomBoatsDuo();

    // Assignment operator
    BoomBoatsDuo &operator=(const BoomBoatsDuo &boom_boats_duo);
    
    // Utility functions
    void print_status() const;
    void print_to_file(const string &filename, const string &foldername) const;

    void load_boom_boats_duo_params(std::string filename);
    json get_simulation_params() const;
    float get_time() const;

    // Validation of state
    bool is_valid_state() const; // Check if boom doesn't intersect itself and
    bool are_boats_close() const ;// if the boats are not colliding

    // Propagation function
    MatrixXf state_der(const Vector2f &control1, const Vector2f &control2,
     MatrixXf state) const;
    void propagate(float dt, const Vector2f &control1,
     const Vector2f &control2, std::string integration_method);

    // Getters
    BoomBoat get_boat1() const;
    BoomBoat get_boat2() const;
    Boom get_boom() const;
};

// Helper functions
// Euler integration
MatrixXf Euler_integration(const MatrixXf &state, const MatrixXf &state_der,
 float dt);

// Runge-Kutta methods

// // Runge-Kutta 2nd order integration
// MatrixXf RK2_integration(const Vector2f &control1, const Vector2f &control2, 
//  const MatrixXf &state, float dt, BoomBoatsDuo boom_boats_duo);

// // Runge-Kutta 3rd order integration
// MatrixXf RK3_integration(const Vector2f &control1, const Vector2f &control2, 
//  const MatrixXf &state, float dt, BoomBoatsDuo boom_boats_duo);

// // Runge-Kutta 4th order integration
// MatrixXf RK4_integration(const Vector2f &control1, const Vector2f &control2, 
//  const MatrixXf &state, float dt, BoomBoatsDuo boom_boats_duo);

// // Runge-Kutta 5th order integration
// MatrixXf RK5_integration(const Vector2f &control1, const Vector2f &control2, 
//  const MatrixXf &state, float dt, BoomBoatsDuo boom_boats_duo);

// // Runge-Kutta 6th order integration
// MatrixXf RK6_integration(const Vector2f &control1, const Vector2f &control2, 
//  const MatrixXf &state, float dt, BoomBoatsDuo boom_boats_duo);

// // Runge-Kutta 4-5 integration. Adaptive time step
// std::pair<MatrixXf, float> RK45_integration(const Vector2f& control1,
//  const Vector2f& control2, const MatrixXf& state, float dt,
//   BoomBoatsDuo boom_boats_duo, json simulation_params);
  
// These functions are already defined in generic-boat.h 
// float wrap_theta(float theta);
// int sign(float x);

#endif
