#ifndef BOOM_BOATS_DUO_H
#define BOOM_BOATS_DUO_H

#include "boom-boat.h"
#include <Eigen/Dense>

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

    

public:
    // Constructor
    Boom(size_t num_links, float L, float mu_l, float mu_ct, float mu_r, float I, float m);
    Boom(size_t num_links, float L);
    // Default constructor
    Boom();
    
    // Destructor
    ~Boom();

    // Assignment operator
    Boom &operator=(const Boom &boom);

    // Accessors
    void set_link_state(size_t index, const VectorXf &state); // sets the state of the link at the given index
    Eigen::VectorXf get_link_state(size_t index) const; // returns the state of the link at the given index
    float get_L() const;
    float get_I() const;
    float get_m() const;
    float get_mu_l() const;
    float get_mu_ct() const;
    float get_mu_r() const;
    
    

    // Utility
    size_t get_num_links() const;
    void print_links_states() const;

    // Validation of state
    bool is_valid_state() const; // Check if boom doesn't intersect itself

};

class BoomBoatsDuo {
private:
    BoomBoat boat1;
    BoomBoat boat2;
    Boom boom;

public:
    // Constructor
    BoomBoatsDuo(const BoomBoat &b1, const BoomBoat &b2, size_t num_links,
     float L, float mu_l, float mu_ct, float mu_r, float I, float m, Vector2f center,
      float orientation);
    BoomBoatsDuo(Vector2f center, float orientation, size_t num_links, float L);

    // Destructor
    ~BoomBoatsDuo();

    // Assignment operator
    BoomBoatsDuo &operator=(const BoomBoatsDuo &boom_boats_duo);
    
    // Utility functions
    void print_status() const;
    void print_to_file(const string &filename, const string &foldername) const;

    // Propagation function
    void propagate(const Vector2f &control1, const Vector2f &control2);
};

#endif
