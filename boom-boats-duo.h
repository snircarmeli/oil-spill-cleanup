#ifndef BOOM_BOATS_DUO_H
#define BOOM_BOATS_DUO_H

#include "boom-boat.h"
#include <Eigen/Dense>

using Eigen::MatrixXf;
using Eigen::Vector2f;

class Boom {
private:
    MatrixXf links_states; // Matrix representing all links and their states: N rows x 4 columns (length, x, y, theta)
    float mu_l; // Linear drag coefficient
    float mu_ct; // Cross-track drag coefficient
    float mu_r; // Rotational drag coefficient

public:
    // Constructor
    Boom(size_t num_links, float default_length, float mu_l, float mu_ct, float mu_r);

    // Destructor
    ~Boom();

    // Accessors
    Eigen::Vector4f get_link_state(size_t index) const;
    void set_link_state(size_t index, const Eigen::Vector4f &state);

    float get_mu_l() const;
    void set_mu_l(float value);
    float get_mu_ct() const;
    void set_mu_ct(float value);
    float get_mu_r() const;
    void set_mu_r(float value);

    // Utility
    size_t get_num_links() const;
    void print_links_states() const;
};

class BoomBoatsDuo {
private:
    BoomBoat boat1;
    BoomBoat boat2;
    Boom boom;

public:
    // Constructor
    BoomBoatsDuo(const BoomBoat &b1, const BoomBoat &b2, const Boom &boom);

    // Destructor
    ~BoomBoatsDuo();

    // Utility functions
    void synchronize_boats();
    void print_status();

    // Propagation function
    void propagate(const Vector2f &control1, const Vector2f &control2);
};

#endif
