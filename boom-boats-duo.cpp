#include "boom-boats-duo.h"
#include <iostream>

using Eigen::Vector4f;
using Eigen::Vector2f;
using std::cout;
using std::endl;

// Boom Constructor
Boom::Boom(size_t num_links, float default_length, float mu_l, float mu_ct, float mu_r)
    : links_states(num_links, 4), mu_l(mu_l), mu_ct(mu_ct), mu_r(mu_r) {
    // Initialize all links with default values
    for (size_t i = 0; i < num_links; ++i) {
        links_states(i, 0) = default_length; // Length
        links_states(i, 1) = 0.0f;          // x position
        links_states(i, 2) = 0.0f;          // y position
        links_states(i, 3) = 0.0f;          // theta (heading)
    }
}

// Boom Destructor
Boom::~Boom() {}

// Accessors
Vector4f Boom::get_link_state(size_t index) const {
    return links_states.row(index);
}

void Boom::set_link_state(size_t index, const Vector4f &state) {
    links_states.row(index) = state;
}

float Boom::get_mu_l() const { return mu_l; }
void Boom::set_mu_l(float value) { mu_l = value; }

float Boom::get_mu_ct() const { return mu_ct; }
void Boom::set_mu_ct(float value) { mu_ct = value; }

float Boom::get_mu_r() const { return mu_r; }
void Boom::set_mu_r(float value) { mu_r = value; }

size_t Boom::get_num_links() const {
    return links_states.rows();
}

void Boom::print_links_states() const {
    cout << "Links States: \n" << links_states << endl;
}

// BoomBoatsDuo Constructor
BoomBoatsDuo::BoomBoatsDuo(const BoomBoat &b1, const BoomBoat &b2, const Boom &boom)
    : boat1(b1), boat2(b2), boom(boom) {}

// BoomBoatsDuo Destructor
BoomBoatsDuo::~BoomBoatsDuo() {}

// Utility functions
void BoomBoatsDuo::synchronize_boats() {
    // Example synchronization logic
    auto pos1 = boat1.get_pos();
    auto pos2 = boat2.get_pos();

    cout << "Synchronizing boats based on positions:\n";
    cout << "Boat 1 Position: [" << pos1[0] << ", " << pos1[1] << ", " << pos1[2] << "]\n";
    cout << "Boat 2 Position: [" << pos2[0] << ", " << pos2[1] << ", " << pos2[2] << "]\n";
}

void BoomBoatsDuo::print_status() {
    cout << "Boat 1 Status:\n";
    boat1.print_params();

    cout << "Boat 2 Status:\n";
    boat2.print_params();

    cout << "Boom Status:\n";
    boom.print_links_states();
}

void BoomBoatsDuo::propagate(const Vector2f &control1, const Vector2f &control2) {
    try {
        // Propagate individual boats
        boat1.propogate(control1, 0.1f); // Example time step of 0.1 seconds
        boat2.propogate(control2, 0.1f);

        // Adjust boom states if necessary
        for (size_t i = 0; i < boom.get_num_links(); ++i) {
            Vector4f link_state = boom.get_link_state(i);
            link_state(1) += 0.01f; // Example modification for x position
            link_state(2) += 0.01f; // Example modification for y position
            boom.set_link_state(i, link_state);
        }

        cout << "Propagation complete. Updated states:\n";
        print_status();
    } catch (const std::exception &e) {
        cout << "Error during propagation: " << e.what() << endl;
    }
}
