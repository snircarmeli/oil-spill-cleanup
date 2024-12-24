#include "boom-boats-duo.h"
#include <iostream>

using Eigen::VectorXf;
using Eigen::Vector2f;
using Eigen::MatrixXf;
using std::cout;
using std::endl;

// Boom Constructor
Boom::Boom(size_t num_links, float L, float mu_l, float mu_ct, float mu_r)
    : links_states(num_links, 6), L(L), mu_l(mu_l), mu_ct(mu_ct), mu_r(mu_r) {
       links_states = MatrixXf::Zero(num_links, 6);
    }
    

// Boom Destructor
Boom::~Boom() {}

// Accessors
void Boom::set_link_state(size_t index, const VectorXf &state) {
    links_states.row(index) = state;
}

VectorXf Boom::get_link_state(size_t index) const {
    return links_states.row(index);
}

float Boom::get_L() const { return L; }

float Boom::get_mu_l() const { return mu_l; }

float Boom::get_mu_ct() const { return mu_ct; }

float Boom::get_mu_r() const { return mu_r; }

size_t Boom::get_num_links() const {
    return links_states.rows();
}

void Boom::print_links_states() const {
    cout << "Links States: \n" << links_states << endl;
}

// Function to check if two line segments intersect
    bool check_intersection(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) {
        // Calculate the direction of the lines
        float d1 = direction(x3, y3, x4, y4, x1, y1);
        float d2 = direction(x3, y3, x4, y4, x2, y2);
        float d3 = direction(x1, y1, x2, y2, x3, y3);
        float d4 = direction(x1, y1, x2, y2, x4, y4);
    
        // Check if the line segments intersect
        if (( d1 * d2 < 0 ) && ( d3 * d4 < 0 )) {
            return true;
        }
    
        // Check if the points are collinear and on the segment
        if (d1 == 0 && on_segment(x3, y3, x4, y4, x1, y1)) return true;
        if (d2 == 0 && on_segment(x3, y3, x4, y4, x2, y2)) return true;
        if (d3 == 0 && on_segment(x1, y1, x2, y2, x3, y3)) return true;
        if (d4 == 0 && on_segment(x1, y1, x2, y2, x4, y4)) return true;
    
        return false;
    }
    
    // Helper function to calculate the direction
    float direction(float xi, float yi, float xj, float yj, float xk, float yk) {
        return (xk - xi) * (yj - yi) - (yk - yi) * (xj - xi);
    }
    
    // Helper function to check if a point is on a segment
    bool on_segment(float xi, float yi, float xj, float yj, float xk, float yk) {
        return std::min(xi, xj) <= xk && xk <= std::max(xi, xj) && std::min(yi, yj) <= yk && yk <= std::max(yi, yj);
    }

bool Boom::is_valid_state() const {
    // Check if boom doesn't intersect itself

    for (size_t i = 0; i < get_num_links(); ++i) {

        float x_i = links_states(i, 1);
        float y_i = links_states(i, 2);
        float theta_i = links_states(i, 3);
        float L = get_L();

        // Coordinates of boom link i
        float x_11 = x_i + 0.5 * L * cos(theta_i);
        float y_11 = y_i + 0.5 * L * sin(theta_i);
        float x_12 = x_i - 0.5 * L * cos(theta_i);
        float y_12 = y_i - 0.5 * L * sin(theta_i); 

        for (size_t j = i + 1; j < get_num_links(); ++j) {
            float x_j = links_states(j, 1);
            float y_j= links_states(j, 2);
            float theta_j = links_states(j, 3);
            L = get_L();
            // Coordinates of boom link j
            float x_21 = x_j + 0.5 * L * cos(theta_j);
            float y_21 = y_j + 0.5 * L * sin(theta_j);
            float x_22 = x_j - 0.5 * L * cos(theta_j);
            float y_22 = y_j - 0.5 * L * sin(theta_j);

            // Check if the two links intersect
            if ( j == i + 1) {
                // Check if the two links completely overlap
                if (x_11 == x_21 && y_11 == y_21 && x_22 == x_12 && y_22 == y_12) {
                    return false;
                }
            } else {
                return !check_intersection(x_11, y_11, x_12, y_12, x_21, y_21, x_22, y_22);
            }

        }
    }
    return true;
}


// BoomBoatsDuo Constructor
BoomBoatsDuo::BoomBoatsDuo(const BoomBoat &b1, const BoomBoat &b2, size_t num_links, float L, float mu_l, float mu_ct, float mu_r) {
        // // Check if distance between boats is greater than the sum of the boom links
        // float dist = (boat1.get_pos().head<2>() - boat2.get_pos().head<2>()).norm();
        // float boom_length = num_links * L;
        // if (dist > boom_length) {
        //     throw std::invalid_argument("Distance between boats is greater than the sum of the boom links");
        // }
        Boom boom(num_links, L, mu_l, mu_ct, mu_r);
        BoomBoat boat1 = BoomBoat(b1.get_radius(), b1.get_mass(), 
        b1.get_inertia(), b1.get_mu_l(), b1.get_mu_ct(), b1.get_mu_r(),
         b1.get_fuel(), b1.get_cap(), b1.get_F_max(), b1.get_eta_max());

        // Place boom links on an arc between the two boats
        for (size_t i = 0; i < boom.get_num_links(); ++i) {
            
        }
    }



// BoomBoatsDuo Destructor
BoomBoatsDuo::~BoomBoatsDuo() {}

// Utility functions

void BoomBoatsDuo::print_status() {
    cout << "Boat 1 State: \n";
    boat1.print_status();
    cout << "Boat 2 State: \n";
    boat2.print_status();
    cout << "Boom State: \n";
    boom.print_links_states();
}

// Propagation function
void BoomBoatsDuo::propagate(const Vector2f &control1, const Vector2f &control2) {
    
}