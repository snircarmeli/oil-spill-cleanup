#include "boom-boats-duo.h"
#include "boom-boat.h"
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem> // For filesystem operations (C++17 and later)

using Eigen::VectorXf;
using Eigen::Vector2f;
using Eigen::MatrixXf;
using std::cout;
using std::endl;
using std::string;

namespace fs = std::filesystem;

const float PI = 3.141592653589793;

// Boom Constructor
Boom::Boom(size_t num_links, float L, float mu_l, float mu_ct, float mu_r)
    : L(L), mu_l(mu_l), mu_ct(mu_ct), mu_r(mu_r) {
       links_states = MatrixXf::Zero(num_links, 6);
    }

// Need to decide on friction parameters
Boom::Boom(size_t num_links, float L) : L(L), mu_l(0.0), mu_ct(0.0), mu_r(0.0) {
    links_states = MatrixXf::Zero(num_links, 6);
}

// Default Constructor
Boom::Boom() : L(0.0), mu_l(0.0), mu_ct(0.0), mu_r(0.0) {
    links_states = MatrixXf::Zero(0, 6);
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

// Functions to check if two line segments intersect

    // Helper function to calculate the direction
    float direction(float xi, float yi, float xj, float yj, float xk, float yk) {
        return (xk - xi) * (yj - yi) - (yk - yi) * (xj - xi);
    }
    
    // Helper function to check if a point is on a segment
    bool on_segment(float xi, float yi, float xj, float yj, float xk, float yk) {
        return std::min(xi, xj) <= xk && xk <= std::max(xi, xj) && std::min(yi, yj) <= yk && yk <= std::max(yi, yj);
    }

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
BoomBoatsDuo::BoomBoatsDuo(const BoomBoat &b1, const BoomBoat &b2,
 size_t num_links, float L, float mu_l, float mu_ct, float mu_r,
  Vector2f center, float orientation) {
        // // Check if distance between boats is greater than the sum of the boom links
        // float dist = (boat1.get_pos().head<2>() - boat2.get_pos().head<2>()).norm();
        // float boom_length = num_links * L;
        // if (dist > boom_length) {
        //     throw std::invalid_argument("Distance between boats is greater than the sum of the boom links");
        // }
        Boom boom(num_links, L, mu_l, mu_ct, mu_r);
        BoomBoat boat1 = BoomBoat(b1.get_radius(), b1.get_mass(), 
        b1.get_inertia(), b1.get_mu_l(), b1.get_mu_ct(), b1.get_mu_r(),
         b1.get_pos(), b1.get_vel(), b1.get_fuel(), b1.get_cap(),
          b1.get_F_max(), b1.get_eta_max());
          
        BoomBoat boat2 = BoomBoat(b2.get_radius(), b2.get_mass(), 
        b2.get_inertia(), b2.get_mu_l(), b2.get_mu_ct(), b2.get_mu_r(),
         b2.get_pos(), b2.get_vel(), b2.get_fuel(), b2.get_cap(),
          b2.get_F_max(), b2.get_eta_max());

        // Place boats at the center with the given orientation of the line 
        // connecting the boats
        float b1_theta_orig = boat1.get_pos()(2);
        float b2_theta_orig = boat2.get_pos()(2);
        Vector3f b1_pos = Vector3f(center(0) - 0.5 * num_links * L * cos(orientation), center(1) - 0.5 * N *L * sin(orientation), b1_theta_orig);
        Vector3f b2_pos = Vector3f(center(0) + 0.5 * num_links * L * cos(orientation), center(1) + 0.5 * N *L * sin(orientation), b2_theta_orig);

        Vector3f b1_stern = Vector3f(b1_pos(0) - b1.get_radius() * sin(b1_pos(2)), b1_pos(1) - b1.get_radius() * cos(b1_pos(2)), b1_pos(2));
        Vector3f b2_stern = Vector3f(b2_pos(0) - b2.get_radius() * sin(b2_pos(2)), b2_pos(1) - b2.get_radius() * cos(b2_pos(2)), b2_pos(2));

        boat1.set_pos(b1_pos);
        boat2.set_pos(b2_pos);
        for (size_t i = 0; i < boom.get_num_links(); ++i) {
            float x = b1_stern(0) + (i + 0.5) * L * cos(orientation);
            float y = b2_stern(1) + (i + 0.5) * L * sin(orientation);
            float theta = orientation;
            VectorXf state(6);
            state << x, y, theta, 0.0, 0.0, 0.0;
            boom.set_link_state(i, state);
        }
    }

BoomBoatsDuo:: BoomBoatsDuo(Vector2f center,  float orientation,
 size_t num_links, float L) {
    Boom boom(num_links, L);
    Vector3f boat1_pos = Vector3f(center(0) - L * num_links * cos(orientation), 
    center(1) - L * num_links * sin(orientation), PI/2 + orientation);
    Vector3f boat2_pos = Vector3f(center(0) + L * num_links * cos(orientation), 
    center(1) + L * num_links * sin(orientation), PI/2 + orientation);

    BoomBoat boat1 = BoomBoat(boat1_pos);
    BoomBoat boat2 = BoomBoat(boat2_pos);
}


// BoomBoatsDuo Destructor
BoomBoatsDuo::~BoomBoatsDuo() {}

// Utility functions

void BoomBoatsDuo::print_status() const {
    cout << "Boat 1 State: \n";
    boat1.print_status();
    cout << "Boat 2 State: \n";
    boat2.print_status();
    cout << "Boom State: \n";
    boom.print_links_states();
}

void BoomBoatsDuo::print_to_file(const string &filename,
 const string &foldername) const {
    // Check if the folder exists
    if (!fs::exists(foldername)) {
        if (!fs::create_directory(foldername)) {
            std::cerr << "Failed to create directory" << std::endl;
            return;
        }
    }
    // Construct the full file path
    string filepath = foldername + "/" + filename;

    // Open the file in append mode
    std::ofstream file(filepath, std::ios_base::app);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return;
    }


    // Write the state to a file: first 6 numbers are boat1.get_pos(),
    // boat1.get_vel(), second 6 numbers are boat2.get_pos. After that num_links
    // and L, and then a sequence of every link's position parameters

    // Variable to hold all data
    std::stringstream ss;

    // Write the state of boat1
    Vector3f boat1_pos = boat1.get_pos();
    Vector3f boat1_vel = boat1.get_vel();
    ss << boat1_pos.transpose() << " ";
    ss << boat1_vel.transpose() << " ";

    // Write the state of boat2
    Vector3f boat2_pos = boat2.get_pos();
    Vector3f boat2_vel = boat2.get_vel();
    ss << boat2_pos.transpose() << " ";
    ss << boat2_vel.transpose() << " ";

    // Write about the boom
    ss << boom.get_num_links() << " ";
    ss << boom.get_L() << " ";

    // Write the state of the boom links
    for (size_t i = 0; i < boom.get_num_links(); ++i) {
        VectorXf link_state = boom.get_link_state(i);
        ss << link_state.transpose() << " ";
    }

    // Write the collected string to the file
    file << ss.str() << std::endl;

    file.close();
}


// Propagation function
void BoomBoatsDuo::propagate(const Vector2f &control1, const Vector2f &control2) {
    // Example
    // boat1.set_pos(boat1.get_pos() + control1);
    // boat2.set_pos(boat2.get_pos() + control2);
    // VectorXf control(3);
    // control << control1(0), control1(1), control2(0); // Taking the first three positional arguments
}