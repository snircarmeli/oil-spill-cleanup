#include "boom-boats-duo.h"
#include "boom-boat.h"
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem> // For filesystem operations (C++17 and later)

// For JSON parameters parsing
#include "json/json.hpp"
using json = nlohmann::json;

using Eigen::VectorXf;
using Eigen::Vector2f;
using Eigen::MatrixXf;
using std::cout;
using std::endl;
using std::string;

namespace fs = std::filesystem;

const float PI = 3.141592653589793;

float K_deault = 10000000000000;
float C_default = 1000000;

// Boom Constructor
Boom::Boom(size_t num_links, float L, float mu_l, float mu_ct, float mu_r,
 float I, float m, float k, float c)
    : L(L), mu_l(mu_l), mu_ct(mu_ct), mu_r(mu_r), I(I), m(m), k(k), c(c) {
       links_states = MatrixXf::Zero(num_links, 6);
    }

Boom::Boom(size_t num_links, float L) {
    // Read parameters from JSON file
    std::string params_file = "params.json";
    std::ifstream file(params_file);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open params file");
    }
    json params;
    file >> params;
    file.close();

    links_states = MatrixXf::Zero(num_links, 6);
    this->L = L;
    
    json boom_params = params["boom"];
    json drag_params = boom_params["drag_coefficients"];
    // Extract parameters from JSON
    this->mu_l = drag_params["linear"].get<float>();
    this->mu_ct = drag_params["cross_track"].get<float>();
    this->mu_r = drag_params["rotational"].get<float>();
    this->I = boom_params["inertia"].get<float>();
    this->m = boom_params["mass"].get<float>();
    this->k = boom_params["spring_constant"].get<float>();
    this->c = boom_params["damping_coefficient"].get<float>();

    links_states = MatrixXf::Zero(num_links, 6);
}

// Default Constructor
Boom::Boom() {
    // Read parameters from JSON file
    std::string params_file = "params.json";
    std::ifstream file(params_file);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open params file");
    }
    json params;
    file >> params;
    file.close();

    json boom_params = params["boom"];
    json drag_params = boom_params["drag_coefficients"];
    
    // Extract parameters from JSON
    this->L = boom_params["link_length"].get<float>();
    this->mu_l = drag_params["linear"].get<float>();
    this->mu_ct = drag_params["cross_track"].get<float>();
    this->mu_r = drag_params["rotational"].get<float>();
    this->I = boom_params["inertia"].get<float>();
    this->m = boom_params["mass"].get<float>();
    this->k = boom_params["spring_constant"].get<float>();
    this->c = boom_params["damping_coefficient"].get<float>();
    size_t num_links = boom_params["num_links"].get<size_t>();

    links_states = MatrixXf::Zero(num_links, 6);
}

// Boom Destructor
Boom::~Boom() {}

// Assignment Operator
Boom& Boom::operator=(const Boom& other) {
    if (this != &other) {
        L = other.L;
        mu_l = other.mu_l;
        mu_ct = other.mu_ct;
        mu_r = other.mu_r;
        I = other.I;
        m = other.m;
        k = other.k;
        c = other.c;
        links_states = other.links_states;
    }
    return *this;
}

// Accessors
void Boom::set_link_state(size_t index, const VectorXf &state) {
    links_states.row(index) = state;
}

VectorXf Boom::get_link_state(size_t index) const {
    return links_states.row(index);
}

void Boom::set_links_states(MatrixXf &states) {
    for (size_t i = 0; i < static_cast<size_t>(states.rows()); ++i) {
        this->links_states.row(i) = states.row(i);
    }
}

float Boom::get_L() const { return L; }

float Boom::get_I() const { return I; }

float Boom::get_m() const { return m; }

float Boom::get_mu_l() const { return mu_l; }

float Boom::get_mu_ct() const { return mu_ct; }

float Boom::get_mu_r() const { return mu_r; }

float Boom::get_k() const { return k; }

float Boom::get_c() const { return c; }

int Boom::get_num_links() const {
    return static_cast<int>(links_states.rows());
}

void Boom::print_links_states() const {
    cout << "Boom Length: " << L << endl;
    cout << "Number of Links: " << links_states.rows() << endl;
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

    for (int i = 0; i < get_num_links(); ++i) {

        float x_i = links_states(i, 1);
        float y_i = links_states(i, 2);
        float theta_i = links_states(i, 3);
        float L = get_L();

        // Coordinates of boom link i
        float x_11 = x_i + 0.5 * L * cos(theta_i);
        float y_11 = y_i + 0.5 * L * sin(theta_i);
        float x_12 = x_i - 0.5 * L * cos(theta_i);
        float y_12 = y_i - 0.5 * L * sin(theta_i); 

        // Starting from the second next link. Every two adjacent links can
        // intersect because of the spring-damper model.
        for (int j = i + 2; j < get_num_links(); ++j) {
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

// State derivative function
MatrixXf Boom::state_der(const MatrixXf &state, const Vector2f Boom_force1,
 const Vector2f Boom_force2) const {
    // Model the dynamics of the boom links as spring and dampers
    int num_links = this->get_num_links();
    float L = this->get_L();
    float mu_l = this->get_mu_l();
    float mu_ct = this->get_mu_ct();
    float mu_r = this->get_mu_r();
    float I = this->get_I();
    float m = this->get_m();
    float k = this->get_k();
    float c = this->get_c();

    // 2 boats and num_links links, 6 states each
    MatrixXf state_der = MatrixXf::Zero(num_links, 6);

    // state is of same order as state_der

    for (int i = 0; i < num_links; ++i) {

        // float x_i = state(i, 0);
        // float y_i = state(i, 1);
        float theta = state(i, 2);
        float x_dot_i = state(i, 3);
        float y_dot_i = state(i, 4);
        float theta_dot_i = state(i, 5);

        Vector2f link_vel = Vector2f(x_dot_i, y_dot_i);

        // P0 - left enf of link i
        // P1 - right end of link i
        Vector2f P0 = Vector2f(state(i, 0), state(i, 1)) - (L / 2) * Vector2f(cos(state(i, 2)), sin(state(i, 2)));
        Vector2f P1 = Vector2f(state(i, 0), state(i, 1)) + (L / 2) * Vector2f(cos(state(i, 2)), sin(state(i, 2)));
        Vector2f P0_dot = Vector2f(state(i, 3), state(i, 4)) - (L / 2) * state(i, 5) * Vector2f(-sin(state(i, 2)), cos(state(i, 2)));
        Vector2f P1_dot = Vector2f(state(i, 3), state(i, 4)) + (L / 2) * state(i, 5) * Vector2f(-sin(state(i, 2)), cos(state(i, 2)));

        // Normalized vector in the direction of the link
        Vector2f e = (P1 - P0).normalized();
        // Normalized vector perpendicular to the link, rotate v by 90 degrees in the counter-clockwise direction   
        Vector2f n = Vector2f(-e(1), e(0));

        // Calculate the forces on the link
        // F0 - force on the left end of the link
        // F1 - force on the right end of the link
        Vector2f F0 = Vector2f(0, 0);
        Vector2f F1 = Vector2f(0, 0);

        // M - right end of link i-1
        // N - left end of link i+1
        if (i != 0) {
            Vector2f M = Vector2f(state(i - 1, 0), state(i - 1, 1)) + (L / 2) * Vector2f(cos(state(i - 1, 2)), sin(state(i - 1, 2)));
            Vector2f M_dot = Vector2f(state(i - 1, 3), state(i - 1, 4)) + (L / 2) * state(i - 1, 5) * Vector2f(-sin(state(i - 1, 2)), cos(state(i - 1, 2)));

            Vector2f F_spring_0 = k * (M - P0);
            // unit vector in the direction of (M-P0)
            Vector2f e0 = (M-P0).normalized();
            Vector2f F_damp_0 = c * (M_dot - P0_dot).dot(e0) * e0;
            F0 = F_spring_0 + F_damp_0;
        } else {F0 = Boom_force1;}

        if (i != num_links - 1) {
            Vector2f N = Vector2f(state(i + 1, 0), state(i + 1, 1)) - (L / 2) * Vector2f(cos(state(i + 1, 2)), sin(state(i + 1, 2)));
            Vector2f N_dot = Vector2f(state(i + 1, 3), state(i + 1, 4)) - (L / 2) * state(i + 1, 5) * Vector2f(-sin(state(i + 1, 2)), cos(state(i + 1, 2)));

            Vector2f F_spring_1 = k * (N -P1);
            // unit vector in the direction of (N-P1)
            Vector2f e1 = (N - P1).normalized();
            Vector2f F_damp_1 = c * (N_dot - P1_dot).dot(e1) * e1;
            F1 = F_spring_1 + F_damp_1;
        } else {F1 = Boom_force2;}

        // Calculate the torques on the link 
        // (Cross product of the force and the position vector)
        float torque = 0.5 * L * (e.x() * (F1 - F0).y() - e.y() * (F1 - F0).x());

        // Calculate the state derivatives
        // Rotate to n and e frame
        float vel_e = link_vel.dot(e);
        float vel_n = link_vel.dot(n);
        float F_e = (F0 + F1).dot(e);
        float F_n = (F0 + F1).dot(n);
        // Calculate the state derivatives
        float vel_e_dot = (1 / m) * (F_e - mu_l * (vel_e * vel_e) * sign(vel_e));
        float vel_n_dot = (1 / m) * (F_n - mu_ct * (vel_n * vel_n) * sign(vel_n));
        float omega_dot = (1 / I) * (torque - mu_r * (theta_dot_i * theta_dot_i) * sign(theta_dot_i));
        // Rotate back to the global frame
        float x_dotdot = vel_e_dot * cos(theta) - vel_n_dot * sin(theta);
        float y_dotdot = vel_e_dot * sin(theta) + vel_n_dot * cos(theta);
        float theta_dotdot = omega_dot;
        // Store the state derivatives
        state_der(i, 0) = x_dot_i;  // x_dot
        state_der(i, 1) = y_dot_i;  // y_dot
        state_der(i, 2) = theta_dot_i;  // theta_dot
        state_der(i, 3) = x_dotdot;  // x_dotdot
        state_der(i, 4) = y_dotdot;  // y_dotdot
        state_der(i, 5) = theta_dotdot;  // theta_dotdot

    }

    return state_der;
}



// BoomBoatsDuo Constructor
BoomBoatsDuo::BoomBoatsDuo(const BoomBoat &b1, const BoomBoat &b2,
 size_t num_links, float L, float mu_l, float mu_ct, float mu_r,
  float I, float m, float k, float c, Vector2f center, float orientation)
    : boat1(b1.get_radius(), b1.get_mass(), b1.get_inertia(), b1.get_mu_l(), b1.get_mu_ct(), b1.get_mu_r(),
            b1.get_pos(), b1.get_vel(), b1.get_fuel(), b1.get_cap(), b1.get_F_max(), b1.get_eta_max()),
      boat2(b2.get_radius(), b2.get_mass(), b2.get_inertia(), b2.get_mu_l(), b2.get_mu_ct(), b2.get_mu_r(),
            b2.get_pos(), b2.get_vel(), b2.get_fuel(), b2.get_cap(), b2.get_F_max(), b2.get_eta_max()),
            boom(num_links, L, mu_l, mu_ct, mu_r, I, m, k, c) {

        // Place boats at the center with the given orientation of the line 
        // connecting the boats

        Vector3f b1_pos = Vector3f(center(0) - 0.5 * num_links * L * cos(orientation), center(1) - 0.5 * num_links * L * sin(orientation), orientation);
        Vector3f b2_pos = Vector3f(center(0) + 0.5 * num_links * L * cos(orientation), center(1) + 0.5 * num_links * L * sin(orientation), orientation);

        // Vector3f b1_stern = Vector3f(b1_pos(0) - b1.get_radius() * sin(b1_pos(2)), b1_pos(1) - b1.get_radius() * cos(b1_pos(2)), b1_pos(2));
        // Vector3f b2_stern = Vector3f(b2_pos(0) - b2.get_radius() * sin(b2_pos(2)), b2_pos(1) - b2.get_radius() * cos(b2_pos(2)), b2_pos(2));

        boat1.set_pos(b1_pos);
        boat2.set_pos(b2_pos);
        for (int i = 0; i < boom.get_num_links(); ++i) {
            float x = b1_pos(0) + (i + 0.5) * L * cos(orientation);
            float y = b1_pos(1) + (i + 0.5) * L * sin(orientation);
            float theta = orientation;
            VectorXf state(6);
            state << x, y, theta, 0.0, 0.0, 0.0;
            boom.set_link_state(i, state);
        }
    }

BoomBoatsDuo:: BoomBoatsDuo(Vector2f center,  float orientation,
 size_t num_links, float L) {
    Boom boom(num_links, L);
    this->boom = boom;
    Vector3f boat1_pos = Vector3f(center(0) - L * num_links * cos(orientation), 
    center(1) - L * num_links * sin(orientation), PI/2 + orientation);
    Vector3f boat2_pos = Vector3f(center(0) + L * num_links * cos(orientation), 
    center(1) + L * num_links * sin(orientation), PI/2 + orientation);

    BoomBoat boat1 = BoomBoat(boat1_pos);
    BoomBoat boat2 = BoomBoat(boat2_pos);
    this->boat1 = boat1;
    this->boat2 = boat2;

}


// BoomBoatsDuo Destructor
BoomBoatsDuo::~BoomBoatsDuo() {}

// Assignment Operator
BoomBoatsDuo& BoomBoatsDuo::operator=(const BoomBoatsDuo &other) {
    if (this != &other) {
        boat1 = other.boat1;
        boat2 = other.boat2;
        boom = other.boom;
    }
    return *this;
}

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
    // boat1.get_vel(), next two numbers are Force F and steering angle eta.
    // next, 6 numbers are boat2.get_pos and boat2.get_vel(). next two numbers 
    // are force F and steering angle eta. After that num_links
    // and L, and then a sequence of every link's position parameters

    // Variable to hold all data
    std::stringstream ss;

    // Write the state of boat1
    Vector3f boat1_pos = boat1.get_pos();
    Vector3f boat1_vel = boat1.get_vel();
    Vector2f boat1_control = boat1.get_control();
    ss << boat1_pos.transpose() << " ";
    ss << boat1_vel.transpose() << " ";
    ss << boat1_control.transpose() << " ";

    // Write the state of boat2
    Vector3f boat2_pos = boat2.get_pos();
    Vector3f boat2_vel = boat2.get_vel();
    Vector2f boat2_control = boat2.get_control();
    ss << boat2_pos.transpose() << " ";
    ss << boat2_vel.transpose() << " ";
    ss << boat2_control.transpose() << " ";

    // Write about the boom
    ss << boom.get_num_links() << " ";
    ss << boom.get_L() << " ";

    // Write the state of the boom links
    for (int i = 0; i < boom.get_num_links(); ++i) {
        VectorXf link_state = boom.get_link_state(i);
        ss << link_state.transpose() << " ";
    }
    // Write the collected string to the file
    file << ss.str() << std::endl;

    file.close();
}

// Validation of states
bool BoomBoatsDuo::is_valid_state() const {
    // Check if the boom doesn't intersect itself
    if (!this->boom.is_valid_state()) {
        cout << "Boom intersects itself" << endl;
        cout.flush();
        return false;
    }

    // Check if the boats don't intersect each other
    // Load parameter ["boom_boats_duo"]["minimum_distance"]
    std::string params_file = "params.json";
    std::ifstream file(params_file);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open params file");
    }
    json params;
    file >> params;
    file.close();
    float min_distance = params["boom_boats_duo"]["minimum_distance"];
    Vector2f boat1_pos = boat1.get_pos().head(2);
    Vector2f boat2_pos = boat2.get_pos().head(2);
    float distance = (boat1_pos - boat2_pos).norm();
    if (distance < min_distance) {
        cout << "Boats are too close" << endl;
        cout.flush();
        return false;
        }
    return true;

}

// Propagation function
MatrixXf BoomBoatsDuo::state_der(const Vector2f &control1,
 const Vector2f &control2, MatrixXf state) const {
    // Model the dynamics of the boom links as spring and dampers
    int num_links = boom.get_num_links();
    float L = this->boom.get_L();

    // 2 boats and num_links links, 6 states each
    MatrixXf state_der = MatrixXf::Zero(state.rows(), state.cols());

    // state is of same order as state_der
    // state = [boat1; boat2; link1; link2; ...; link_num_links]

    // Calculate state derivative of boat1

    Vector2f boat1_pos = Vector2f(state(0, 0), state(0, 1));
    Vector2f boat1_vel = Vector2f(state(0, 3), state(0, 4));
    // P0: left end of link 1 - closest link to boat1
    float theta_link = state(2, 2);
    Vector2f P0 = Vector2f(state(2, 0), state(2, 1)) - (L / 2) * Vector2f(cos(theta_link), sin(theta_link));
    Vector2f P0_dot = Vector2f(state(2, 3), state(2, 4)) - (L / 2) * state(2, 5) * Vector2f(-sin(theta_link), cos(theta_link));

    Vector2f F_spring1 = this->boom.get_k() * (P0 - boat1_pos);
    // unit vector in the direction of (P0 - boat1_pos)
    Vector2f e1 = (P0 - boat1_pos).normalized();
    Vector2f F_damp1 = this->boom.get_c() * ((P0_dot - boat1_vel).dot(e1)) * e1;
    Vector2f boom_force1 = F_spring1 + F_damp1;

    VectorXf boat1_state_der = this->boat1.state_der(VectorXf(state.row(0)),
     control1, boom_force1);

    // Calculate state derivative of boat2
    Vector2f boat2_pos = Vector2f(state(1, 0), state(1, 1));
    Vector2f boat2_vel = Vector2f(state(1, 3), state(1, 4));
    // P1: right end of link num_links - closest link to boat2
    theta_link = state(num_links + 1, 2);
    Vector2f P1 = Vector2f(state(num_links + 1, 0), state(num_links + 1, 1)) + (L / 2) * Vector2f(cos(theta_link), sin(theta_link));
    Vector2f P1_dot = Vector2f(state(num_links + 1, 3), state(num_links + 1, 4)) + (L / 2) * state(num_links + 1, 5) * Vector2f(-sin(theta_link), cos(theta_link));

    Vector2f F_spring2 = this->boom.get_k() * (P1 - boat2_pos);
    // unit vector in the direction of (P1 - boat2_pos)
    Vector2f e2 = (P1 - boat2_pos).normalized();
    Vector2f F_damp2 = this->boom.get_c() * ((P1_dot - boat2_vel).dot(e2)) * e2;
    Vector2f boom_force2 = F_spring2 + F_damp2;

    VectorXf boat2_state_der = this->boat2.state_der(VectorXf(state.row(1)),
     control2, boom_force2);
 
    // Calculate state derivative of the boom links
    // Use boom state derivative function
    MatrixXf boom_state = MatrixXf::Zero(num_links, 6);
    // take all the lines from the state matrix: third line and onwards
    for (int i = 0; i < num_links; i++) {
        boom_state.row(i) = state.row(i + 2);
    }
    MatrixXf boom_state_der = MatrixXf::Zero(num_links, 6);
    // Enter forces in negative sign as the boom is applying the force on the boats
    boom_state_der = this->boom.state_der(boom_state, -boom_force1, -boom_force2);

    // Combine the state derivatives
    state_der.row(0) = boat1_state_der.transpose();
    state_der.row(1) = boat2_state_der.transpose();
    // set states' derivatives of links: rows 2-end
    for (int i = 0; i < num_links; i++) {
        state_der.row(i + 2) = boom_state_der.row(i);
    }
    state_der.block(2, 0, num_links, 6) = boom_state_der;

    return state_der;
}

// Propagation function
void BoomBoatsDuo::propagate(float dt, const Vector2f &control1,
 const Vector2f &control2, std::string integration_method) {
    // Calculate state derivative
    MatrixXf state = MatrixXf::Zero(2 + this->boom.get_num_links(), 6);
    state.row(0).head(3) = this->boat1.get_pos().transpose();
    state.row(0).tail(3) = this->boat1.get_vel().transpose();
    state.row(1).head(3) = this->boat2.get_pos().transpose();
    state.row(1).tail(3) = this->boat2.get_vel().transpose();

    MatrixXf state_new = MatrixXf::Zero(state.rows(), state.cols());
    // set states of links: rows 2-end
    for (int i = 0; i < this->boom.get_num_links(); i++) {
        state.row(i + 2) = this->boom.get_link_state(i).transpose();
    }

    if (integration_method == "RK4") {
        state_new = RK4_integration(control1, control2, state, dt, *this);
        }  else if (integration_method == "RK45") {
            state_new = RK45_integration(control1, control2, state, dt, *this);
        } else if (integration_method == "Euler") {
        state_new = Euler_integration(state, this->state_der(control1, control2, state), dt);
    } else {
        throw std::runtime_error("Invalid integration method: " + integration_method);
    }

    // Update the state using Runge-Kutta 4
    // MatrixXf k1 = this->state_der(control1, control2, state);
    // MatrixXf k2 = this->state_der(control1, control2, state + (dt / 2) * k1);
    // MatrixXf k3 = this->state_der(control1, control2, state + (dt / 2) * k2);
    // MatrixXf k4 = this->state_der(control1, control2, state + dt * k3);
    // state_new = state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
    
    // // Update the state using forward Euler
    // MatrixXf state_der = this->state_der(control1, control2, state);
    // state_new = state + dt * state_der;

    // wrap theta for all states, third column
    for (int i = 0; i < state_new.rows(); i++) {
        state_new(i, 2) = wrap_theta(state_new(i, 2));
    }

    this->boat1.set_pos(state_new.row(0).head(3).transpose());
    this->boat1.set_vel(state_new.row(0).tail(3).transpose());
    this->boat2.set_pos(state_new.row(1).head(3).transpose());
    this->boat2.set_vel(state_new.row(1).tail(3).transpose());
    // set states of links: rows 2-end
    for (int i = 0; i < this->boom.get_num_links(); i++) {
        this->boom.set_link_state(i, state_new.row(i + 2).transpose());
    }
}



// Helper functions 
// Euler integration
MatrixXf Euler_integration(const MatrixXf &state, const MatrixXf &state_der,
 float dt) {
    return state + dt * state_der;
}

// Runge-Kutta 4 integration
MatrixXf RK4_integration(const Vector2f& control1, const Vector2f& control2,
 const MatrixXf& state, float dt, BoomBoatsDuo boom_boats_duo) {
    MatrixXf k1 = boom_boats_duo.state_der(control1, control2, state);
    MatrixXf k2 = boom_boats_duo.state_der(control1, control2, state + (dt / 2) * k1);
    MatrixXf k3 = boom_boats_duo.state_der(control1, control2, state + (dt / 2) * k2);
    MatrixXf k4 = boom_boats_duo.state_der(control1, control2, state + dt * k3);

    return state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
}

// Runge-Kutta 4-5 integration
MatrixXf RK45_integration(const Vector2f& control1, const Vector2f& control2,
 const MatrixXf& state, float dt, BoomBoatsDuo boom_boats_duo) {
    MatrixXf k1 = boom_boats_duo.state_der(control1, control2, state);
    MatrixXf k2 = boom_boats_duo.state_der(control1, control2,
     state + (dt / 4) * k1);
    MatrixXf k3 = boom_boats_duo.state_der(control1, control2,
     state + (3 * dt / 8) * k1 + (9 * dt / 32) * k2);
    MatrixXf k4 = boom_boats_duo.state_der(control1, control2,
    state + (12 * dt / 13) * k1 - (63 * dt / 64) * k2 + (81 * dt / 64) * k3);
    MatrixXf k5 = boom_boats_duo.state_der(control1, control2,
    state + (dt) * k1 + (13 * dt / 64) * k2 - (81 * dt / 64) * k3 + (81 * dt / 64) * k4);
    MatrixXf k6 = boom_boats_duo.state_der(control1, control2,
    state + (dt) * k1 + (3 * dt / 32) * k2 + (9 * dt / 16) * k3 + (3 * dt / 8) * k4);
    return 
 }


// Already defined in generic-boat.cpp
// float wrap_theta(float theta) {
//     theta = fmod(theta, 2 * PI); // Normalize theta within [-2PI, 2PI]
//     if (theta > PI) {
//         theta -= 2 * PI; // Adjust if theta is in (PI, 2PI]
//     } else if (theta < -PI) {
//         theta += 2 * PI; // Adjust if theta is in [-2PI, -PI)
//     }
//     // cout << theta << endl;
//     return theta;
// }

// int sign(float x) {
//     if (x > 0) return 1;
//     else if (x < 0) return -1;
//     return 0;
// }