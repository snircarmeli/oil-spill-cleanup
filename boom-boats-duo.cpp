#include "boom-boats-duo.h"
#include "boom-boat.h"
#include "integrator.h"
#include "helper_funcs.h"
// #include "integrator.h"
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem> // For filesystem operations (C++17 and later)

// For JSON parameters parsing
#include "json/json.hpp"
using json = nlohmann::json;

using std::cout;
using std::endl;
using std::string;
using std::pair;
using std::make_pair;

namespace fs = std::filesystem;

// const double PI = 3.141592653589793;

double K_deault = 10000000000000;
double C_default = 1000000;

// Boom Constructor
Boom::Boom(size_t num_links, double L, double mu_l, double mu_ct, double mu_r,
 double I, double m, double k, double c)
    : L(L), mu_l(mu_l), mu_ct(mu_ct), mu_r(mu_r), I(I), m(m), k(k), c(c) {
       links_states = MatrixXd::Zero(num_links, 6);
       this->load_boom_params("params.json");
    }

Boom::Boom(size_t num_links, double L) {
    this->load_boom_params("params.json");

    links_states = MatrixXd::Zero(num_links, 6);
    this->L = L;
    
    json boom_params = this->boom_params;
    json drag_params = boom_params["drag_coefficients"];
    // Extract parameters from JSON
    this->mu_l = drag_params["linear"].get<double>();
    this->mu_ct = drag_params["cross_track"].get<double>();
    this->mu_r = drag_params["rotational"].get<double>();
    this->I = boom_params["inertia"].get<double>();
    this->m = boom_params["mass"].get<double>();
    this->k = boom_params["spring_constant"].get<double>();
    this->c = boom_params["damping_coefficient"].get<double>();
    // this->t = 0;

    links_states = MatrixXd::Zero(num_links, 6);
}

// Default Constructor
Boom::Boom() {
    this->load_boom_params("params.json");
    json boom_params = this->boom_params;

    json drag_params = boom_params["drag_coefficients"];
    
    this->L = boom_params["link_length"].get<double>();
    this->mu_l = drag_params["linear"].get<double>();
    this->mu_ct = drag_params["cross_track"].get<double>();
    this->mu_r = drag_params["rotational"].get<double>();
    this->I = boom_params["inertia"].get<double>();
    this->m = boom_params["mass"].get<double>();
    this->k = boom_params["spring_constant"].get<double>();
    this->c = boom_params["damping_coefficient"].get<double>();
    size_t num_links = boom_params["num_links"].get<size_t>();

    links_states = MatrixXd::Zero(num_links, 6);
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
        boom_params = other.boom_params;
    }
    return *this;
}

// Accessors
void Boom::set_link_state(size_t index, const VectorXd &state) {
    links_states.row(index) = state;
}

VectorXd Boom::get_link_state(size_t index) const {
    return links_states.row(index);
}

void Boom::set_links_states(MatrixXd &states) {
    for (size_t i = 0; i < static_cast<size_t>(states.rows()); ++i) {
        this->links_states.row(i) = states.row(i);
    }
}

double Boom::get_L() const { return L; }

double Boom::get_I() const { return I; }

double Boom::get_m() const { return m; }

double Boom::get_mu_l() const { return mu_l; }

double Boom::get_mu_ct() const { return mu_ct; }

double Boom::get_mu_r() const { return mu_r; }

double Boom::get_k() const { return k; }

double Boom::get_c() const { return c; }

void Boom::load_boom_params(std::string filename) {
    // Read parameters from JSON file
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open params file at Boom::load_boom_params");
    }
    json params;
    file >> params;
    file.close();
    this->boom_params = params["boom"];
}

int Boom::get_num_links() const {
    return static_cast<int>(links_states.rows());
}

void Boom::print_links_states() const {
    cout << "Boom Length: " << L << endl;
    cout << "Number of Links: " << links_states.rows() << endl;
    cout << "Links States: \n" << links_states << endl;
}

void Boom::print_link_state(size_t index) const {
    cout << "Link " << index << " State: " << links_states.row(index) << endl;
}

// Functions to check if two line segments intersect

// Helper function to calculate the direction
double direction(double xi, double yi, double xj, double yj, double xk, double yk) {
    return (xk - xi) * (yj - yi) - (yk - yi) * (xj - xi);
}

bool Boom::is_valid_state() const {
    // Check if boom doesn't intersect itself
    double L = this->get_L();
    for (int i = 0; i < this->get_num_links(); i++) {

        double x_i = links_states(i, 0);
        double y_i = links_states(i, 1);
        double theta_i = links_states(i, 2);

        // Coordinates of boom link i
        double x_11 = x_i + 0.5 * L * cos(theta_i);
        double y_11 = y_i + 0.5 * L * sin(theta_i);
        double x_12 = x_i - 0.5 * L * cos(theta_i);
        double y_12 = y_i - 0.5 * L * sin(theta_i); 

        // Starting from the second next link. Every two adjacent links can
        // intersect because of the spring-damper model.
        for (int j = this->get_num_links() - 1; j >= i + 2; j--) {
            double x_j = links_states(j, 0);
            double y_j= links_states(j, 1);
            double theta_j = links_states(j, 2);
            // Coordinates of boom link j
            double x_21 = x_j + 0.5 * L * cos(theta_j);
            double y_21 = y_j + 0.5 * L * sin(theta_j);
            double x_22 = x_j - 0.5 * L * cos(theta_j);
            double y_22 = y_j - 0.5 * L * sin(theta_j);

            // Check if the two links intersect
            // if ( j == i + 1) {
            //     // Check if the two links completely overlap
            //     if (x_11 == x_21 && y_11 == y_21 && x_22 == x_12 && y_22 == y_12) {
            //         return false;
            //     }
            // } else {
            if (check_intersection(x_11, y_11, x_12, y_12, x_21, y_21, x_22,
             y_22, L)) {
                // cout << "Link " << i << " and Link " << j << " intersect" << endl;
                // cout.flush();
                return false;
            }
        }
    }
    return true;
}

// State derivative function
MatrixXd Boom::state_der(const MatrixXd &state, const Vector2d Boom_force1,
 const Vector2d Boom_force2) const {
    // Model the dynamics of the boom links as spring and dampers
    int num_links = this->get_num_links();
    double L = this->get_L();
    double mu_l = this->get_mu_l();
    double mu_ct = this->get_mu_ct();
    double mu_r = this->get_mu_r();
    double I = this->get_I();
    double m = this->get_m();
    double k = this->get_k();
    double c = this->get_c();

    // 2 boats and num_links links, 6 states each
    MatrixXd state_der = MatrixXd::Zero(num_links, 6);

    // state is of same order as state_der
    // Declare variables for the calculations
    double theta; double x_dot_i; double y_dot_i; double theta_dot_i; 
    Vector2d link_vel; Vector2d P0; Vector2d P1; Vector2d P0_dot;
    Vector2d P1_dot; Vector2d e; Vector2d n; Vector2d F0;
    Vector2d F1; Vector2d M; Vector2d M_dot; Vector2d F_spring_0;
    Vector2d e0; Vector2d F_damp_0; Vector2d N; Vector2d N_dot;
    Vector2d F_spring_1; Vector2d e1; Vector2d F_damp_1; double torque;
    double vel_e; double vel_n; double F_e; double F_n; double vel_e_dot;
    double vel_n_dot; double omega_dot; double x_dotdot; double y_dotdot;
    double theta_dotdot;

    for (int i = 0; i < num_links; ++i) {
        // double x_i = state(i, 0);
        // double y_i = state(i, 1);
        theta = state(i, 2);
        x_dot_i = state(i, 3);
        y_dot_i = state(i, 4);
        theta_dot_i = state(i, 5);

        link_vel = Vector2d(x_dot_i, y_dot_i);

        // P0 - left enf of link i
        // P1 - right end of link i
        P0 = Vector2d(state(i, 0), state(i, 1)) - (L / 2) * Vector2d(cos(state(i, 2)), sin(state(i, 2)));
        P1 = Vector2d(state(i, 0), state(i, 1)) + (L / 2) * Vector2d(cos(state(i, 2)), sin(state(i, 2)));
        P0_dot = Vector2d(state(i, 3), state(i, 4)) - (L / 2) * state(i, 5) * Vector2d(-sin(state(i, 2)), cos(state(i, 2)));
        P1_dot = Vector2d(state(i, 3), state(i, 4)) + (L / 2) * state(i, 5) * Vector2d(-sin(state(i, 2)), cos(state(i, 2)));

        // Normalized vector in the direction of the link
        e = (P1 - P0).normalized();
        // Normalized vector perpendicular to the link, rotate v by 90 degrees in the counter-clockwise direction   
        n = Vector2d(-e(1), e(0));

        // Calculate the forces on the link
        // F0 - force on the left end of the link
        // F1 - force on the right end of the link
        F0 = Vector2d(0, 0);
        F1 = Vector2d(0, 0);

        // M - right end of link i-1
        // N - left end of link i+1
        if (i != 0) {
            M = Vector2d(state(i - 1, 0), state(i - 1, 1)) + (L / 2) * Vector2d(cos(state(i - 1, 2)), sin(state(i - 1, 2)));
            M_dot = Vector2d(state(i - 1, 3), state(i - 1, 4)) + (L / 2) * state(i - 1, 5) * Vector2d(-sin(state(i - 1, 2)), cos(state(i - 1, 2)));

            F_spring_0 = k * (M - P0);
            // unit vector in the direction of (M-P0)
            e0 = (M-P0).normalized();
            F_damp_0 = c * (M_dot - P0_dot).dot(e0) * e0;
            F0 = F_spring_0 + F_damp_0;
        } else {F0 = Boom_force1;}

        if (i != num_links - 1) {
            N = Vector2d(state(i + 1, 0), state(i + 1, 1)) - (L / 2) * Vector2d(cos(state(i + 1, 2)), sin(state(i + 1, 2)));
            N_dot = Vector2d(state(i + 1, 3), state(i + 1, 4)) - (L / 2) * state(i + 1, 5) * Vector2d(-sin(state(i + 1, 2)), cos(state(i + 1, 2)));

            F_spring_1 = k * (N -P1);
            // unit vector in the direction of (N-P1)
            e1 = (N - P1).normalized();
            F_damp_1 = c * (N_dot - P1_dot).dot(e1) * e1;
            F1 = F_spring_1 + F_damp_1;
        } else {F1 = Boom_force2;}

        // Calculate the torques on the link 
        // (Cross product of the force and the position vector)
        torque = 0.5 * L * (e.x() * (F1 - F0).y() - e.y() * (F1 - F0).x());
        // cout << "Torque: " << torque << endl;
        // cout.flush();
        // print if F0 and F1 are not Nan
        // if (isnan(F0.x()) || isnan(F0.y()) || isnan(F1.x()) || isnan(F1.y())) {
        //     // cout << "F0 or F1 is Nan" << endl;
        //     // cout.flush();
        // } else {
        //     cout << "Link " << i << " Forces: \n";
        //     cout.flush();
        //     cout << "F0_x: " << F0.x() << endl;
        //     cout.flush();
        //     cout << "F0_y: " << F0.y() << endl;
        //     cout.flush();
        //     cout << "F1_x: " << F1.x() << endl;
        //     cout.flush();
        //     cout << "F1_y: " << F1.y() << endl;
        //     cout.flush();
        // }
       
        // Rotate to n and e frame
        vel_e = link_vel.dot(e);
        vel_n = link_vel.dot(n);
        F_e = (F0 + F1).dot(e);
        F_n = (F0 + F1).dot(n);
        // Calculate the state derivatives
        vel_e_dot = (1 / m) * (F_e - mu_l * (vel_e * vel_e) * sign(vel_e));
        vel_n_dot = (1 / m) * (F_n - mu_ct * (vel_n * vel_n) * sign(vel_n));
        omega_dot = (1 / I) * (torque - mu_r * (theta_dot_i * theta_dot_i) * sign(theta_dot_i));
        // Rotate back to the global frame
        x_dotdot = vel_e_dot * cos(theta) - vel_n_dot * sin(theta);
        y_dotdot = vel_e_dot * sin(theta) + vel_n_dot * cos(theta);
        theta_dotdot = omega_dot;
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
 size_t num_links, double L, double mu_l, double mu_ct, double mu_r,
  double I, double m, double k, double c, Vector2d center, double orientation)
    : boat1(b1.get_radius(), b1.get_mass(), b1.get_inertia(), b1.get_mu_l(), b1.get_mu_ct(), b1.get_mu_r(),
            b1.get_pos(), b1.get_vel(), b1.get_fuel(), b1.get_cap(), b1.get_F_max(), b1.get_eta_max()),
      boat2(b2.get_radius(), b2.get_mass(), b2.get_inertia(), b2.get_mu_l(), b2.get_mu_ct(), b2.get_mu_r(),
            b2.get_pos(), b2.get_vel(), b2.get_fuel(), b2.get_cap(), b2.get_F_max(), b2.get_eta_max()),
            boom(num_links, L, mu_l, mu_ct, mu_r, I, m, k, c), t(0) {

        this->load_boom_boats_duo_params("params.json");

        // Place boats at the center with the given orientation of the line 
        // connecting the boats
        
        // b1 - left boat
        // b2 - right boat
        // Orientation is with respect to the y-axis

        Vector3d b1_pos = Vector3d(center(0) - 0.5 * num_links * L * cos(orientation), center(1) + 0.5 * num_links * L * sin(orientation), orientation);
        Vector3d b2_pos = Vector3d(center(0) + 0.5 * num_links * L * cos(orientation), center(1) - 0.5 * num_links * L * sin(orientation), orientation);

        // // print both positions
        // cout << "Boat 1 position: (" << b1_pos(0) << ", " << b1_pos(1) << ", " << b1_pos(2) << ")" << endl;
        // cout << "Boat 2 position: (" << b2_pos(0) << ", " << b2_pos(1) << ", " << b2_pos(2) << ")" << endl;

        // Vector3d b1_stern = Vector3d(b1_pos(0) - b1.get_radius() * sin(b1_pos(2)), b1_pos(1) - b1.get_radius() * cos(b1_pos(2)), b1_pos(2));
        // Vector3d b2_stern = Vector3d(b2_pos(0) - b2.get_radius() * sin(b2_pos(2)), b2_pos(1) - b2.get_radius() * cos(b2_pos(2)), b2_pos(2));

        boat1.set_pos(b1_pos);
        boat2.set_pos(b2_pos);
        VectorXd state(6);
        for (int i = 0; i < boom.get_num_links(); ++i) {
            double x = b1_pos(0) + (i + 0.5) * L * cos(orientation);
            double y = b1_pos(1) - (i + 0.5) * L * sin(orientation);
            double theta = -orientation; // With respect to the x-axis
            state << x, y, theta, 0.0, 0.0, 0.0;
            boom.set_link_state(i, state);
        }

        // // Print link 1 state
        // cout << "Link 1 state: \n" << boom.get_link_state(0) << endl;
        // cout.flush();
        // // Print last link state
        // cout << "Last link state: \n" << boom.get_link_state(boom.get_num_links() - 1) << endl;
        // cout.flush();
    }

BoomBoatsDuo:: BoomBoatsDuo(Vector2d center,  double orientation,
 size_t num_links, double L) : t(0) {
    Boom boom(num_links, L);
    this->boom = boom;
    Vector3d boat1_pos = Vector3d(center(0) - L * num_links * cos(orientation), 
    center(1) - L * num_links * sin(orientation), PI/2 + orientation);
    Vector3d boat2_pos = Vector3d(center(0) + L * num_links * cos(orientation), 
    center(1) + L * num_links * sin(orientation), PI/2 + orientation);

    BoomBoat boat1 = BoomBoat(boat1_pos);
    BoomBoat boat2 = BoomBoat(boat2_pos);
    this->boat1 = boat1;
    this->boat2 = boat2;

    this->load_boom_boats_duo_params("params.json");
}


// BoomBoatsDuo Destructor
BoomBoatsDuo::~BoomBoatsDuo() {}

// Assignment Operator
BoomBoatsDuo& BoomBoatsDuo::operator=(const BoomBoatsDuo &other) {
    if (this != &other) {
        boat1 = other.boat1;
        boat2 = other.boat2;
        boom = other.boom;
        t = other.t;
        boom_boats_duo_params = other.boom_boats_duo_params;
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
            throw std::runtime_error("Failed to create directory");
            return;
        }
    }
    // Construct the full file path
    string filepath = foldername + "/" + filename;
    // Open the file in append mode
    std::ofstream file(filepath, std::ios_base::app);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filepath);
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
    Vector3d boat1_pos = boat1.get_pos();
    Vector3d boat1_vel = boat1.get_vel();
    Vector2d boat1_control = boat1.get_control();
    ss << boat1_pos.transpose() << " ";
    ss << boat1_vel.transpose() << " ";
    ss << boat1_control.transpose() << " ";

    // Write the state of boat2
    Vector3d boat2_pos = boat2.get_pos();
    Vector3d boat2_vel = boat2.get_vel();
    Vector2d boat2_control = boat2.get_control();
    ss << boat2_pos.transpose() << " ";
    ss << boat2_vel.transpose() << " ";
    ss << boat2_control.transpose() << " ";

    // Write about the boom
    ss << boom.get_num_links() << " ";
    ss << boom.get_L() << " ";

    // Write the state of the boom links
    for (int i = 0; i < boom.get_num_links(); ++i) {
        VectorXd link_state = boom.get_link_state(i);
        ss << link_state.transpose() << " ";
    }

    // input the time
    ss << this->t << " ";

    // Write the collected string to the file
    file << ss.str() << std::endl;

    file.close();
}

void BoomBoatsDuo::load_boom_boats_duo_params(std::string filename){
    // Read parameters from JSON file
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open params file at BoomBoatsDuo::load_boom_boats_duo_params");
    }
    json params;
    file >> params;
    file.close();
    this->boom_boats_duo_params = params["boom_boats_duo"];
    this->simulation_params = params["simulation"];
}

json BoomBoatsDuo::get_simulation_params() const {
    return this->simulation_params;
}

double BoomBoatsDuo::get_time() const {
    return this->t;
}

// Validation of states
bool BoomBoatsDuo::is_valid_state() const {
    // Check if the boom doesn't intersect itself
    if (!this->boom.is_valid_state()) {
        cout << "Boom intersects itself at time " << this->t << " [s]" << endl;
        cout.flush();
        return false;
    }

    // Check if the boats are close
    if (this->are_boats_close()) {
        // Function prints the error message
        return false;
    }
    return true;

}

bool BoomBoatsDuo::are_boats_close() const {
    // Check if the boats are too close or intersect The boom

    double size = this->boat1.get_ship_size();
    double min_dist = this->boom_boats_duo_params["minimal_distance_size_ratio"];
    min_dist *= size;
    // Create two matrices to store the points of the boats
    MatrixXd boat1_points = MatrixXd::Zero(5, 2);
    MatrixXd boat2_points = MatrixXd::Zero(5, 2);
    // 5 points of boat structure

    Vector3d boat1_pos = this->boat1.get_pos();
    Vector3d boat2_pos = this->boat2.get_pos();

    // print both boats positions
    // cout << "Boat1 position: " << boat1_pos(0) << ", " << boat1_pos(1) << endl;
    // cout.flush();
    // cout << "Boat2 position: " << boat2_pos(0) << ", " << boat2_pos(1) << endl;
    // cout.flush();

    MatrixXd boat = MatrixXd::Zero(5,2);
    boat << -size/2, 0,
           size/2, 0,
           size/2, size,
           -size/2, size,
           0, 1.5*size;
    
    // Rotate the boat points clockwise by the orientation of the boat
    Matrix2d R1;
    Matrix2d R2;
    R1 << cos(boat1_pos(2)), sin(boat1_pos(2)),
          -sin(boat1_pos(2)), cos(boat1_pos(2));
    R2 << cos(boat2_pos(2)), sin(boat2_pos(2)),
          -sin(boat2_pos(2)), cos(boat2_pos(2));
    
    boat1_points = (boat * R1.transpose()).rowwise() + boat1_pos.head(2).transpose();
    boat2_points = (boat * R2.transpose()).rowwise() + boat2_pos.head(2).transpose();

    // Check if the boats are close
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            if ((boat1_points.row(i) - boat2_points.row(j)).norm() < min_dist) {
                cout << "Boats are too close at time: " << this->t << " [s]" << endl;
                cout.flush();
                // cout << "Minimal distance was: " << min_dist << endl;
                // cout.flush();
                // // print distance between the two points
                // cout << "Distance between the two points: " << (boat1_points.row(i) - boat2_points.row(j)).norm() << endl;
                // cout.flush();
                return true;
            }
        }
    }

    // Check if the boats intersect the boom
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < boom.get_num_links(); ++j) {
            Vector2d P0 = Vector2d(boom.get_link_state(j)(0), boom.get_link_state(j)(1)) - (boom.get_L() / 2) * Vector2d(cos(boom.get_link_state(j)(2)), sin(boom.get_link_state(j)(2)));
            Vector2d P1 = Vector2d(boom.get_link_state(j)(0), boom.get_link_state(j)(1)) + (boom.get_L() / 2) * Vector2d(cos(boom.get_link_state(j)(2)), sin(boom.get_link_state(j)(2)));
            if (j == 0) {
                if (check_intersection(boat2_points(i, 0), boat2_points(i, 1), boat2_pos(0), boat2_pos(1), P0.x(), P0.y(), P1.x(), P1.y(), boom.get_L())) {
                    cout << "Boat 2 intersects the boom at time: " << this->t << " [s]" << endl;
                    cout.flush();
                    return true;
                }
            } else if (j == boom.get_num_links() - 1) {
                if (check_intersection(boat1_points(i, 0), boat1_points(i, 1), boat1_pos(0), boat1_pos(1), P0.x(), P0.y(), P1.x(), P1.y(), boom.get_L())) {
                    cout << "Boat 1 intersects the boom at time: " << this->t << " [s]" << endl;
                    cout.flush();
                    return true;
                }
            } else {
                if (check_intersection(boat1_points(i, 0), boat1_points(i, 1), boat1_pos(0), boat1_pos(1), P0.x(), P0.y(), P1.x(), P1.y(), boom.get_L()) ||
                    check_intersection(boat2_points(i, 0), boat2_points(i, 1), boat2_pos(0), boat2_pos(1), P0.x(), P0.y(), P1.x(), P1.y(), boom.get_L())) {
                    cout << "One of the boats intersect the boom at time: " << this->t << " [s]" << endl;
                    return true;
                }
            }
        }
    }

    return false;
}

// Propagation function
MatrixXd BoomBoatsDuo::state_der(const Vector2d &control1,
 const Vector2d &control2, MatrixXd state) const {

    // Model the dynamics of the boom links as spring and dampers
    int num_links = boom.get_num_links();
    double L = this->boom.get_L();

    // 2 boats and num_links links, 6 states each
    MatrixXd state_der = MatrixXd::Zero(state.rows(), state.cols());

    // state is of same order as state_der
    // state = [boat1; boat2; link1; link2; ...; link_num_links]

    // Calculate state derivative of boat1

    Vector2d boat1_pos = Vector2d(state(0, 0), state(0, 1));
    Vector2d boat1_vel = Vector2d(state(0, 3), state(0, 4));
    // P0: left end of link 0 - closest link to boat1
    double theta_link = state(2, 2);
    Vector2d P0 = Vector2d(state(2, 0), state(2, 1)) - (L / 2) * Vector2d(cos(theta_link), sin(theta_link));
    Vector2d P0_dot = Vector2d(state(2, 3), state(2, 4)) - (L / 2) * state(2, 5) * Vector2d(-sin(theta_link), cos(theta_link));

    Vector2d F_spring1 = this->boom.get_k() * (P0 - boat1_pos);

    // unit vector in the direction of (P0 - boat1_pos)
    Vector2d e1 = (P0 - boat1_pos).normalized();
    Vector2d F_damp1 = this->boom.get_c() * ((P0_dot - boat1_vel).dot(e1)) * e1;

    Vector2d boom_force1 = F_spring1 + F_damp1;
    // // print boom force1 size and direction
    // cout << "Boom force1 size in state_der: " << boom_force1.norm() << endl;
    // cout << "Boom force1 direction in state_der: " << atan2(boom_force1(1), boom_force1(0)) * RAD2DEG << endl;
    // // Print boat1 force and steering angle
    // cout << "Boat1 force in state_der: " << control1(0) << endl;
    // cout << "Boat1 steering angle in state_der: " << control1(1) << endl;
    // cout.flush();

    VectorXd boat1_state_der = this->boat1.state_der(VectorXd(state.row(0)),
     control1, boom_force1);

    // //  Print force and steering angle of boat1
    // cout << "Force of boat1 in state_der: " << control1(0) << endl;
    // cout << "Steering angle of boat1 in state_der: " << control1(1) << endl;

    // //  Print direction and y-point of boat1
    // cout << "Direction of boat1 in state_der: " << state(0, 2) << endl;
    // cout << "Y-point of boat1 in state_der: " << state(0, 1) << endl;
    // cout.flush();

    // Calculate state derivative of boat2
    Vector2d boat2_pos = Vector2d(state(1, 0), state(1, 1));
    Vector2d boat2_vel = Vector2d(state(1, 3), state(1, 4));
    // P1: right end of link num_links - closest link to boat2
    theta_link = state(num_links + 1, 2);
    Vector2d P1 = Vector2d(state(num_links + 1, 0), state(num_links + 1, 1)) + (L / 2) * Vector2d(cos(theta_link), sin(theta_link));
    Vector2d P1_dot = Vector2d(state(num_links + 1, 3), state(num_links + 1, 4)) + (L / 2) * state(num_links + 1, 5) * Vector2d(-sin(theta_link), cos(theta_link));

    Vector2d F_spring2 = this->boom.get_k() * (P1 - boat2_pos);

    // unit vector in the direction of (P1 - boat2_pos)
    Vector2d e2 = (P1 - boat2_pos).normalized();
    Vector2d F_damp2 = this->boom.get_c() * ((P1_dot - boat2_vel).dot(e2)) * e2;
    Vector2d boom_force2 = F_spring2 + F_damp2;
    // print boom force2 size and direction
    // cout << "Boom force2 size in state_der: " << boom_force2.norm() << endl;
    // cout << "Boom force2 direction in state_der: " << atan2(boom_force2(1), boom_force2(0)) << endl;
    // cout.flush();

    VectorXd boat2_state_der = this->boat2.state_der(VectorXd(state.row(1)),
     control2, boom_force2);

    // // print direction and y-point of boat2
    // cout << "Direction of boat2 in state_der: " << state(1, 2) << endl;
    // cout << "Y-point of boat2 in state_der: " << state(1, 1) << endl;
    // cout.flush();
 
    // Calculate state derivative of the boom links
    // Use boom state derivative function
    MatrixXd boom_state = MatrixXd::Zero(num_links, 6);
    // take all the lines from the state matrix: third line and onwards
    for (int i = 0; i < num_links; i++) {
        boom_state.row(i) = state.row(i + 2);
    }
    MatrixXd boom_state_der = MatrixXd::Zero(num_links, 6);
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
void BoomBoatsDuo::propagate(double dt, const Vector2d &control1,
 const Vector2d &control2, std::string integration_method, Vector3d setpoint1,
 Vector3d setpoint2, Vector3d setpoint1_dot, Vector3d setpoint2_dot) {
    // Calculate state derivative



    MatrixXd state = MatrixXd::Zero(2 + this->boom.get_num_links(), 6);
    state.row(0).head(3) = this->boat1.get_pos().transpose();
    state.row(0).tail(3) = this->boat1.get_vel().transpose();
    state.row(1).head(3) = this->boat2.get_pos().transpose();
    state.row(1).tail(3) = this->boat2.get_vel().transpose();

    // Check validity of the control inputs
    if (!this->boat1.is_valid_control(control1) && this->t > 0) {
        // cout<< "Invalid control input for boat 1 at time: " << this->t << " [s]" << endl;
        // cout.flush();
        // std::cerr << "Check size of control inputs or check whether the control inputs are Lipschitz continuous" << endl;
        // std::cerr.flush();
    }
    if (!this->boat2.is_valid_control(control2) && this->t > 0) {
        // cout<< "Invalid control input for boat 2 at time: " << this->t << " [s]" << endl;
        // cout.flush();
        // std::cerr << "Check size of control inputs or check whether the control inputs are Lipschitz continuous" << endl; 
        // std::cerr.flush();
    }

    this->boat1.set_control(control1);
    this->boat2.set_control(control2);


    MatrixXd state_new = MatrixXd::Zero(2 + this->boom.get_num_links(), 6);
    // set states of links: rows 2-end
    for (int i = 0; i < this->boom.get_num_links(); i++) {
        state.row(i + 2) = this->boom.get_link_state(i).transpose();
    }

    if (integration_method == "Euler") {
        MatrixXd state_der = this->state_der(control1, control2, state);

        state_new = Euler_integration(state, state_der, dt);
        this->t += dt;
    } else if (integration_method == "RK2") {
        state_new = RK2_integration(control1, control2, state, dt, this);
        this->t += dt;
    } else if (integration_method == "RK3") {
        state_new = RK3_integration(control1, control2, state, dt, this);
        this->t += dt;
    } else if (integration_method == "RK4") {
        state_new = RK4_integration(control1, control2, state, dt, this);
        this->t += dt;
    } else if (integration_method == "RK5") {
        state_new = RK5_integration(control1, control2, state, dt, this);
        this->t += dt;
    } else if (integration_method == "RK6") {
        state_new = RK6_integration(control1, control2, state, dt, this);
        this->t += dt;
    } else if (integration_method == "RK45") {
        pair<MatrixXd, double> result = RK45_integration(control1, control2, state, dt, this, this->simulation_params);
        state_new = result.first;
        this->t += result.second;
    } else {
        throw std::runtime_error("Invalid integration method: " + integration_method);
    }

    // wrap theta for all states, third column
    for (int i = 0; i < state_new.rows(); i++) {
        state_new(i, 2) = wrap_theta(state_new(i, 2));
    }


    // this->boat1.set_pos(state_new.row(0).head(3).transpose());
    // this->boat1.set_vel(state_new.row(0).tail(3).transpose());
    // this->boat2.set_pos(state_new.row(1).head(3).transpose());
    // this->boat2.set_vel(state_new.row(1).tail(3).transpose());

    // set states of boats from setpoints - temporary fix
    setpoint1(2) = wrap_theta(PI / 2 - setpoint1(2));
    setpoint1_dot(2) = -setpoint1_dot(2);
    setpoint2(2) = wrap_theta(PI / 2 - setpoint2(2));
    setpoint2_dot(2) = -setpoint2_dot(2);
    this->boat1.set_pos(setpoint1);
    this->boat1.set_vel(setpoint1_dot);
    this->boat2.set_pos(setpoint2);
    this->boat2.set_vel(setpoint2_dot);

    // set states of links: rows 2-end
    for (int i = 0; i < this->boom.get_num_links(); i++) {
        this->boom.set_link_state(i, state_new.row(i + 2).transpose());
    }
}

// Getters
BoomBoat BoomBoatsDuo::get_boat1() const { // return copy of boat1
    return this->boat1;
}

BoomBoat BoomBoatsDuo::get_boat2() const { // return copy of boat2
    return this->boat2;
}

Boom BoomBoatsDuo::get_boom() const { // return copy of boom
    return this->boom;
}
