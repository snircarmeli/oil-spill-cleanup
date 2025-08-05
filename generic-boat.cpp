#include "generic-boat.h"
#include "boom-boat.h"
#include "helper_funcs.h"
#include <cmath>
#include <iostream>
#include <cstdio>
#include <stdexcept> // For std::runtime_error
#include <Eigen/Dense>
#include <fstream> // For file I/O

// Parsing json parameters
#include "json/json.hpp"
using json = nlohmann::json;

using std::sin;
using std::cos;
using std::abs;

// using Eigen::Vector3d;
// using Eigen::Vector2d;
// using Matrix2x3d = Eigen::Matrix<double, 2, 3>;


// const double PI = 3.141592653589793;

// Constructors
// radius - 2 mass - 600 inertia - 50 mu_l - 1 mu_ct - 100 mu_r - 1 F_max - 100 eta_max - pi/2
GenericBoat::GenericBoat() {
    // Load parameters from params.json
    this->load_params("params.json");

    json generic_boat_params = this->generic_boat_params;

    this->radius = generic_boat_params["radius"].get<double>();
    this->mass = generic_boat_params["mass"].get<double>();
    this->inertia = generic_boat_params["inertia"].get<double>();
    this->mu_l = generic_boat_params["drag_coefficients"]["linear"].get<double>();
    this->mu_ct = generic_boat_params["drag_coefficients"]["cross_track"].get<double>();
    this->mu_r = generic_boat_params["drag_coefficients"]["rotational"].get<double>();
    this->F_max = generic_boat_params["max_controls"]["force"].get<double>();
    this->eta_max = generic_boat_params["max_controls"]["steering_angle"].get<double>();
    this->pos << generic_boat_params["initial_position"][0].get<double>(),
     generic_boat_params["initial_position"][1].get<double>(),
     generic_boat_params["initial_position"][2].get<double>();
    this->vel << generic_boat_params["initial_velocity"][0].get<double>(),
     generic_boat_params["initial_velocity"][1].get<double>(), 
     generic_boat_params["initial_velocity"][2].get<double>();

    this->F = 0;
    this->eta = 0;
}

GenericBoat::GenericBoat(const GenericBoat &gen_boat) {
    this->radius = gen_boat.radius;
    this->mass = gen_boat.mass;
    this->inertia = gen_boat.inertia;
    this->mu_l = gen_boat.mu_l;
    this->mu_ct = gen_boat.mu_ct;
    this->mu_r = gen_boat.mu_r;
    
    this->pos = gen_boat.pos;
    this->vel = gen_boat.vel;
    this->F_max = gen_boat.F_max;
    this->eta_max = gen_boat.eta_max;

    this->F = gen_boat.F;
    this->eta = gen_boat.eta;

    this->load_params("params.json");
}
GenericBoat::GenericBoat(double radius, double mass, double inertia, double mu_l, 
        double mu_ct, double mu_r, Vector3d pos, Vector3d vel, double F_max,
         double eta_max) : 
        radius(radius), mass(mass), 
         inertia(inertia), mu_l(mu_l), mu_ct(mu_ct), mu_r(mu_r), F_max(F_max), 
         eta_max(eta_max) {
    this->pos = pos;
    this->vel = vel;

    this->F = 0;
    this->eta = 0;
    this->load_params("params.json");
}

// Destructor
GenericBoat::~GenericBoat() {
        // No specific resource management needed here
}

// Assignment operator
GenericBoat& GenericBoat::operator=(const GenericBoat &gen_boat) {
    this->radius = gen_boat.radius;
    this->mass = gen_boat.mass;
    this->inertia = gen_boat.inertia;
    this->mu_l = gen_boat.mu_l;
    this->mu_ct = gen_boat.mu_ct;
    this->mu_r = gen_boat.mu_r;
    this->pos = gen_boat.pos;
    this->vel = gen_boat.vel;
    this->F_max = gen_boat.F_max;
    this->eta_max = gen_boat.eta_max;
    this->pos = gen_boat.pos;
    this->vel = gen_boat.vel;
    this->F = gen_boat.F;
    this->eta = gen_boat.eta;

    this->load_params("params.json");
    return *this;
}






// Accessors
Vector3d GenericBoat::get_pos() const { return this->pos; }

Vector3d GenericBoat::get_vel() const { return this->vel; }

Vector3d GenericBoat::get_frame_vel() const {
    Vector3d global_vel = this->get_vel();
    double theta = this->get_pos()[2];
    double x_dot = global_vel[0];
    double y_dot = global_vel[1];
    double omega = global_vel[2];

    double u = x_dot * sin(theta) + y_dot * cos(theta);
    double v = x_dot * cos(theta) - y_dot * sin(theta);
    double omega_local = omega;

    return Vector3d(u, v, omega_local);
}

Vector2d GenericBoat::get_control() const { return Vector2d(this->F, this->eta); }

double GenericBoat::get_radius() const { return this->radius; }

double GenericBoat::get_mass() const { return this->mass; }

double GenericBoat::get_inertia() const { return this->inertia; }

double GenericBoat::get_mu_l() const { return this->mu_l; }

double GenericBoat::get_mu_ct() const { return this->mu_ct; }

double GenericBoat::get_mu_r() const { return this->mu_r; }

double GenericBoat::get_F_max() const { return this->F_max; }

double GenericBoat::get_eta_max() const { return this->eta_max; }  

double GenericBoat::get_ship_size() const { return this->generic_boat_params["ship_size"]; }

void GenericBoat::print_params() {
    std::cout << "Radius: " << this->radius << " m\n" << endl;
    std::cout << "Mass: " << this->mass << " kg\n" << endl;
    std::cout << "Inertia: " << this->inertia << " kg*m^2\n" << endl;
    std::cout << "Mu_l (Linear Drag): " << this->mu_l << " kg/m\n" << endl;
    std::cout << "Mu_ct (Cross Track Drag): " << this->mu_ct << " kg/m\n" << endl;
    std::cout << "Mu_r (Rotational Drag): " << this->mu_r << " kg*m^2\n" << endl;
    std::cout << "F_max (Max Force): " << this->F_max << " N\n" << endl;
    std::cout << "Eta_max (Max Steering Angle): " << this->eta_max << " rad\n" << endl;
    std::cout << "Initial Position: " << this->pos.transpose() << endl;
    std::cout << "Initial Velocity: " << this->vel.transpose() << endl;
    std::cout << "Current Control: " << this->F << " N, " << this->eta << " rad" << endl;
}

void GenericBoat::set_pos(Vector3d pos) {
    this->pos = pos;
}

void GenericBoat::set_vel(Vector3d vel) {
    this->vel = vel;
}

void GenericBoat::set_control(Vector2d control) {
    if (abs(control(0)) > this->F_max) {
        this->F = this->F_max * sign(control(0));
    }
    this->eta = wrap_eta(control[1]);
}

bool GenericBoat::is_valid_control(Vector2d control) const {

    // Check if the abs(force) is within the limits
    if (abs(control(0)) > this->get_F_max()) {
        // cout << "Force exceeds the maximum limit: " << this->get_F_max() << " [N]" << endl;
        cout.flush();
        return false;
    }
    // Check if the steering angle is within the limits
    if (abs(control(1)) > this->get_eta_max()) {
        // cout << "Steering angle exceeds the maximum limit: " << this->get_eta_max() << " [rad]" << endl;
        cout.flush();
        return false;
    }


    double Lips_F = this->generic_boat_params["lipschitz_cont_F"];
    double Lips_eta = this->generic_boat_params["lipschitz_cont_eta_deg"];
    Lips_eta = Lips_eta * PI / 180.0;

    // Check Lipschitz continuity
    if (abs(control(0) - this->F) > Lips_F ) {
        // cout << "Lipschitz continuity violated for force control." << endl;
        // cout.flush();
        // cout << "Force: " << control(0) << " " << this->get_control()(0) << endl;
        // cout.flush();
        return false;
    }
    if (abs(control(1) - this->eta) > Lips_eta) {
        // cout << "Lipschitz continuity violated for steering control." << endl;
        // cout.flush();
        // cout << "Steering: " << control(1) * 180 / PI << " " << this->get_control()(1) * 180 / PI << endl;
        // cout.flush();
        return false;
    }

    return true;
}

void GenericBoat::load_params(std::string filename) {
    // std::string file_path = "params.json";

    // Open the file and parse it
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open parameters file in GenericBoat load_params(). ");
        return;
    }
    this->generic_boat_params= json();
    file >> this->generic_boat_params;
    this->generic_boat_params = this->generic_boat_params["generic_boat"];
}

// double wrap_theta(double theta) {
//     theta = fmod(theta, 2 * PI); // Normalize theta within [-2PI, 2PI]
//     if (theta > PI) {
//         theta -= 2 * PI; // Adjust if theta is in (PI, 2PI]
//     } else if (theta < -PI) {
//         theta += 2 * PI; // Adjust if theta is in [-2PI, -PI)
//     }
//     // cout << theta << endl;
//     return theta;
// }

// int sign(double x) {
//     if (x > 0) return 1;
//     if (x < 0) return -1;
//     return 0;
// }



