#include "boom-boat.h"
#include <iostream>
#include <Eigen/Dense>
#include <fstream>

// Parsing json parameters
#include "json/json.hpp"
using json = nlohmann::json;

using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::Matrix3f;
using Eigen::VectorXf;

// // These parameters are defined in the json file
// const float DEFAULT_CAPACITY = 100;
// const float DEFAULT_INITIAL_FUEL = DEFAULT_CAPACITY;


const float PI = 3.141592653589793;

// Default constructor
BoomBoat::BoomBoat() : GenericBoat() {
    // // Load parameters from params.json
    // std::ifstream file("params.json");
    // if (!file.is_open()) {
    //     throw std::runtime_error("Could not open params.json");
    // }
    // json params;
    // file >> params;
    this->load_boom_boat_params("params.json");
    this->fuel = this->boom_boat_params["initial_fuel"].get<float>();
    this->cap = this->boom_boat_params["waste_tank_capacity"].get<float>();
    this->tank_curr = 0.0;
    this->pos << 0, 0, 0;
    this->vel << 0, 0, 0;
    this->set_control(Vector2f(0.0, 0.0));
}

// Constructor with position
BoomBoat::BoomBoat(Vector3f pos) : GenericBoat() {
    // // Load parameters from params.json
    // std::ifstream file("params.json");
    // if (!file.is_open()) {
    //     throw std::runtime_error("Could not open params.json");
    // }
    // json params;
    // file >> params;
    this->load_boom_boat_params("params.json");
    this->fuel = this->boom_boat_params["initial_fuel"].get<float>();
    this->cap = this->boom_boat_params["waste_tank_capacity"].get<float>();
    this->tank_curr = 0.0;
    this->pos = pos;
    this->vel << 0, 0, 0;
    this->set_control(Vector2f(0.0, 0.0));
}

// Copy constructor
BoomBoat::BoomBoat(const BoomBoat &boom_boat) : GenericBoat(boom_boat),
 fuel(boom_boat.fuel), tank_curr(boom_boat.tank_curr), cap(boom_boat.cap) {
    this->set_control(Vector2f(0.0, 0.0));
    this->boom_boat_params = boom_boat.boom_boat_params;
 }


// Parameterized constructor
BoomBoat::BoomBoat(float radius, float mass, float inertia, float mu_l,
 float mu_ct, float mu_r, Vector3f pos, Vector3f vel, float fuel, float cap,
  float F_max, float eta_max) : GenericBoat(radius, mass, inertia, mu_l, mu_ct,
   mu_r, pos, vel, F_max, eta_max), fuel(fuel), tank_curr(fuel), cap(cap) {
    this->set_control(Vector2f(0.0, 0.0));
    this->load_boom_boat_params("params.json");
}



// Destructor
BoomBoat::~BoomBoat() {}

// Assignment operator
BoomBoat &BoomBoat::operator=(const BoomBoat &boom_boat) {
    if (this != &boom_boat) {
        GenericBoat::operator=(boom_boat);
        this->fuel = boom_boat.fuel;
        this->tank_curr = boom_boat.tank_curr;
        this->cap = boom_boat.cap;
        this->boom_boat_params = boom_boat.boom_boat_params;
        }
    return *this;
}

// Getter for fuel
float BoomBoat::get_fuel() const { return fuel; }

// Getter for cap
float BoomBoat::get_cap() const { return cap; }


// Getter for tank_curr
float BoomBoat::get_tank_curr() const { return tank_curr; }

// Setter for fuel
void BoomBoat::set_fuel(float fuel) {
    if (fuel < 0) {
        std::cerr << "Fuel cannot be negative." << std::endl;
        return;
    }
    this->fuel = fuel;
}

// Load parameters from the json file
void BoomBoat::load_boom_boat_params(std::string filename) {
    // Read parameters from JSON file
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open params file at BoomBoat::load_boom_boat_params");
    }
    json params;
    file >> params;
    this->boom_boat_params = params["boom_boat"];
}

// Check if the control is valid
// bool BoomBoat::is_valid_control(Vector2f control) const {
//     // Check if the abs(force) is within the limits
//     if (abs(control(0)) > this->get_F_max()) {
//         std::cerr << "Force exceeds the maximum limit: " << this->get_F_max() << " [N]" << std::endl;
//         return false;
//     }
//     // Check if the steering angle is within the limits
//     if (abs(control(1)) > this->get_eta_max()) {
//         std::cerr << "Steering angle exceeds the maximum limit: " << this->get_eta_max() << " [rad]" << std::endl;
//         return false;
//     }

//     // Load Lipschitz continuity parameters
//     std::string file_path = "params.json";

//     // Open the file and parse it
//     std::ifstream file(file_path);
//     if (!file.is_open()) {
//         std::cerr << "Failed to open parameters file " << file_path << std::endl;
//         return 1;
//     }
//     json params;
//     file >> params;

//     float Lips_F = params["generic_boat"]["lipschitz_cont_F"];
//     float Lips_eta = params["generic_boat"]["lipschitz_cont_eta_deg"];
//     Lips_eta = Lips_eta * PI / 180.0;

//     // Check Lipschitz continuity
//     if (abs(control(0) - this->get_control()(0)) > Lips_F) {
//         std::cerr << "Lipschitz continuity violated for force control." << std::endl;
//         return false;
//     }
//     if (abs(control(1) - this->get_control()(1)) > Lips_eta) {
//         std::cerr << "Lipschitz continuity violated for steering angle control." << std::endl;
//         return false;
//     }

//     return true;
// }

// Print the status of the boat
void BoomBoat::print_status() const {
    std::cout << "Fuel: " << this->fuel << " kg\n" << std::endl;
    std::cout << "Tank Current: " << this->tank_curr << " kg\n"
                << std::endl;
    std::cout << "Capacity: " << this->cap << " kg\n" << std::endl;

    // position and velocity
    std::cout << "Position: " << this->get_pos().transpose() << std::endl;
    std::cout << "Velocity: " << this->get_vel().transpose() << std::endl;
}

// State derivative function
VectorXf BoomBoat::state_der(VectorXf state, Vector2f control,
     Vector2f boom_force) const {
    // state = [x, y, theta, x_dot, y_dot, omega]
    // control = [F, eta]

    // Unpack state
    // float x = state(0);
    // float y = state(1);
    float theta = state(2);
    float x_dot = state(3);
    float y_dot = state(4);
    float omega = state(5);
    // Unpack control
    float F = control(0);
    float eta = control(1);

    // Unpack boat parameters
    float r = this->get_radius();
    float mass = this->get_mass();
    float I = this->get_inertia();
    float mu_r = this->get_mu_r();
    float mu_l = this->get_mu_l();
    float mu_ct = this->get_mu_ct();

    // Rotation to local frame
    float u = x_dot * sin(theta) + y_dot * cos(theta);
    float v = x_dot * cos(theta) - y_dot * sin(theta);

    float F1 = boom_force(0);
    float F2 = boom_force(1);
    float F_boom_u = F1 * sin(theta) + F2 * cos(theta);
    float F_boom_v = F1 * cos(theta) - F2 * sin(theta);

    // EOM
    float u_dot = (F * cos(eta) - mu_l * u * u * sign(u) + F_boom_u) / mass;
    float v_dot = (-F * sin(eta) - mu_ct * v * v * sign(v) + F_boom_v) / mass;
    float omega_dot = (r * F * sin(eta) - mu_r * omega * omega * sign(omega) - r * F_boom_v) / I;

    // // print u_dot
    // cout << "u_dot: " << u_dot << endl;
    // cout.flush();
    // cout << "F_boom_v: " << F_boom_v << endl;
    // cout.flush();

    // Rotation backward to global frame
    float x_dotdot = u_dot * sin(theta) + v_dot * cos(theta);
    float y_dotdot = u_dot * cos(theta) - v_dot * sin(theta);

    VectorXf state_dot(6);
    state_dot << x_dot, y_dot, omega, x_dotdot, y_dotdot, omega_dot;
    return state_dot;
    }
