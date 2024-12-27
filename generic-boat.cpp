#include "generic-boat.h"
#include "boom-boat.h"
#include "container-boat.h"
#include <cmath>
#include <iostream>
#include <cstdio>
#include <stdexcept> // For std::runtime_error
#include <Eigen/Dense>

using std::sin;
using std::cos;
using std::abs;

using Eigen::Vector3f;
using Eigen::Vector2f;
using Matrix2x3f = Eigen::Matrix<float, 2, 3>;

#define N 6

const double PI = 3.141592653589793;


// Constructors
// radius - 2 mass - 600 inertia - 50 mu_l - 1 mu_ct - 100 mu_r - 1 F_max - 100 eta_max - pi/2
GenericBoat::GenericBoat() : radius(2), mass(600), inertia(50), mu_l(100), mu_ct(1000), 
mu_r(1000), F_max(100000), eta_max(PI / 2) {
        this->pos << 0, 0, 0;
        this->vel << 0, 0, 0;
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
}
GenericBoat::GenericBoat(float radius, float mass, float inertia, float mu_l, 
        float mu_ct, float mu_r, Vector3f pos, Vector3f vel, float F_max,
         float eta_max) : 
        radius(radius), mass(mass), 
inertia(inertia), mu_l(mu_l), mu_ct(mu_ct), mu_r(mu_r), F_max(F_max), 
eta_max(eta_max) {
    this->pos = pos;
    this->vel = vel;
}

// Destructor
GenericBoat::~GenericBoat() {
        // No specific resource management needed here
}


//EOM
 Matrix2x3f GenericBoat::state_der(Matrix2x3f state, Vector2f control) {
    float F = control[0]; // F is 10
    float eta = control[1]; // 
    // State: x, y, theta, x_dot, y_dot, omega
    float theta = wrap_theta(state(0, 2));
  
    float x_dot = state(1, 0);
    float y_dot = state(1, 1);
    float omega = state(1, 2);

    float r = this->radius;
    float mass = this->mass;
    float I = this->inertia;
    float mu_r = this->mu_r;
    float mu_l = this->mu_l;
    float mu_ct = this->mu_ct;
    
    // Rotation to local frame
    float u = x_dot * sin(theta) + y_dot * cos(theta);
    float v = x_dot * cos(theta) - y_dot * sin(theta);
    // EOM
    float u_dot = (F * cos(eta) - mu_l * u * u * sign(u)) / mass;
    float v_dot = (-F * sin(eta) - mu_ct * v * v * sign(v)) / mass;
    float omega_dot = (r * F * sin(eta) - mu_r * omega * omega * sign(omega)) / I;
    // Rotation backward to global frame
    float x_dotdot = u_dot * sin(theta) + v_dot * cos(theta);
    float y_dotdot = u_dot * cos(theta) - v_dot * sin(theta);

    Matrix2x3f state_dot;
    state_dot << x_dot, y_dot, omega, x_dotdot, y_dotdot, omega_dot;
    return state_dot;
}

void GenericBoat::propogate(Vector2f control, float dt) {
    float F = control[0];
    float eta = control[1];
    // cout << eta << endl;
    if (abs(F) > this->F_max || abs(eta) > this->eta_max) {
        throw std::runtime_error("Force or steering angle exceeds maximum allowed values");
    }


    // Runge-Kutta 4
    Matrix2x3f y_n;
    y_n << this->pos[0], this->pos[1], wrap_theta(this->pos[2]), 
        this->vel[0], this->vel[1], this->vel[2];

    Matrix2x3f k1 = this->state_der(y_n, control);

    // Vectorized operations to update y_tmp
    Matrix2x3f y_tmp = y_n + (dt / 2.0f) * k1;

    Matrix2x3f k2 = this->state_der(y_tmp, control);
    y_tmp = y_n + (dt / 2.0f) * k2;

    Matrix2x3f k3 = this->state_der(y_tmp, control);
    y_tmp = y_n + dt * k3;

    Matrix2x3f k4 = this->state_der(y_tmp, control);

    // Compute y_(n+1) using vectorized operations
    Matrix2x3f y_next = y_n + (dt / 6.0f) * (k1 + 2 * k2 + 2 * k3 + k4);
    // Runge-Kutta 4
    
    // // Forward euler
    // Matrix2x3f y_n;
    // y_n << this->pos[0], this->pos[1], wrap_theta(this->pos[2]),
    //        this->vel[0], this->vel[1], this->vel[2];
    // Matrix2x3f state_dot = this->state_der(y_n, control);
    // Matrix2x3f y_next = y_n + dt * state_dot;

    this->pos = y_next.row(0);  // Takes the first 3 elements from y_next

    // Wrap theta (third element)
    this->pos[2] = wrap_theta(this->pos[2]);

    // Assign vel (last 3 elements)
    this->vel = y_next.row(1);  // Takes the last 3 elements from y_next


    // Debugging statement
    // cout << "hi" << endl;
}


// Accessors
Vector3f GenericBoat::get_pos() const { return this->pos; }

Vector3f GenericBoat::get_vel() const { return this->vel; }

float GenericBoat::get_radius() const { return this->radius; }

float GenericBoat::get_mass() const { return this->mass; }

float GenericBoat::get_inertia() const { return this->inertia; }

float GenericBoat::get_mu_l() const { return this->mu_l; }

float GenericBoat::get_mu_ct() const { return this->mu_ct; }

float GenericBoat::get_mu_r() const { return this->mu_r; }

float GenericBoat::get_F_max() const { return this->F_max; }

float GenericBoat::get_eta_max() const { return this->eta_max; }    

void GenericBoat::print_params() {
    std::cout << "Radius: " << this->radius << " m\n" << endl;
    std::cout << "Mass: " << this->mass << " kg\n" << endl;
    std::cout << "Inertia: " << this->inertia << " kg*m^2\n" << endl;
    std::cout << "Mu_l (Linear Drag): " << this->mu_l << " kg/m\n" << endl;
    std::cout << "Mu_ct (Cross Track Drag): " << this->mu_ct << " kg/m\n" << endl;
    std::cout << "Mu_r (Rotational Drag): " << this->mu_r << " kg*m^2\n" << endl;
}

void GenericBoat::set_pos(Vector3f pos) {
    this->pos = pos;
}

void GenericBoat::set_vel(Vector3f vel) {
    this->vel = vel;
}

float wrap_theta(float theta) {
    theta = fmod(theta, 2 * PI); // Normalize theta within [-2PI, 2PI]
    if (theta > PI) {
        theta -= 2 * PI; // Adjust if theta is in (PI, 2PI]
    } else if (theta < -PI) {
        theta += 2 * PI; // Adjust if theta is in [-2PI, -PI)
    }
    // cout << theta << endl;
    return theta;
}

int sign(float x) {
    if (x > 0) return 1;
    else if (x < 0) return -1;
    return 0;
}



