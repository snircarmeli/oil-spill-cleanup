#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <Eigen/Dense>
#include <type_traits>
#include <iostream>

// For JSON parameters parsing
#include "json/json.hpp"
using json = nlohmann::json;

using Eigen::Vector2f;
using Eigen::MatrixXf;
using std::pair;
using std::cout;
using std::endl;
using std::make_pair;

// Euler integration
MatrixXf Euler_integration(const MatrixXf& state, const MatrixXf& state_der, float dt) {
    return (state + dt * state_der);
}

// Runge-Kutta methods

// RK2 integration
template <typename T>
MatrixXf RK2_integration(const Vector2f& control1,
 const Vector2f& control2, const MatrixXf& state, float dt,
  T* object) {
    if (!object) {
        throw std::runtime_error("Object is null");
    }
    // Define constants for RK2 coefficients
    const float a[] = {0.0f, 0.5f};
    const float b[] = {0.0f, 1.0f};
    // State derucation functions does not depend on time
    // const float c[] = {0.5f, 0.5f};

    // Initialize RK stages (k1 and k2)
    MatrixXf k1 = object->state_der(control1, control2, state);
    MatrixXf k2 = object->state_der(control1, control2, state + dt * a[1] * k1);

    // Compute the RK2 solution
    MatrixXf state_new = state + dt * (b[0] * k1 + b[1] * k2);

    return state_new;
}

// RK3 integration
template <typename T>
// RK3 integration
MatrixXf RK3_integration(const Vector2f& control1, const Vector2f& control2,
 const MatrixXf& state, float dt, T* object) {
    if (!object) {
        throw std::runtime_error("Object is null");
    }
    // Define constants for RK3 coefficients
    const float a[] = {0.0f, 0.5f, 1.0f};
    const float b[] = {1.0f / 6.0f, 2.0f / 3.0f, 1.0f / 6.0f};
    // State derivation functions does not depend on time
    // const float c[] = {1.0f / 6.0f, 2.0f / 3.0f, 1.0f / 6.0f};

    // Initialize RK stages (k1, k2, and k3)
    MatrixXf k1 = object->state_der(control1, control2, state);
    MatrixXf k2 = object->state_der(control1, control2, state + dt * a[1] * k1);
    MatrixXf k3 = object->state_der(control1, control2, state + dt * (a[2] * k1 + a[3] * k2));

    // Compute the RK3 solution
    MatrixXf state_new = state + dt * (b[0] * k1 + b[1] * k2 + b[2] * k3);

    return state_new;
}

// RK4 integration
template <typename T>
MatrixXf RK4_integration(const Vector2f &control1, const Vector2f &control2, const MatrixXf &state, float dt, T* object) {
    // Check if the object is not null and has a state derivative function
    if (!object) {
        throw std::runtime_error("Object is null");
    }
    MatrixXf k1 = object->state_der(control1, control2, state);
    MatrixXf k2 = object->state_der(control1, control2, state + (dt / 2) * k1);
    MatrixXf k3 = object->state_der(control1, control2, state + (dt / 2) * k2);
    MatrixXf k4 = object->state_der(control1, control2, state + dt * k3);
    return state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
}

// RK5 integration
template <typename T>
MatrixXf RK5_integration(const Vector2f& control1, const Vector2f& control2,
 const MatrixXf& state, float dt, T* object) {
    // Check if the object is not null and has a state derivative function
    if (!object) {
        throw std::runtime_error("Object is null");
    }
    // Define constants for RK5 coefficients
    const float a[] = {0.25f, 0.0f, 0.25f, 0.125f, 0.0f, 0.375f, 0.5f, 0.0f, -1.5f, 2.0f};
    const float b[] = {7.0f / 90.0f, 0.0f, 16.0f / 45.0f, 2.0f / 15.0f, 1.0f / 10.0f};
    // State derivation functions does not depend on time
    // const float c[] = {};

    // Initialize RK stages (k1 to k5)
    MatrixXf k1 = object->state_der(control1, control2, state);
    MatrixXf k2 = object->state_der(control1, control2, state + dt * a[0] * k1);
    MatrixXf k3 = object->state_der(control1, control2, state + dt * (a[1] * k1 + a[2] * k2));
    MatrixXf k4 = object->state_der(control1, control2, state + dt * (a[3] * k1 + a[4] * k2 + a[5] * k3));
    MatrixXf k5 = object->state_der(control1, control2, state + dt * (a[6] * k1 + a[7] * k2 + a[8] * k3 + a[9] * k4));

    // Compute the RK5 solution
    MatrixXf state_new = state + dt * (b[0] * k1 + b[1] * k2 + b[2] * k3 + b[3] * k4 + b[4] * k5);
    
    return state_new;
}

// RK6 integration
template <typename T>
MatrixXf RK6_integration(const Vector2f& control1, const Vector2f& control2,
 const MatrixXf& state, float dt, T* object) {
    // Check if the object is not null and has a state derivative function
    if (!object) {
        throw std::runtime_error("Object is null");
    }
    // Note: Apparently, there are many RK6 methods. This is one of the methods
    // working with 6 stages and a fixed time step
    // Define constants for RK6 coefficients
    const float a[] = {0.210312f,
        0.115470f, 0.293846f,
        0.063382f, 0.0f, 0.551618f,
        0.010161f, 0.0f, 0.234853f, 0.561986f, 
        0.002773f, 0.0f, 0.0f, 0.289325f, 0.431902f};
    const float b[] = {0.020151f, 0.0f, 0.237139f, 0.327040f, 0.265617f, 0.150053f};
    
    // State derivation functions does not depend on time
    // const float c[] = {};

    // Initialize RK stages (k1 to k6)
    MatrixXf k1 = object->state_der(control1, control2, state);
    MatrixXf k2 = object->state_der(control1, control2, state + dt * a[0] * k1);
    MatrixXf k3 = object->state_der(control1, control2, state + dt * (a[1] * k1 + a[2] * k2));
    MatrixXf k4 = object->state_der(control1, control2, state + dt * (a[3] * k1 + a[4] * k2 + a[5] * k3));
    MatrixXf k5 = object->state_der(control1, control2, state + dt * (a[6] * k1 + a[7] * k2 + a[8] * k3 + a[9] * k4));
    MatrixXf k6 = object->state_der(control1, control2, state + dt * (a[10] * k1 + a[11] * k2 + a[12] * k3 + a[13] * k4 + a[14] * k5));

    // Compute the RK6 solution
    MatrixXf state_new = state + dt * (b[0] * k1 + b[1] * k2 + b[2] * k3 + b[3] * k4 + b[4] * k5 + b[5] * k6);

    return state_new;
}

// RK 4-5 integration - Adaptive time step
template <typename T>
std::pair<MatrixXf, float> RK45_integration(const Vector2f& control1,
 const Vector2f& control2, const MatrixXf& state, float dt,
  T* object, json simulation_params) {
    // Check if the object is not null and has a state derivative function
    if (!object) {
        throw std::runtime_error("Object is null");
    }
    // Check if the simulation parameters are not null
    if (simulation_params.empty()) {
        throw std::runtime_error("Simulation parameters are null (RK 4-5)");
    }
    // Define constants for RK45 coefficients

    const float b[6][5] = {
        {},
        {0.25f},
        {3.0f / 32.0f, 9.0f / 32.0f},
        {1932.0f / 2197.0f, -7200.0f / 2197.0f, 7296.0f / 2197.0f},
        {439.0f / 216.0f, -8.0f, 3680.0f / 513.0f, -845.0f / 4104.0f},
        {-8.0f / 27.0f, 2.0f, -3544.0f / 2565.0f, 1859.0f / 4104.0f, -11.0f / 40.0f}};
    const float c4[] = {25.0f / 216.0f, 0.0f, 1408.0f / 2565.0f, 2197.0f / 4104.0f, -1.0f / 5.0f};
    const float c5[] = {16.0f / 135.0f, 0.0f, 6656.0f / 12825.0f, 28561.0f / 56430.0f, -9.0f / 50.0f, 2.0f / 55.0f};

    float current_dt = dt;
    int cnt = 0;
    // MatrixXf def_state = RK4_integration(control1, control2, state, current_dt, boom_boats_duo);
    MatrixXf state_5th = MatrixXf::Zero(state.rows(), state.cols());
    int max_iterations = simulation_params["RK45_max_iterations"];
    float tolerance = simulation_params["RK45_tolerance"];
    while (cnt < max_iterations) {
        // Initialize RK stages (k1 to k6)
        MatrixXf k1 = object->state_der(control1, control2, state);
        MatrixXf k2 = object->state_der(control1, control2, state + current_dt * b[1][0] * k1);
        MatrixXf k3 = object->state_der(control1, control2, state + current_dt * (b[2][0] * k1 + b[2][1] * k2));
        MatrixXf k4 = object->state_der(control1, control2, state + current_dt * (b[3][0] * k1 + b[3][1] * k2 + b[3][2] * k3));
        MatrixXf k5 = object->state_der(control1, control2, state + current_dt * (b[4][0] * k1 + b[4][1] * k2 + b[4][2] * k3 + b[4][3] * k4));
        MatrixXf k6 = object->state_der(control1, control2, state + current_dt * (b[5][0] * k1 + b[5][1] * k2 + b[5][2] * k3 + b[5][3] * k4 + b[5][4] * k5));

        // Compute the RK45 fourth-order and fifth-order solutions
        MatrixXf state_4th = state + current_dt * (c4[0] * k1 + c4[1] * k2 + c4[2] * k3 + c4[3] * k4 + c4[4] * k5);
        MatrixXf state_5th = state + current_dt * (c5[0] * k1 + c5[1] * k2 + c5[2] * k3 + c5[3] * k4 + c5[4] * k5 + c5[5] * k6);

        // Compute the error
        MatrixXf error = state_5th - state_4th;
        float TE = error.norm(); // Total error magnitude

        // Check if the error is within the tolerance
        if (TE <= tolerance) {
            // Accept the step and return the fifth-order solution
            return make_pair(state_5th, current_dt);
        } else {
            // Reject the step and reduce the step size
            current_dt = 0.9f * current_dt * std::pow(tolerance / TE, 1.0f / 5.0f);
            // Ensure the step size is not too small
            if (current_dt < 1e-6f) {
            throw std::runtime_error("Step size too small");
            }
        }
        cnt++;
    }
    return make_pair(state_5th, current_dt);
}

#endif // INTEGRATOR_H