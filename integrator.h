#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <Eigen/Dense>

// For JSON parameters parsing
#include "json/json.hpp"
using json = nlohmann::json;

using Eigen::Vector2f;
using Eigen::MatrixXf;
using std::pair;

MatrixXf Euler_integration(const MatrixXf &state, const MatrixXf &state_der, float dt);

template <typename T>
MatrixXf RK4_integration(const Vector2f &control1, const Vector2f &control2, const MatrixXf &state, float dt, void* object) {
    // Check if the object is not null and has a state derivative function
    if (!object) {
        throw std::runtime_error("Object is null");
    }
    T* obj = static_cast<T*>(object);
    MatrixXf k1 = obj->state_der(control1, control2, state);
    MatrixXf k2 = obj->state_der(control1, control2, state + (dt / 2) * k1);
    MatrixXf k3 = obj->state_der(control1, control2, state + (dt / 2) * k2);
    MatrixXf k4 = obj->state_der(control1, control2, state + dt * k3);

    return state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
}

template <typename T>
pair<MatrixXf, float> RK45_integration(const Vector2f &control1, const Vector2f &control2, const MatrixXf &state, float dt, void* object, json simulation_params) {
    // Check if the object is not null and has a state derivative function
    if (!object) {
        throw std::runtime_error("Object is null");
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
    MatrixXf state_5th = MatrixXf::Zero(state.rows(), state.cols());
    int max_iterations = simulation_params["RK45_max_iterations"];
    float tolerance = simulation_params["RK45_tolerance"];
    T* obj = static_cast<T*>(object);
    while (cnt < max_iterations) {
        // Initialize RK stages (k1 to k6)
        MatrixXf k1 = obj->state_der(control1, control2, state);
        MatrixXf k2 = obj->state_der(control1, control2, state + current_dt * b[1][0] * k1);
        MatrixXf k3 = obj->state_der(control1, control2, state + current_dt * (b[2][0] * k1 + b[2][1] * k2));
        MatrixXf k4 = obj->state_der(control1, control2, state + current_dt * (b[3][0] * k1 + b[3][1] * k2 + b[3][2] * k3));
        MatrixXf k5 = obj->state_der(control1, control2, state + current_dt * (b[4][0] * k1 + b[4][1] * k2 + b[4][2] * k3 + b[4][3] * k4));
        MatrixXf k6 = obj->state_der(control1, control2, state + current_dt * (b[5][0] * k1 + b[5][1] * k2 + b[5][2] * k3 + b[5][3] * k4 + b[5][4] * k5));

        // Compute the RK45 fourth-order and fifth-order solutions
        MatrixXf state_4th = state + current_dt * (c4[0] * k1 + c4[1] * k2 + c4[2] * k3 + c4[3] * k4 + c4[4] * k5);
        state_5th = state + current_dt * (c5[0] * k1 + c5[1] * k2 + c5[2] * k3 + c5[3] * k4 + c5[4] * k5 + c5[5] * k6);

        // Compute the error
        MatrixXf error = state_5th - state_4th;
        float TE = error.norm(); // Total error magnitude

        // Check if the error is within the tolerance
        if (TE <= tolerance) {
            // Accept the step and return the fifth-order solution
            return std::make_pair(state_5th, current_dt);
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
    return std::make_pair(state_5th, current_dt);
}

#endif // INTEGRATOR_H