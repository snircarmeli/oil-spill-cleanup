#include "integrator.h"

using std::make_pair;
using Eigen::MatrixXf;

// Euler integration
MatrixXf Euler_integration(const MatrixXf &state, const MatrixXf &state_der, float dt) {
    return state + dt * state_der;
}

class BoomBoatsDuo;
// Explicit instantiation of template functions
template MatrixXf RK4_integration<BoomBoatsDuo>(const Vector2f &control1, const Vector2f &control2, const MatrixXf &state, float dt, void* object);
template pair<MatrixXf, float> RK45_integration<BoomBoatsDuo>(const Vector2f &control1, const Vector2f &control2, const MatrixXf &state, float dt, void* object, json simulation_params);
