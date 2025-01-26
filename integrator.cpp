#include <Eigen/Dense>
using Eigen::MatrixXf;

// Euler integration
MatrixXf Euler_integration(const MatrixXf& state, const MatrixXf& state_der, float dt) {
    return (state + dt * state_der);
}