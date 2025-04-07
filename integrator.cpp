#include <Eigen/Dense>
using Eigen::MatrixXd;

// Euler integration
MatrixXd Euler_integration(const MatrixXd& state, const MatrixXd& state_der, double dt) {
    return (state + dt * state_der);
}

