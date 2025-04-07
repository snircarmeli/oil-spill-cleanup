#ifndef NON_LINEAR_CONTROLLER_H
#define NON_LINEAR_CONTROLLER_H

#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem> // For file operations
#include <cmath>
#include "boom-boats-duo.h"
#include "helper_funcs.h"

// For JSON parameters parsing
#include "json/json.hpp"
using json = nlohmann::json;

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix;

using Matrix2X2d = Matrix<double, 2, 2>;
using Matrix3X2d = Matrix<double, 3, 2>;

using std::string;
using std::endl;
using std::cout;

class NonLinearController {
    private:
        // Controller parameters
        double k_u;
        double k_v;
        double k_theta;
        double epsilon; // For the calculation of denominators
        double ts; // Time step

        // Error values for each ship
        Vector2d e_u;
        Vector2d e_v;
        Vector2d e_r;
        Vector2d e_theta;

        Vector2d int_e_theta;

        Vector2d e_u_dot;
        Vector2d e_v_dot;

        // Control outputs
        Matrix2X2d output;

    public:
    // Constructor
    NonLinearController(string path_to_json);

    // Destructor
    ~NonLinearController();

    // Getters
    Vector2d get_e_u();
    Vector2d get_e_v();
    Vector2d get_e_r();
    Vector2d get_e_theta();

    // Update errors and step
    void update_errors(BoomBoatsDuo duo, Vector3d setpoint1,
        Vector3d setpoint2, Vector3d setpoint1_dot, Vector3d setpoint2_dot);
    
    // Calculate the closest links forces in local frame
    Matrix2X2d calculate_closest_links_forces(BoomBoatsDuo duo);

    // Calculate du and dv for each boat (Cancelling drag and link forces)
    Matrix2X2d calculate_du_dv(BoomBoatsDuo duo);

    // Calculate au and av
    Matrix2X2d calculate_au_av(BoomBoatsDuo duo, Vector3d setpoint1_dot,
        Vector3d setpoint2_dot);

    // After calculating au and av, calculate the control signals
    Matrix2X2d calculate_control_signals(Matrix2X2d a, Matrix2X2d d);

    // Calculate the control signals and update the controller
    Matrix2X2d update(BoomBoatsDuo duo, Vector3d setpoint1, Vector3d setpoint2, Vector3d setpoint1_dot, Vector3d setpoint2_dot);

};

#endif // NON_LINEAR_CONTROLLER_H