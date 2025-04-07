#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem> // For file operations
#include <cmath>
#include "boom-boats-duo.h"
// #include "boom-boats-duo.h"

// For JSON parameters parsing
#include "json/json.hpp"
using json = nlohmann::json;

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;

using Matrix2X2d = Matrix<double, 2, 2>;
using Matrix3X2d = Matrix<double, 3, 2>;

using std::string;
using std::endl;
using std::cout;

class PID_Controller {
private:
    Vector2d kp; // First for force, second for heading
    Vector2d ki; // First for force, second for heading
    Vector2d kd; // First for force, second for heading
    double dt;

    double switch_tolerance; // Tolerance for switching between setpoints

    Matrix2X2d error; // First column for left ship, second column for right ship
    Matrix2X2d integral; // First column for left ship, second column for right ship
    Matrix2X2d derivative; // First column for left ship, second column for right ship

    Matrix3X2d setpoint; // First column for left ship, second column for right ship

    Matrix2X2d output; // First column for left ship, second column for right ship 

    Vector2d output_limits; // F_max and eta_max

    json PID_params;

public:
    PID_Controller(); // Default constructor: load from JSON file
    PID_Controller(Vector2d kp, Vector2d ki, Vector2d kd, double dt,
       double switch_tolerance, Vector2d output_limits); // Constructor with parameters
    // Constructor which loads the parameters from a JSON file
    PID_Controller(string filename);
    ~PID_Controller();

    void set_gains(Vector2d kp, Vector2d ki, Vector2d kd);
    void set_dt(double dt);
    void set_setpoint(Matrix3X2d setpoint);
    void set_output_limits(double F_max, double eta_max);
    void set_output(Matrix2X2d output);
    void set_switch_tolerance(double switch_tolerance);

    Vector2d get_kp() const;
    Vector2d get_ki() const;
    Vector2d get_kd() const;
    double get_dt() const;
    double get_switch_tolerance() const;
    Matrix3X2d get_setpoint() const;
    Matrix2X2d get_error() const;
    Matrix2X2d get_integral() const;
    Matrix2X2d get_derivative() const;
    Vector2d get_output_limits() const;

    // Calculate errors and update the controller
    void update(Vector3d boat1_pos, Vector3d boat2_pos, Vector3d boat1_vel, 
     Vector3d boat2_vel, Matrix3X2d next_set_point, Matrix3X2d next_set_point_dot);

    Matrix2X2d get_control(Vector3d boat1_pos, Vector3d boat2_pos, Vector3d boat1_vel,
       Vector3d boat2_vel, Matrix3X2d set_point, Matrix3X2d set_point_dot);

    // Control method which uses aditional setpoint and setpoint_dot
    void update_with_next(Vector3d boat1_pos, Vector3d boat2_pos, Vector3d boat1_vel,
     Vector3d boat2_vel, Matrix3X2d set_point, Matrix3X2d set_point_dot,
     Matrix3X2d next_set_point, Matrix3X2d next_set_point_dot);
     
    Matrix2X2d get_control_with_next(Vector3d boat1_pos, Vector3d boat2_pos, Vector3d boat1_vel,
     Vector3d boat2_vel, Matrix3X2d set_point, Matrix3X2d set_point_dot,
      Matrix3X2d next_set_point, Matrix3X2d next_set_point_dot);
};  

#endif