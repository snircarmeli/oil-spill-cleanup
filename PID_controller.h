#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem> // For file operations

// #include "boom-boats-duo.h"

// For JSON parameters parsing
#include "json/json.hpp"
using json = nlohmann::json;

using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::VectorXf;
using Eigen::MatrixXf;
using Eigen::Matrix;

using Matrix2X2f = Matrix<float, 2, 2>;
using Matrix3X2f = Matrix<float, 3, 2>;

using std::string;
using std::endl;
using std::cout;

class PID_Controller {
private:
    Vector2f kp; // First for force, second for heading
    Vector2f ki; // First for force, second for heading
    Vector2f kd; // First for force, second for heading
    float dt;

    Matrix2X2f error; // First column for left ship, second column for right ship
    Matrix2X2f integral; // First column for left ship, second column for right ship
    Matrix2X2f derivative; // First column for left ship, second column for right ship

    Matrix3X2f setpoint; // First column for left ship, second column for right ship

    Matrix2X2f output; // First column for left ship, second column for right ship 

    Vector2f output_limits; // F_max and eta_max

    json PID_params;

public:
    PID_Controller(); // Default constructor: load from JSON file
    PID_Controller(Vector2f kp, Vector2f ki, Vector2f kd, float dt, 
    Vector2f output_limits); // Constructor with parameters
    // Constructor which loads the parameters from a JSON file
    PID_Controller(string filename);
    ~PID_Controller();

    void set_gains(Vector2f kp, Vector2f ki, Vector2f kd);
    void set_dt(float dt);
    void set_setpoint(Matrix3X2f setpoint);
    void set_output_limits(float F_max, float eta_max);
    void set_output(Matrix2X2f output);

    Vector2f get_kp() const;
    Vector2f get_ki() const;
    Vector2f get_kd() const;
    float get_dt() const;
    Matrix3X2f get_setpoint() const;
    Matrix2X2f get_error() const;
    Matrix2X2f get_integral() const;
    Matrix2X2f get_derivative() const;
    Vector2f get_output_limits() const;

    // Calculate errors and update the controller
    void update(Vector3f boat1_pos, Vector3f boat2_pos, Vector3f boat1_vel, 
    Vector3f boat2_vel, Matrix3X2f next_set_point, Matrix3X2f next_set_point_dot);
    Matrix2X2f get_control(Vector3f boat1_pos, Vector3f boat2_pos, Vector3f boat1_vel, 
    Vector3f boat2_vel, Matrix3X2f next_set_point, Matrix3X2f next_set_point_dot);
};


// Helper functions
float wrap_eta(float angle);
int val_sign(float x);
#endif