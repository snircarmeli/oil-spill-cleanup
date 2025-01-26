#include "PID_controller.h"

#define PI 3.14159265358979323846

PID_Controller::PID_Controller() {
    // Default constructor
    kp << 0.1, 0.1;
    ki << 0.1, 0.1;
    kd << 0.1, 0.1;
    dt = 0.1;

    error = Matrix2X2f::Zero();
    // past_errors = MatrixXf::Zero(4, 10);

    integral = Matrix2X2f::Zero();
    derivative = Matrix2X2f::Zero();

    setpoint = Matrix3X2f::Zero();

    output = Matrix2X2f::Zero();

    output_limits << -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity();

    // Load the parameters from the JSON file
    std::ifstream file("params.json");
    if (!file.is_open()) {
        std::cerr << "Failed to open parameters file in PID constructor" << endl;
        return;
    }

    json params;
    file >> params;
    file.close();

    json PID_params = params["PID"];
}

PID_Controller::PID_Controller(Vector2f kp, Vector2f ki, Vector2f kd, float dt,
 Vector2f output_limits) {
    // Constructor with parameters
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;

    error = Matrix2X2f::Zero();
    // past_errors = MatrixXf::Zero(4, 10);

    integral = Matrix2X2f::Zero();
    derivative = Matrix2X2f::Zero();

    setpoint = Matrix3X2f::Zero();

    output = Matrix2X2f::Zero();

    output_limits = output_limits;

    // Load the parameters from the JSON file
    std::ifstream file("params.json");
    if (!file.is_open()) {
        std::cerr << "Failed to open parameters file in PID constructor" << endl;
        return;
    }

    json params;
    file >> params;
    file.close();

    json PID_params = params["PID"];
}

PID_Controller::PID_Controller(std::string filename) {
    // Constructor which loads the parameters from a JSON file
    // Open the file and parse it
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open parameters file in PID constructor" << filename << endl;
        return;
    }

    json params;
    file >> params;
    file.close();

    PID_params = params["PID"];

    kp << PID_params["Kp"][0], PID_params["Kp"][1];
    ki << PID_params["Ki"][0], PID_params["Ki"][1];
    kd << PID_params["Kd"][0], PID_params["Kd"][1];
    dt = params["simulation"]["time_step"];

    error = Matrix2X2f::Zero();
    // past_errors = MatrixXf::Zero(4, PID_params["num_past_errors"]);

    integral = Matrix2X2f::Zero();
    derivative = Matrix2X2f::Zero();

    setpoint = Matrix3X2f::Zero();

    output = Matrix2X2f::Zero();

    output_limits << PID_params["output_F_max"], PID_params["output_eta_max"];
}

PID_Controller::~PID_Controller() {
    // Destructor
}

void PID_Controller::set_gains(Vector2f kp, Vector2f ki, Vector2f kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PID_Controller::set_dt(float dt) {
    this->dt = dt;
}

void PID_Controller::set_setpoint(Matrix3X2f setpoint) {
    this->setpoint = setpoint;
}

void PID_Controller::set_output_limits(float F_max, float eta_max) {
    output_limits << F_max, eta_max;
}

void PID_Controller::set_output(Matrix2X2f output) {
    this->output = output;
}

Vector2f PID_Controller::get_kp() const {
    return kp;
}

Vector2f PID_Controller::get_ki() const {
    return ki;
}

Vector2f PID_Controller::get_kd() const {
    return kd;
}

float PID_Controller::get_dt() const {
    return dt;
}

Matrix3X2f PID_Controller::get_setpoint() const {
    return setpoint;
}

Matrix2X2f PID_Controller::get_error() const {
    return error;
}

Matrix2X2f PID_Controller::get_integral() const {
    return integral;
}

Matrix2X2f PID_Controller::get_derivative() const {
    return derivative;
}

Vector2f PID_Controller::get_output_limits() const {
    return output_limits;
}

void PID_Controller::update(Vector3f boat1_pos, Vector3f boat2_pos,
 Vector3f boat1_vel, Vector3f boat2_vel, Matrix3X2f next_set_point,
  Matrix3X2f next_set_point_dot) {
    // Calculate the error for boat 1. e_ct - error cross track, e_at - error along track
    float theta_ref1 = next_set_point(2, 0); 
    Matrix2X2f Rot;
    Rot << -cos(theta_ref1), -sin(theta_ref1), sin(theta_ref1), -cos(theta_ref1);
    Vector2f d = boat1_pos.head(2) - Vector2f(next_set_point(0, 0), next_set_point(1, 0));
    Vector2f err1 = Rot * d;
    //  e_at1 = err1(0);
    //  e_ct1 = err1(1);
    
    // Calculate the derivative of the error for boat 1
    Matrix2X2f M; M << 0, 1, -1, 0; // Can also be used in boat2
    Vector2f d_dot1 = boat1_vel.head(2) - Vector2f(next_set_point_dot(0, 0), next_set_point_dot(1, 0));
    float theta_ref1_dot = next_set_point_dot(2, 0);
    Vector2f err1_dot = Rot * d_dot1 + theta_ref1_dot * M * err1;

    // Calculate the error for boat 2
    float theta_ref2 = next_set_point(2, 1);
    Rot << -cos(theta_ref2), -sin(theta_ref2), sin(theta_ref2), -cos(theta_ref2);
    d = boat2_pos.head(2) - Vector2f(next_set_point(0, 1), next_set_point(1, 1));
    Vector2f err2 = Rot * d;
    //  e_at2 = err2(0);
    //  e_ct2 = err2(1);

    // Calculate the derivative of the error for boat 2
    Vector2f d_dot2 = boat2_vel.head(2) - Vector2f(next_set_point_dot(0, 1), next_set_point_dot(1, 1));
    float theta_ref2_dot = next_set_point_dot(2, 1);
    Vector2f err2_dot = Rot * d_dot2 + theta_ref2_dot * M * err2;


    // Update the error
    error.col(0) = err1;
    error.col(1) = err2;

    // Update the integral term
    integral.col(0) += err1 * dt;
    integral.col(1) += err2 * dt;

    // Update the derivative term
    derivative.col(0) = err1_dot;
    derivative.col(1) = err2_dot;
    
    // Calculate the output for both boats
    Matrix2X2f KP; KP << kp(0), 0, 0, kp(1);
    Matrix2X2f KI; KI << ki(0), 0, 0, ki(1);
    Matrix2X2f KD; KD << kd(0), 0, 0, kd(1);
    output.col(0) = ( KP * error.col(0) + KI * integral.col(0) + KD * derivative.col(0) ); // Boat 1
    output.col(1) = ( KP * error.col(1) + KI * integral.col(1) + KD * derivative.col(1) ); // Boat 2

    output.row(1) *= -1; // Inverse steering for both boats

    // Wrap eta
    for (int i = 0; i < 2; i++) {
        output(1, i) = wrap_eta(output(1, i));
    }

    
    // Check the output limits
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            if (abs(output(i, j)) > output_limits(i)) {
                output(i, j) = output_limits(i) * val_sign(output(i, j));
            }
        }
    }

    // Debug prints
    if (PID_params["print_PID"]) {
        cout << "Updated Error for Boat 1: " << error.col(0).transpose() << endl;
        cout << "Updated Error for Boat 2: " << error.col(1).transpose() << endl;
        cout << "Integral Term for Boat 1: " << integral.col(0).transpose() << endl;
        cout << "Integral Term for Boat 2: " << integral.col(1).transpose() << endl;
        cout << "Derivative Term for Boat 1: " << derivative.col(0).transpose() << endl;
        cout << "Derivative Term for Boat 2: " << derivative.col(1).transpose() << endl;
        cout << "Force for boat 1: " << output.col(0).transpose() << endl;
        cout << "Force for boat 2: " << output.col(1).transpose() << endl;
        cout.flush();
        cout << endl;
    }
    
}

Matrix2X2f PID_Controller::get_control(Vector3f boat1_pos, Vector3f boat2_pos,
 Vector3f boat1_vel, Vector3f boat2_vel, Matrix3X2f next_set_point,
  Matrix3X2f next_set_point_dot) {
    // Update the controller
    this->update( boat1_pos, boat2_pos, boat1_vel, boat2_vel, next_set_point,
     next_set_point_dot);
    // Get the output
    return output;
}

// Helper functions

float wrap_eta(float angle) {
    // Wrap the angle between -pi and pi
    return fmod(angle + PI, 2 * PI) - PI;
}

int val_sign(float x) {
    if (x == 0) {
        return 0;
    }
    return (x > 0) - (x < 0);
}



