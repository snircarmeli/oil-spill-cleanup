#include "PID_controller.h"

#define PI 3.14159265358979323846

PID_Controller::PID_Controller() {
    // Default constructor
    kp << 0.1, 0.1;
    ki << 0.1, 0.1;
    kd << 0.1, 0.1;
    dt = 0.1;
    switch_tolerance = 5 * DEG2RAD;

    error = Matrix2X2d::Zero();
    // past_errors = MatrixXd::Zero(4, 10);

    integral = Matrix2X2d::Zero();
    derivative = Matrix2X2d::Zero();

    setpoint = Matrix3X2d::Zero();

    output = Matrix2X2d::Zero();

    output_limits << -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity();

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

PID_Controller::PID_Controller(Vector2d kp, Vector2d ki, Vector2d kd, double dt,
 double switch_tolerance, Vector2d output_limits) {
    // Constructor with parameters
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;
    this->switch_tolerance = switch_tolerance;

    error = Matrix2X2d::Zero();
    // past_errors = MatrixXd::Zero(4, 10);

    integral = Matrix2X2d::Zero();
    derivative = Matrix2X2d::Zero();

    setpoint = Matrix3X2d::Zero();

    output = Matrix2X2d::Zero();

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
    switch_tolerance = PID_params["switch_tolerance_deg"].get<double>() * PI / 180.0f;
    error = Matrix2X2d::Zero();
    // past_errors = MatrixXd::Zero(4, PID_params["num_past_errors"]);

    integral = Matrix2X2d::Zero();
    derivative = Matrix2X2d::Zero();

    setpoint = Matrix3X2d::Zero();

    output = Matrix2X2d::Zero();

    output_limits << PID_params["output_F_max"], PID_params["output_eta_max"];
}

PID_Controller::~PID_Controller() {
    // Destructor
}

void PID_Controller::set_gains(Vector2d kp, Vector2d ki, Vector2d kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PID_Controller::set_dt(double dt) {
    this->dt = dt;
}

void PID_Controller::set_setpoint(Matrix3X2d setpoint) {
    this->setpoint = setpoint;
}

void PID_Controller::set_output_limits(double F_max, double eta_max) {
    output_limits << F_max, eta_max;
}

void PID_Controller::set_output(Matrix2X2d output) {
    this->output = output;
}

void PID_Controller::set_switch_tolerance(double switch_tolerance) {
    this->switch_tolerance = switch_tolerance;
}

Vector2d PID_Controller::get_kp() const {
    return kp;
}

Vector2d PID_Controller::get_ki() const {
    return ki;
}

Vector2d PID_Controller::get_kd() const {
    return kd;
}

double PID_Controller::get_dt() const {
    return dt;
}

double PID_Controller::get_switch_tolerance() const {
    return switch_tolerance;
}

Matrix3X2d PID_Controller::get_setpoint() const {
    return setpoint;
}

Matrix2X2d PID_Controller::get_error() const {
    return error;
}

Matrix2X2d PID_Controller::get_integral() const {
    return integral;
}

Matrix2X2d PID_Controller::get_derivative() const {
    return derivative;
}

Vector2d PID_Controller::get_output_limits() const {
    return output_limits;
}

void PID_Controller::update(Vector3d boat1_pos, Vector3d boat2_pos,
 Vector3d boat1_vel, Vector3d boat2_vel, Matrix3X2d set_point,
  Matrix3X2d set_point_dot ) {
    // Calculate the error for boat 1. e_ct - error cross track, e_at - error along track
    double theta_ref1 = set_point(2, 0); 
    Matrix2X2d Rot;
    Rot << -cos(theta_ref1), -sin(theta_ref1), sin(theta_ref1), -cos(theta_ref1);
    Vector2d d = boat1_pos.head(2) - Vector2d(set_point(0, 0), set_point(1, 0));
    Vector2d err1 = Rot * d;
    //  e_at1 = err1(0);
    //  e_ct1 = err1(1);
    
    // Calculate the derivative of the error for boat 1
    Matrix2X2d M; M << 0, 1, -1, 0; // Can also be used in boat2
    Vector2d d_dot1 = boat1_vel.head(2) - Vector2d(set_point_dot(0, 0), set_point_dot(1, 0));
    double theta_ref1_dot = set_point_dot(2, 0);
    Vector2d err1_dot = Rot * d_dot1 + theta_ref1_dot * M * err1;

    // Calculate the error for boat 2
    double theta_ref2 = set_point(2, 1);
    Rot << -cos(theta_ref2), -sin(theta_ref2), sin(theta_ref2), -cos(theta_ref2);
    d = boat2_pos.head(2) - Vector2d(set_point(0, 1), set_point(1, 1));
    Vector2d err2 = Rot * d;
    //  e_at2 = err2(0);
    //  e_ct2 = err2(1);

    // Calculate the derivative of the error for boat 2
    Vector2d d_dot2 = boat2_vel.head(2) - Vector2d(set_point_dot(0, 1), set_point_dot(1, 1));
    double theta_ref2_dot = set_point_dot(2, 1);
    Vector2d err2_dot = Rot * d_dot2 + theta_ref2_dot * M * err2;


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
    Matrix2X2d KP; KP << kp(0), 0, 0, kp(1);
    Matrix2X2d KI; KI << ki(0), 0, 0, ki(1);
    Matrix2X2d KD; KD << kd(0), 0, 0, kd(1);
    output.col(0) = ( KP * error.col(0) + KI * integral.col(0) + KD * derivative.col(0) ); // Boat 1
    output.col(1) = ( KP * error.col(1) + KI * integral.col(1) + KD * derivative.col(1) ); // Boat 2

    output.row(1) *= -1; // Inverse steering for both boats

    // Wrap eta
    for (int i = 0; i < 2; i++) {
        output(1, i) = wrap_theta(output(1, i));
    }

    
    // Check the output limits
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            if (abs(output(i, j)) > output_limits(i)) {
                output(i, j) = output_limits(i) * sign(output(i, j));
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

Matrix2X2d PID_Controller::get_control(Vector3d boat1_pos, Vector3d boat2_pos,
 Vector3d boat1_vel, Vector3d boat2_vel, Matrix3X2d set_point,
  Matrix3X2d set_point_dot) {
    // Update the controller
    this->update(boat1_pos, boat2_pos, boat1_vel, boat2_vel, set_point,
     set_point_dot);
    // Get the output
    return output;
}

// Control method which uses aditional setpoint and setpoint_dot

void PID_Controller::update_with_next(Vector3d boat1_pos, Vector3d boat2_pos, Vector3d boat1_vel,
 Vector3d boat2_vel, Matrix3X2d set_point, Matrix3X2d set_point_dot,
  Matrix3X2d next_set_point, Matrix3X2d next_set_point_dot) {
    // Calculate the error for boat 1. e_ct - error cross track, e_at - error along track
    double theta_ref1 = set_point(2, 0); 
    Matrix2X2d Rot;
    Rot << -cos(theta_ref1), -sin(theta_ref1), sin(theta_ref1), -cos(theta_ref1);
    Vector2d d = boat1_pos.head(2) - Vector2d(set_point(0, 0), set_point(1, 0));
    Vector2d err1 = Rot * d;
    //  e_at1 = err1(0);
    //  e_ct1 = err1(1);
    
    // Calculate the derivative of the error for boat 1
    Matrix2X2d M; M << 0, 1, -1, 0; // Can also be used in boat2
    Vector2d d_dot1 = boat1_vel.head(2) - Vector2d(set_point_dot(0, 0), set_point_dot(1, 0));
    double theta_ref1_dot = set_point_dot(2, 0);
    Vector2d err1_dot = Rot * d_dot1 + theta_ref1_dot * M * err1;

    // Calculate the error for boat 2
    double theta_ref2 = set_point(2, 1);
    Rot << -cos(theta_ref2), -sin(theta_ref2), sin(theta_ref2), -cos(theta_ref2);
    d = boat2_pos.head(2) - Vector2d(set_point(0, 1), set_point(1, 1));
    Vector2d err2 = Rot * d;
    //  e_at2 = err2(0);
    //  e_ct2 = err2(1);

    // Calculate the derivative of the error for boat 2
    Vector2d d_dot2 = boat2_vel.head(2) - Vector2d(set_point_dot(0, 1), set_point_dot(1, 1));
    double theta_ref2_dot = set_point_dot(2, 1);
    Vector2d err2_dot = Rot * d_dot2 + theta_ref2_dot * M * err2;


    /* Do the same thing with error of next set point */

    double theta_ref1_next = next_set_point(2, 0); 
    Matrix2X2d Rot_next;
    Rot_next << -cos(theta_ref1_next), -sin(theta_ref1_next), sin(theta_ref1_next), -cos(theta_ref1_next);
    Vector2d d_next = boat1_pos.head(2) - Vector2d(next_set_point(0, 0), next_set_point(1, 0));
    Vector2d err1_next = Rot * d;
    //  e_at1 = err1(0);
    //  e_ct1 = err1(1);
    
    // Calculate the derivative of the error for boat 1
    Matrix2X2d M_next; M_next << 0, 1, -1, 0; // Can also be used in boat2
    Vector2d d_dot1_next = boat1_vel.head(2) - Vector2d(next_set_point_dot(0, 0), next_set_point_dot(1, 0));
    double theta_ref1_dot_next = next_set_point_dot(2, 0);
    Vector2d err1_dot_next = Rot_next * d_dot1_next + theta_ref1_dot_next * M_next * err1_next;

    // Calculate the error for boat 2
    double theta_ref2_next = next_set_point(2, 1);
    Rot_next << -cos(theta_ref2_next), -sin(theta_ref2_next), sin(theta_ref2_next), -cos(theta_ref2_next);
    d = boat2_pos.head(2) - Vector2d(next_set_point(0, 1), next_set_point(1, 1));
    Vector2d err2_next = Rot_next * d_next;
    //  e_at2 = err2(0);
    //  e_ct2 = err2(1);

    // Calculate the derivative of the error for boat 2
    Vector2d d_dot2_next = boat2_vel.head(2) - Vector2d(next_set_point_dot(0, 1), next_set_point_dot(1, 1));
    double theta_ref2_dot_next = next_set_point_dot(2, 1);
    Vector2d err2_dot_next = Rot_next * d_dot2_next + theta_ref2_dot_next * M_next * err2_next;
 
    // Update the error
    error.col(0) = err1 + PID_params["alpha"] * err1_next;
    error.col(1) = err2 + PID_params["alpha"] * err2_next;

    // Update the integral term
    integral.col(0) += (err1 + PID_params["alpha"] * err1_next) * dt;
    integral.col(1) += (err2 + PID_params["alpha"] * err2_next ) * dt;

    // Update the derivative term
    derivative.col(0) = err1_dot + PID_params["alpha"] * err1_dot_next;
    derivative.col(1) = err2_dot + PID_params["alpha"] * err2_dot_next;

    // Calculate the output for both boats
    Matrix2X2d KP; KP << kp(0), 0, 0, kp(1);
    Matrix2X2d KI; KI << ki(0), 0, 0, ki(1);
    Matrix2X2d KD; KD << kd(0), 0, 0, kd(1);
    output.col(0) = ( KP * error.col(0) + KI * integral.col(0) + KD * derivative.col(0) ); // Boat 1
    output.col(1) = ( KP * error.col(1) + KI * integral.col(1) + KD * derivative.col(1) ); // Boat 2

    output.row(1) *= -1; // Inverse steering for both boats

    // Wrap eta
    for (int i = 0; i < 2; i++) {
        output(1, i) = wrap_theta(output(1, i));
    }

    
    // Check the output limits
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            if (abs(output(i, j)) > output_limits(i)) {
                output(i, j) = output_limits(i) * sign(output(i, j));
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

Matrix2X2d PID_Controller::get_control_with_next(Vector3d boat1_pos, Vector3d boat2_pos, Vector3d boat1_vel,
 Vector3d boat2_vel, Matrix3X2d set_point, Matrix3X2d set_point_dot,
  Matrix3X2d next_set_point, Matrix3X2d next_set_point_dot) {
    // Update the controller
    this->update_with_next(boat1_pos, boat2_pos, boat1_vel, boat2_vel, set_point,
     set_point_dot, next_set_point, next_set_point_dot);
    // Get the output
    return output;
}
