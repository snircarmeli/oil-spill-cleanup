/*
    Control Validation Module
    This module is responsible for validating the control system of the Boom-Boats Duo.
    It ensures that the control commands are executed correctly and that the system behaves as expected.

    Compile:
    g++ -Wall -g -std=c++17 control_validation.cpp -o control_validation.exe -I./json -I./eigen generic-boat.cpp boom-boat.cpp boom-boats-duo.cpp dubin.cpp helper_funcs.cpp integrator.cpp set_point_controller.cpp -lpthread

    */
#include "boom-boats-duo.h"
#include "dubin.h"
#include "helper_funcs.h"
#include "set_point_controller.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <filesystem> 


// For JSON parameters parsing
#include "json/json.hpp"
using json = nlohmann::json;

namespace fs = std::filesystem;

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::RowVectorXd;
using std::string;
using std::cerr;
using std::cout;
using std::endl;
using std::ifstream;
using std::exception;
using std::vector;
using std::fixed;
using std::setprecision;
using std::setw;

// Path types
#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

// The three segment types a path can be made up of
#define L_SEG (0)
#define S_SEG (1)
#define R_SEG (2)

#define PI 3.14159265358979323846

void erase_folder_content(const string& foldername) {
    if (!fs::exists(foldername)) {
        cerr << "Folder does not exist: " << foldername << endl;
        return;
    }

    if (!fs::is_directory(foldername)) {
        cerr << "Path is not a folder: " << foldername << endl;
        return;
    }

    for (const auto& entry : fs::directory_iterator(foldername)) {
        if (entry.is_regular_file()) { // Only process regular files
            if (!fs::remove(entry.path())) {
                cerr << "Failed to delete file: " << entry.path() << endl;
            }
        }
    }

    // cout << "All files in folder " << foldername << " have been removed." << endl << endl;
    // cout.flush(); // Force immediate display of the output
}

bool reached_last_setpoint(const BoomBoatsDuo& duo, const MatrixXd& path_1, const MatrixXd& path_2, double tolerance_d, int last_index) {
    Vector2d current_pos1 = duo.get_boat1().get_pos().head(2);
    Vector2d current_pos2 = duo.get_boat2().get_pos().head(2);

    Vector2d target_pos1 = path_1.col(last_index).head(2);
    Vector2d target_pos2 = path_2.col(last_index).head(2);

    double dist1 = (current_pos1 - target_pos1).norm();
    double dist2 = (current_pos2 - target_pos2).norm();

    if (dist1 < tolerance_d && dist2 < tolerance_d) {
        cout << "In function reached_last_setpoint: " << endl;
        cout << "Reached last setpoint" << endl;
        cout.flush(); // Force immediate display of the output
        return true;
    }
    return false;
}


int main(int argc, char* argv[]) {
    // Get parameters from the JSON file
    // Path to the JSON file
    string params_path = "params.json";

    // Open the file and parse it
    ifstream file(params_path);
    if (!file.is_open()) {
        cerr << "Failed to open parameters file " << params_path << endl;
        return 1;
    }

    json params;
    file >> params;
    file.close();

    json dubin_params = params["dubin"];

    // Create duo
    json boom_params = params["boom"];
    json duo_params = params["boom_boats_duo"];
    json simulation_params = params["simulation"];
    json file_management_params = params["file_management"];
    json control_validation_params = params["ControlValidation"];

    // Simulation parameters
    double T = control_validation_params["max_time"];
    double dt = simulation_params["time_step"];

    // Boom parameters
    size_t num_links = boom_params["num_links"];
    double L = boom_params["link_length"];
    double I = boom_params["inertia"];
    double m = boom_params["mass"];
    double k = boom_params["spring_constant"];
    double c = boom_params["damping_coefficient"];
    double mu_l = boom_params["drag_coefficients"]["linear"];
    double mu_ct = boom_params["drag_coefficients"]["cross_track"];
    double mu_r = boom_params["drag_coefficients"]["rotational"];



    

    // initial and final position
    double qi[3];
    double qf[3];
    double qf1[3];
    double qf2[3];
    for (int i = 0; i < 3; ++i) {
        qi[i] = control_validation_params["qi"][i];
        qf[i] = control_validation_params["qf"][i];

        qf1[i] = qf[i];
        qf2[i] = qf[i];
    }
  
    qf2[2] = wrap_theta(qf2[2] + PI); // Inverse angle of attack
    

    // Create duo
    BoomBoat *boat = new BoomBoat();
    // -x + PI / 2 --> To accomodate the orientation of the boat in its local frame
    // double orientation = wrap_theta(-qi[2] + PI / 2);
    double orientation = -qi[2] + PI / 2;
    Vector2d center = Vector2d(qi[0], qi[1]);
    BoomBoatsDuo* duo = new BoomBoatsDuo(*boat, *boat, num_links, L, mu_l,
     mu_ct, mu_r, I, m, k, c, center, orientation, duo_params["V_max"],
      duo_params["V_max"], duo_params["cleaning_rate"]);

    // Plan path for right and left ship
    // int num_links = params["boom"]["num_links"];
    // double L = params["boom"]["link_length"];
    double boom_total_length = num_links * L;
    double dist_from_center_ratio = params["dubin"]["dist_from_center_ratio"];
    double dist_from_center = boom_total_length * dist_from_center_ratio;
    double dist_between_boats = dist_from_center / 2;

 
    // Rho value
    double rho = dist_between_boats; //dubin_params["min_turning_radius"];



    
    // Get length of the path with regular attack angle
    DubinsPath path;
    int ret = dubins_init(qi, qf1, rho, &path);
    if (ret != EDUBOK) {
        cerr << "Failed to initialize Dubins path with original angle of attack." << endl;
        return 1;
    }
    double path_length1 = dubins_path_length(&path);
    // Get length of the path with inverse attack angle
    ret = dubins_init(qi, qf2, rho, &path);
    if (ret != EDUBOK) {
        cerr << "Failed to initialize Dubins path with inverse angle of attack." << endl;
        return 1;
    }
    double path_length2 = dubins_path_length(&path);


    // Compare to get shortest path
    if (path_length1 < path_length2) {
        ret = dubins_init(qi, qf1, rho, &path);
        if (ret != EDUBOK) {
            cerr << "Failed to initialize Dubins path with original angle of attack." << endl;
            return 1;
        }
    } else {
        ret = dubins_init(qi, qf2, rho, &path);
        if (ret != EDUBOK) {
            cerr << "Failed to initialize Dubins path with original angle of attack." << endl;
            return 1;
        }
    }
   
    // Sample the path and save it in matrix
    double step_size = dubin_params["sample_step_size"];
    double q[3];
    int num_points = static_cast<int>(dubins_path_length(&path) / step_size);
    MatrixXd path_points(3, num_points);
    for (double t = 0; t < dubins_path_length(&path); t += step_size) {
        ret = dubins_path_sample(&path, t, q);
        if (ret != EDUBOK) {
            cerr << "Failed to sample Dubins path." << endl;
            return 1;
        }
        path_points.col(t / step_size) << q[0], q[1], wrap_theta(q[2]);
    }

    // Check for jumps in the path
    double max_jump = dubin_params["max_jump"];
    path_points = check_path(path_points, max_jump);    

    
    // Create two matrices for path points
    MatrixXd path_L(3, num_points);
    MatrixXd path_R(3, num_points);
    double orientation_path = 0.0;
    // Fill the matrices with the path points
    for (int i = 0; i < num_points; i++) {
        orientation_path = path_points(2, i) - PI / 2;

        path_L(0, i) = path_points(0, i) - dist_between_boats * cos(orientation_path);
        path_L(1, i) = path_points(1, i) - dist_between_boats * sin(orientation_path);
        path_L(2, i) = path_points(2, i);
        
        path_R(0, i) = path_points(0, i) + dist_between_boats * cos(orientation_path);
        path_R(1, i) = path_points(1, i) + dist_between_boats * sin(orientation_path);
        path_R(2, i) = path_points(2, i);
    }

    // Save the global path to a file
    string folder_name = "DubinPath";
    string file_name = "dubin_path";
    save_to_file(file_name, folder_name, path_points, path_R, path_L);

    // Create controller
    SetPointController controller(params_path, path_L, path_R, *duo);

    // Start tracking problem
    // int numSteps = static_cast<int>(T / dt) + 1;
    // RowVectorXd t = RowVectorXd::LinSpaced(numSteps, 0.0f, T);
    // MatrixXd boat_data = MatrixXd::Zero(numSteps, 9);
    string foldername = file_management_params["output_folder"];
    erase_folder_content(foldername);
    
    // Print time every k iterations
    int k_itr = simulation_params["print_interval"];
    // int check_valid_interval = simulation_params["check_valid_interval"];
    string integration_method = simulation_params["integration_method"];

    // data file for duo
    string filename = "Duo0.txt";

    cout << "Running simulation..." << endl << endl;
    cout.flush(); // Force immediate display of the output
    int cnt = 0;

    Matrix2X2d control = Matrix2X2d::Zero();
    Vector2d control1 = Vector2d::Zero();
    Vector2d control2 = Vector2d::Zero();

    int last_index = num_points - 1; // Last index of the path
    double t = 0;

    // Start the simulation loop


    while ( /* !reached_last_setpoint(*duo, path_L, path_R, controller.get_tolerance_d(), last_index) && */ (t < T)) {
        // Print current state to file
        duo->print_to_file(filename, foldername);
        if (cnt % k_itr == 0) {
            cout << "Simulation time: " << fixed << setprecision(2)
                 << setw(6) << duo->get_time() << " [s]" << endl;
            // cout << "Path completion: " << path_percent << "%" << endl;

            cout.flush(); // Force flush the buffer
        }


        // use SetPointController
        control = controller.update(*duo);
        controller.print_to_file(file_management_params["Control_File"], file_management_params["Control_Folder"], *duo);

        // Unpack the control
        control1 = control.col(0);
        control2 = control.col(1);

        // // Print the control values with time
        // cout << "Time: " << fixed << setprecision(2) << t << " [s] - "
        //      << "Control1: (" << control1(0) << ", " << control1(1) << "), "
        //      << "Control2: (" << control2(0) << ", " << control2(1) << ")" << endl;
        // cout.flush(); // Force flush the buffer

        // Propagate the system
        duo->propagate(dt, control1, control2, integration_method);

        // Check validity of the state
        // if (cnt % check_valid_interval == 0) {
        //     if (!duo->is_valid_state()) {
        //         cout << "Invalid state detected at time: " << duo->get_time() << endl;
        //         cout.flush();
        //     }
        // }

        cnt += 1; 
        t += dt;
    
    }

    cout << "Reached last setpoint or maximum time reached." << endl;
    cout << "Final time: " << t << " [s]" << endl;
    cout.flush(); // Force flush the buffer

    // Delete pointers
    delete boat;
    delete duo;
}

