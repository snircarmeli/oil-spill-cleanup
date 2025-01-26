#include "PID_controller.h"
#include "boom-boats-duo.h"
#include "dubin.h"

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

using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::MatrixXf;
using Eigen::RowVectorXf;

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

void erase_folder_content(const std::string& foldername) {
    if (!fs::exists(foldername)) {
        std::cerr << "Folder does not exist: " << foldername << std::endl;
        return;
    }

    if (!fs::is_directory(foldername)) {
        std::cerr << "Path is not a folder: " << foldername << std::endl;
        return;
    }

    for (const auto& entry : fs::directory_iterator(foldername)) {
        if (entry.is_regular_file()) { // Only process regular files
            if (!fs::remove(entry.path())) {
                std::cerr << "Failed to delete file: " << entry.path() << std::endl;
            }
        }
    }

    // cout << "All files in folder " << foldername << " have been removed." << std::endl << std::endl;
    // cout.flush(); // Force immediate display of the output
}

// Calculate the dot of the setpoint at the given index - Numerical differentiation
Vector3f get_setpoint_dot(MatrixXf path, int i, float dt) {
    Vector3f setpoint_dot;
    if (i == 0) {
        setpoint_dot << path(0, i + 1) - path(0, i), path(1, i + 1) - path(1, i), path(2, i + 1) - path(2, i);
        setpoint_dot /= dt;
    } else if (i == path.cols() - 1) {
        setpoint_dot << path(0, i) - path(0, i - 1), path(1, i) - path(1, i - 1), path(2, i) - path(2, i - 1);
        setpoint_dot /= dt;
    } else {
        setpoint_dot << path(0, i + 1) - path(0, i - 1), path(1, i + 1) - path(1, i - 1), path(2, i + 1) - path(2, i - 1);
        setpoint_dot /= 2 * dt;
    }
    return setpoint_dot;
}

int main(int argc, char* argv[]) {
    // Get parameters from the JSON file
    // Path to the JSON file
    std::string params_path = "params.json";

    // Open the file and parse it
    std::ifstream file(params_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open parameters file " << params_path << std::endl;
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

    // Simulation parameters
    float T = simulation_params["time_duration"];
    float dt = simulation_params["time_step"];

    // Boom parameters
    size_t num_links = boom_params["num_links"];
    float L = boom_params["link_length"];
    float I = boom_params["inertia"];
    float m = boom_params["mass"];
    float k = boom_params["spring_constant"];
    float c = boom_params["damping_coefficient"];
    float mu_l = boom_params["drag_coefficients"]["linear"];
    float mu_ct = boom_params["drag_coefficients"]["cross_track"];
    float mu_r = boom_params["drag_coefficients"]["rotational"];

    // Boundry conditions
    float qi[3];
    qi[0] = dubin_params["initial_position"][0];
    qi[1] = dubin_params["initial_position"][1];
    qi[2] = dubin_params["initial_position"][2];

    float qf[3];
    qf[0] = dubin_params["final_position"][0];
    qf[1] = dubin_params["final_position"][1];
    qf[2] = dubin_params["final_position"][2];

    // Create duo
    BoomBoat *boat = new BoomBoat();
    // cout << "Managed to create boat" << endl;
    float orientation = qi[2] + PI / 2;
    Vector2f center = Vector2f(qi[0], qi[1]);
    BoomBoatsDuo* duo = new BoomBoatsDuo(*boat, *boat, num_links, L, mu_l,
     mu_ct, mu_r, I, m, k, c, center, orientation);

    // Create controller
    PID_Controller controller(params_path);

    // Plan path for right and left ship
    // int num_links = params["boom"]["num_links"];
    // float L = params["boom"]["link_length"];
    float boom_total_length = num_links * L;
    float dist_from_center_ratio = params["dubin"]["dist_from_center_ratio"];
    float dist_from_center = boom_total_length * dist_from_center_ratio;
    float dist_between_boats = dist_from_center / 2;

 
    // Rho value
    float rho = dist_between_boats; //dubin_params["min_turning_radius"];

    // Initialize the path
    DubinsPath path;
    int ret = dubins_init(qi, qf, rho, &path);
    if (ret != EDUBOK) {
        std::cerr << "Failed to initialize Dubins path." << std::endl;
        return 1;
    }

    // Sample the path and save it in matrix
    float step_size = dubin_params["sample_step_size"];
    float q[3];
    int num_points = static_cast<int>(dubins_path_length(&path) / step_size) + 1;
    MatrixXf path_points(3, num_points);
    for (float t = 0; t < dubins_path_length(&path); t += step_size) {
        ret = dubins_path_sample(&path, t, q);
        if (ret != EDUBOK) {
            std::cerr << "Failed to sample Dubins path." << std::endl;
            return 1;
        }
        // cout << "Sampled point: " << q[0] << ", " << q[1] << ", " << q[2] << std::endl;
        path_points.col(t / step_size) << q[0], q[1], wrap_theta(q[2]);
    }

    
    // Create two matrices for path points
    MatrixXf path_L(3, num_points);
    MatrixXf path_R(3, num_points);
    float orientation_path = 0.0;
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

    // Save the path to a file
    std::string folder_name = "DubinPath";
    std::string file_name = "dubin_path";
    save_to_file(file_name, folder_name, path_points, path_R, path_L);

    // Start tracking problem
    int numSteps = static_cast<int>(T / dt) + 1;
    RowVectorXf t = RowVectorXf::LinSpaced(numSteps, 0.0f, T);
    MatrixXf boat_data = MatrixXf::Zero(numSteps, 9);
    string foldername = file_management_params["output_folder"];
    erase_folder_content(foldername);
    
    // Print time every k iterations
    int k_itr = simulation_params["print_interval"];
    int check_valid_interval = simulation_params["check_valid_interval"];
    std::string integration_method = simulation_params["integration_method"];

    // data file for duo
    string filename = "Duo0.txt";

    cout << "Running simulation..." << std::endl << std::endl;
    cout.flush(); // Force immediate display of the output
    int cnt = 0;
    int jump_index = params["PID"]["jump_index"];

    Vector3f setpoint_L = Vector3f::Zero();
    Vector3f setpoint_R = Vector3f::Zero();
    Matrix3X2f setpoint; // First column is for left ship, second column is for right ship
    Vector3f setpoint_dot_L = Vector3f::Zero(); 
    Vector3f setpoint_dot_R = Vector3f::Zero();
    Matrix3X2f setpoint_dot; // First column is for left ship, second column is for right ship

    Vector3f boat1_pos = Vector3f::Zero();
    Vector3f boat2_pos = Vector3f::Zero();
    Vector3f boat1_vel = Vector3f::Zero();
    Vector3f boat2_vel = Vector3f::Zero();
    Matrix2X2f control = Matrix2X2f::Zero();
    Vector2f control1 = Vector2f::Zero();
    Vector2f control2 = Vector2f::Zero();
    
    while ( (duo->get_time() < T) & (cnt < num_points) ) {
        // Print current state to file
        duo->print_to_file(filename, foldername);
        if (cnt % k_itr == 0) {
            cout << "Simulation time: " 
                    << std::fixed << std::setprecision(2) 
                    << std::setw(6) << duo->get_time() << " [s] out of "
                    << std::setw(6) << T << " [s]" << std::endl;
            cout.flush(); // Force flush the buffer  
        }

        // Get the setpoint
        setpoint_L = path_L.col(cnt);
        setpoint_R = path_R.col(cnt);
        setpoint.col(0) = setpoint_L;
        setpoint.col(1) = setpoint_R;

        // Get the setpoint dot
        setpoint_dot_L = get_setpoint_dot(path_L, cnt, dt);
        setpoint_dot_R = get_setpoint_dot(path_R, cnt, dt);
        setpoint_dot.col(0) = setpoint_dot_L;
        setpoint_dot.col(1) = setpoint_dot_R;

        // Get the control
        boat1_pos = duo->get_boat1().get_pos();
        boat2_pos = duo->get_boat2().get_pos();
        boat1_vel = duo->get_boat1().get_vel();
        boat2_vel = duo->get_boat2().get_vel();
        control = controller.get_control(boat1_pos, boat2_pos, boat1_vel, boat2_vel, setpoint, setpoint_dot);
        // Unpack the control
        control1 = control.col(0);
        control2 = control.col(1);

        // testing
        // float deg2rad = PI / 180;
        // control1[0] = 1200;
        // control2[0] = 1200;
        // control1[1] = 1 * deg2rad;
        // control2[1] = -1 * deg2rad;

        // Print Forces for boat 1 and boat 2
        

        // // Check validity of the control inputs
        // if (!duo->get_boat1().is_valid_control(control1) && duo->get_time() > 0) {
        //     // cout<< "Invalid control input for boat 1 at time: " << duo->get_time() << " [s]" << endl;
        //     // cout.flush();
        // }
        // if (!duo->get_boat2().is_valid_control(control2) && duo->get_time() > 0) {
        // //     cout<< "Invalid control input for boat 2 at time: " << duo->get_time() << " [s]" << endl;
        // //     cout.flush();
        // }

        // Propagate the system
        duo->propagate(dt, control1, control2, integration_method);

        // Check validity of the state
        if (cnt % check_valid_interval == 0) {
            if (!duo->is_valid_state()) {
                cout << "Invalid state detected at time: " << duo->get_time() << std::endl;
                cout.flush();
            }
        }
        cnt += jump_index; 
    

    }

    // Delete pointers
    delete boat;
    delete duo;
}

