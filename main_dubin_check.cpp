// #include "PID_controller.h"
#include "boom-boats-duo.h"
#include "dubin.h"
#include "helper_funcs.h"
#include "oil-spill.h"
#include "set_point_controller.h"
#include "MILP_Allocator.h"
#include "obstacle.h"

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
Vector3d get_setpoint_dot(MatrixXd path, int i, double dt) {
    Vector3d setpoint_dot;
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
    double T = simulation_params["time_duration"];
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

    // Oil spill parameters
    double attack_dist = params["oil_spill"]["attack_distance"];

    // Set an oil spill - read from spills folder
    string spill_folder = file_management_params["spills_folder"];
    // Print convex hull to file
    string convex_hull_folder = file_management_params["spills_convex_folder"];

    // Get the file "oilspill_017.txt" from the spills folder
    string filename_spill = spill_folder + "/oilspill_017.txt";
    OilSpill oil_spill(filename_spill);
    if (!oil_spill.is_valid_sequence()) {
        std::cerr << "Invalid sequence of points in file: " << filename_spill << std::endl;
        return 1;
    }
    // Get convex hull of the oil spill and print it to file
    oil_spill.calculate_convex_hull();

    // Get the file "obstacle_001.txt" from the obstacles folder
    string obstacles_folder = file_management_params["obstacles_folder"];
    string filename_obstacle = obstacles_folder + "/obstacle_001.txt";
    Obstacle obstacle(filename_obstacle);
    if (!obstacle.is_valid_sequence()) {
        std::cerr << "Invalid sequence of points in file: " << filename_obstacle << std::endl;
        return 1;
    }
    // Get convex hull of the obstacle and print it to file
    obstacle.calculate_convex_hull();
    
    double convex_hull_radius = oil_spill.get_convex_hull_radius();
    Vector2d convex_hull_centroid = oil_spill.get_convex_hull_centroid();
    // Get the file name without the ".txt" extension, adjust it
    string convex_file_name = "oilspill_017_convex.txt";
    oil_spill.print_convex_hull_to_file(convex_hull_folder, convex_file_name);

    // // Iterate over the files in the spills folder
    // for (const auto &entry : fs::directory_iterator(spill_folder)) {
    //     // Check if the file is a regular file
    //     if (entry.is_regular_file()) {
    //         string filename = entry.path().string();
    //         try {
    //             OilSpill oil_spill(filename);
    //             if (!oil_spill.is_valid_sequence()) {
    //                 cout << "Invalid sequence of points in file: " << filename << endl;
    //                 cout << endl;
    //                 cout.flush();
    //             }
    //             else {
    //                 // Get convex hull of the oil spill and print it to file
    //                 oil_spill.calculate_convex_hull();
    //                 // Get the file name without the ".txt" extension, adjust it
    //                 string convex_file_name = entry.path().stem().string() + "_convex";
    //                 oil_spill.print_convex_hull_to_file(convex_hull_folder, convex_file_name);
    //             }
    //         } catch (const std::exception &e) {
    //             cout << "Error processing file: " << filename << ". Error: " << e.what() << endl;
    //             cout << endl;
    //             cout.flush();
    //             // Delete the file
    //             if (!fs::remove(filename)) {
    //                 cout << "Failed to delete file: " << filename << endl;
    //                 cout << endl;
    //                 cout.flush();
    //             }
    //         }
    //     }
    // }

    // Boundry conditions for the Dubin's path
    double qi[3];
    qi[0] = dubin_params["initial_position"][0];
    qi[1] = dubin_params["initial_position"][1];
    qi[2] = dubin_params["initial_position"][2];

    
    // qf[0] = dubin_params["final_position"][0];
    // qf[1] = dubin_params["final_position"][1];
    // qf[2] = dubin_params["final_position"][2];

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

    double qf1[3];
    // Get spill angle of attack
    double attack_angle1 = oil_spill.angle_of_attack().first;
    qf1[0] = convex_hull_centroid[0] - (convex_hull_radius + attack_dist) * cos(attack_angle1);
    qf1[1] = convex_hull_centroid[1] - (convex_hull_radius + attack_dist) * sin(attack_angle1);
    qf1[2] = attack_angle1;

    double qf2[3];
    double attack_angle2 = wrap_theta(oil_spill.angle_of_attack().first + PI);
    qf2[0] = convex_hull_centroid[0] - (convex_hull_radius + attack_dist) * cos(attack_angle2);
    qf2[1] = convex_hull_centroid[1] - (convex_hull_radius + attack_dist) * sin(attack_angle2);
    qf2[2] = attack_angle2;


    
    // Get length of the path with regular attack angle
    DubinsPath path;
    int ret = dubins_init(qi, qf1, rho, &path);
    if (ret != EDUBOK) {
        std::cerr << "Failed to initialize Dubins path with original angle of attack." << std::endl;
        return 1;
    }
    double path_length1 = dubins_path_length(&path);
    // Get length of the path with inverse attack angle
    ret = dubins_init(qi, qf2, rho, &path);
    if (ret != EDUBOK) {
        std::cerr << "Failed to initialize Dubins path with inverse angle of attack." << std::endl;
        return 1;
    }
    double path_length2 = dubins_path_length(&path);


    // Compare to get shortest path
    if (path_length1 < path_length2) {
        ret = dubins_init(qi, qf1, rho, &path);
        if (ret != EDUBOK) {
            std::cerr << "Failed to initialize Dubins path with original angle of attack." << std::endl;
            return 1;
        }
    } else {
        ret = dubins_init(qi, qf2, rho, &path);
        if (ret != EDUBOK) {
            std::cerr << "Failed to initialize Dubins path with original angle of attack." << std::endl;
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
            std::cerr << "Failed to sample Dubins path." << std::endl;
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

    // // Calculate the reference_dot in global frame
    // MatrixXd path_points_dot_L(3, num_points - 1);
    // MatrixXd path_points_dot_R(3, num_points - 1);

    // path_points_dot_L = path_der_global(path_L, step_size);
    // path_points_dot_R = path_der_global(path_R, step_size);

    // MatrixXd path_L_local_frame = glob_path_points_2_local_frame_vel(path_L, step_size);
    // MatrixXd path_R_local_frame = glob_path_points_2_local_frame_vel(path_R, step_size);
    // MatrixXd path_local_frame = glob_path_points_2_local_frame_vel(path_points, step_size);

    // Save the global path to a file
    std::string folder_name = "DubinPath";
    std::string file_name = "dubin_path";
    save_to_file(file_name, folder_name, path_points, path_R, path_L);

    // // Save the local frame path to a file
    // std::string folder_name_local = "DubinPathLocal";
    // std::string file_name_local = "dubin_path_local";
    // save_to_file(file_name_local, folder_name_local, path_local_frame, path_R_local_frame, path_L_local_frame);

    // Create controller
    // PID_Controller controller(params_path);
    // NonLinearController controller(params_path);
    SetPointController controller(params_path, path_L, path_R, *duo);

    // Create the vector of duo, currently only one duo
    std::vector<BoomBoatsDuo> duos;
    duos.push_back(*duo);
    // Create the vector of oil spills, currently only one oil spill
    std::vector<OilSpill> oil_spills;
    oil_spills.push_back(oil_spill);
    // Create the vector of obstacles, currently empty
    std::vector<Obstacle> obstacles;

    // Start tracking problem
    int numSteps = static_cast<int>(T / dt) + 1;
    RowVectorXd t = RowVectorXd::LinSpaced(numSteps, 0.0f, T);
    MatrixXd boat_data = MatrixXd::Zero(numSteps, 9);
    string foldername = file_management_params["output_folder"];
    erase_folder_content(foldername);
    
    // Print time every k iterations
    int k_itr = simulation_params["print_interval"];
    // int check_valid_interval = simulation_params["check_valid_interval"];
    std::string integration_method = simulation_params["integration_method"];

    // data file for duo
    string filename = "Duo0.txt";

    cout << "Running simulation..." << std::endl << std::endl;
    cout.flush(); // Force immediate display of the output
    int cnt = 0;
    // int jump_index = params["PID"]["jump_index"];

    // // Setpoint
    // Vector3d setpoint_L = Vector3d::Zero();
    // Vector3d setpoint_R = Vector3d::Zero();
    // Matrix3X2d setpoint; // First column is for left ship, second column is for right ship
    // Vector3d setpoint_dot_L = Vector3d::Zero(); 
    // Vector3d setpoint_dot_R = Vector3d::Zero();
    // Matrix3X2d setpoint_dot; // First column is for left ship, second column is for right ship

    // // Next setpoint
    // Vector3d next_setpoint_L = Vector3d::Zero();
    // Vector3d next_setpoint_R = Vector3d::Zero();
    // Matrix3X2d next_setpoint; // First column is for left ship, second column is for right ship
    // Vector3d next_setpoint_dot_L = Vector3d::Zero();
    // Vector3d next_setpoint_dot_R = Vector3d::Zero();
    // Matrix3X2d next_setpoint_dot; // First column is for left ship, second column is for right ship

    // Vector3d boat1_pos = Vector3d::Zero();
    // Vector3d boat2_pos = Vector3d::Zero();
    // Vector3d boat1_vel = Vector3d::Zero();
    // Vector3d boat2_vel = Vector3d::Zero();
    Matrix2X2d control = Matrix2X2d::Zero();
    Vector2d control1 = Vector2d::Zero();
    Vector2d control2 = Vector2d::Zero();
    
    // Show percent of path completed
    // double path_percent = 0.0;


    // Check the MILP allocator
    // vector<MatrixXi> x_solutions = allocate_duos(duos, oil_spills, obstacles);

    while ( (duo->get_time() < T) ) { // && (cnt < num_points) ) {
        // Print current state to file
        duo->print_to_file(filename, foldername);
        if (cnt % k_itr == 0) {
            cout << "Simulation time: " << std::fixed << std::setprecision(2)
                 << std::setw(6) << duo->get_time() << " [s] out of "
                 << std::setw(6) << T << " [s]" << std::endl;
            // cout << "Path completion: " << path_percent << "%" << std::endl;

            cout.flush(); // Force flush the buffer
        }

        // Get the setpoint
        // setpoint_L = path_L.col(cnt);
        // setpoint_R = path_R.col(cnt);
        // setpoint.col(0) = setpoint_L;
        // setpoint.col(1) = setpoint_R;

        // cout << "Good so far1" << endl; cout.flush();
        // Get the setpoint dot
        // setpoint_dot_L = path_points_dot_L.col(cnt);
        // cout << "Good so far2" << endl; cout.flush();
        // setpoint_dot_R = path_points_dot_R.col(cnt);
        // cout << "Good so far3" << endl; cout.flush();


        // // Get the next setpoint
        // int next_index = cnt + jump_index;
        // if (next_index >= num_points) {
        //     next_index = num_points - 1;
        // }
        // next_setpoint_L = path_L.col(next_index);
        // next_setpoint_R = path_R.col(next_index);
        // next_setpoint.col(0) = next_setpoint_L;
        // next_setpoint.col(1) = next_setpoint_R;

        // // Get the next setpoint dot
        // next_setpoint_dot_L = get_setpoint_dot(path_L, next_index, dt);
        // next_setpoint_dot_R = get_setpoint_dot(path_R, next_index, dt);
        // next_setpoint_dot.col(0) = next_setpoint_dot_L;
        // next_setpoint_dot.col(1) = next_setpoint_dot_R;

        // Get the odometry
        // boat1_pos = duo->get_boat1().get_pos();
        // boat2_pos = duo->get_boat2().get_pos();
        // boat1_vel = duo->get_boat1().get_vel();
        // boat2_vel = duo->get_boat2().get_vel();

        // if (PID_params["use_next_setpoint"]) {
        //     control = controller.get_control_with_next(boat1_pos, boat2_pos, boat1_vel, boat2_vel, setpoint, setpoint_dot, next_setpoint, next_setpoint_dot);
        // } else {
        //     control = controller.get_control(boat1_pos, boat2_pos, boat1_vel, boat2_vel, setpoint, setpoint_dot);
        // }
        
        // Use non linear controller
        // control = controller.update(*duo);

        // use SetPointController
        control = controller.update(*duo);

        // Unpack the control
        control1 = control.col(0);
        control2 = control.col(1);

        // // Print control of boat 2
        // cout << "Force of boat 2: " << control2(0) << endl;
        // cout << "Steering angle of boat 2: " << control2(1) * RAD2DEG << endl;
        // cout.flush();

        // testing
        // double deg2rad = PI / 180;
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

        // print velocities of the boats in the local frame
        // cout << "Boat 1 velocity in local frame: " << duo->get_boat1().get_frame_vel().transpose() << "\n" << endl;
        // cout << "Boat 2 velocity in local frame: " << duo->get_boat2().get_frame_vel().transpose() << "\n" << endl;
        // cout << endl;
        // cout.flush();

        // Propagate the system
        duo->propagate(dt, control1, control2, integration_method);

        // Check validity of the state
        // if (cnt % check_valid_interval == 0) {
        //     if (!duo->is_valid_state()) {
        //         cout << "Invalid state detected at time: " << duo->get_time() << std::endl;
        //         cout.flush();
        //     }
        // }

        cnt += 1; 
        // path_percent += 100.0 / num_points;
    }

    // Delete pointers
    delete boat;
    delete duo;
}

