#include "generic-boat.h"
#include "boom-boats-duo.h"
#include "boom-boat.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <filesystem> 

// For JSON parameters parsing
#include "json/json.hpp"
using json = nlohmann::json;

using std::sin;
using std::vector;
using Matrix = vector<vector<float>>;
using Eigen::Vector3f;
using Eigen::Vector2f;
using Matrix2x3f = Eigen::Matrix<float, 2, 3>;
using Eigen::RowVectorXf;
using Eigen::MatrixXf;

namespace fs = std::filesystem;

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

    std::cout << "All files in folder " << foldername << " have been removed." << std::endl << std::endl;
    std::cout.flush(); // Force immediate display of the output
}

const float PI = 3.141592653589793;

int main(int argc, char* argv[]) {
    // Check if the correct number of arguments is provided
    if (argc < 10) {
        // std::cerr << "No parameters input" << std::endl;
    }

    // Access parameters

    // Path to the JSON file
    std::string file_path = "params.json";

    // Open the file and parse it
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open parameters file " << file_path << std::endl;
        return 1;
    }

    json params;
    file >> params;

    json generic_boat_params = params["generic_boat"];
    json boom_params = params["boom"];
    json duo_params = params["boom_boats_duo"];
    json simulation_params = params["simulation"];
    json file_management_params = params["file_management"];


    //Simulation parameters
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

    BoomBoat *boat = new BoomBoat();
    cout << "Managed to create boat" << endl;
    float orientation = 0.0;
    BoomBoatsDuo* duo = new BoomBoatsDuo(*boat, *boat, num_links, L, mu_l,
     mu_ct, mu_r, I, m, k, c, Vector2f(1.0, 1.0), orientation);
    // duo->print_status();

    int num_duos = 1;

    BoomBoatsDuo** duo_arr = new BoomBoatsDuo*[num_duos];
    duo_arr[0] = duo;
    // duo_arr[0]->print_status();

    
    // float T = 30;
    // float dt = 1e-2;
    int numSteps = static_cast<int>(T / dt) + 1;
    RowVectorXf t = RowVectorXf::LinSpaced(numSteps, 0.0f, T);

    MatrixXf control1 = MatrixXf::Zero(numSteps, 2);
    MatrixXf control2 = MatrixXf::Zero(numSteps, 2);
    // set all forces to 1000 and all steering angles to 0
    control1.col(0) = 1500 * VectorXf::Ones(numSteps);
    control2.col(0) = 1500 * VectorXf::Ones(numSteps);

    MatrixXf boat_data = MatrixXf::Zero(numSteps, 9);
    string foldername = file_management_params["output_folder"];
    erase_folder_content(foldername);

    // Print time every k iterations
    int k_itr = simulation_params["print_interval"];

    std::cout << "Running simulation..." << std::endl << std::endl;
    std::cout.flush(); // Force immediate display of the output

    for (int i = 0; i < numSteps; i++) {
        if (i % k_itr == 0) {

            std::cout << "Simulation time: " 
                    << std::fixed << std::setprecision(2) 
                    << std::setw(6) << i * dt << " [s] out of "
                    << std::setw(6) << T << " [s]" << std::endl;
            std::cout.flush(); // Force flush the buffer
        }

        Vector2f control1_vec = control1.row(i);
        Vector2f control2_vec = control2.row(i);
        duo_arr[0]->propagate(dt, control1_vec, control2_vec);


        // control(i,0) = 1000;
        // control(i, 1) = -1 * PI / 9;

        // boat->propogate(control.row(i), dt);
        // Vector3f tmp_pos = boat->get_pos();
        // Vector3f tmp_vel = boat->get_vel();
        
        // boat_data.row(i) << tmp_pos(0), tmp_pos(1), tmp_pos(2),  // Position X, Y, Theta
        //                     tmp_vel(0), tmp_vel(1), tmp_vel(2),  // Velocity X, Y, Angular velocity
        //                     t(i),                               // Time step
        //                     control(i, 0), control(i, 1);       // Control inputs (force, eta)
        
        for (int j = 0; j < num_duos; ++j) {
            string filename = "Duo" + std::to_string(j) + ".txt";
            duo_arr[j]->print_to_file(filename, foldername);
        }
    }
    std::cout << std::endl; // Going down a line
    std::cout.flush(); // Force immediate display of the output


  
//    Write the matrix to a file
   
//     ofstream file("output.txt");
//     if (file.is_open()) {
//         cout << "File output.txt opened successfully." << endl; // Debugging statement

//         // Directly write the matrix to the file
//         file << boat_data << endl;

//         file.close();
//         cout << "File output.txt written and closed successfully." << endl; // Debugging statement
//     } else {
//         cerr << "Unable to open file output.txt for writing." << endl;
//     }
    
    delete boat;
    delete duo;
    delete duo_arr;
    return 0;

}