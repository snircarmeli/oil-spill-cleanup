#include "generic-boat.h"
#include "boom-boats-duo.h"
#include "boom-boat.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <filesystem> 

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
    
    BoomBoat *boat = new BoomBoat();
    size_t num_links = 6;
    float L = 0.5;
    BoomBoatsDuo* duo = new BoomBoatsDuo(*boat, *boat, num_links, L, 1.0, 1.0,
     1.0, Vector2f(0.0, 0.0), 0.0);
    delete boat;
    int num_duos = 1;
    BoomBoatsDuo** duo_arr = new BoomBoatsDuo*[num_duos];
    duo_arr[0] = duo;

    // boat->print_status();
    
    float T = 30;
    float dt = 1e-2;
    int numSteps = static_cast<int>(T / dt) + 1;
    RowVectorXf t = RowVectorXf::LinSpaced(numSteps, 0.0f, T);

    MatrixXf control = MatrixXf::Zero(numSteps, 2);

    MatrixXf boat_data = MatrixXf::Zero(numSteps, 9);
    string foldername = "DuosData";
    erase_folder_content(foldername);

    // Print time every k iterations
    int k = 500;

    std::cout << "Running simulation..." << std::endl << std::endl;
    std::cout.flush(); // Force immediate display of the output

    for (int i = 0; i < numSteps; i++) {
        if (i % k == 0) {
            std::cout << "Simulation time: " 
                    << std::fixed << std::setprecision(2) 
                    << std::setw(6) << i * dt << " [s] out of "
                    << std::setw(6) << T << " [s]" << std::endl;
            std::cout.flush(); // Force flush the buffer
}

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
    
    // delete boat;
    delete duo;
    delete duo_arr;
    return 0;

}