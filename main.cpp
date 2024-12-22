#include "generic-boat.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>

using std::sin;
using std::vector;
using Matrix = vector<vector<float>>;

using Eigen::Vector3f;
using Eigen::Vector2f;
using Matrix2x3f = Eigen::Matrix<float, 2, 3>;
using Eigen::RowVectorXf;
using Eigen::MatrixXf;

const double PI = 3.141592653589793;

int main(int argc, char* argv[]) {
    // Check if the correct number of arguments is provided
    if (argc < 10) {
        // std::cerr << "No parameters input" << std::endl;
    }
    
    GenericBoat *boat = new GenericBoat();
    boat->print_params();
    
    float T = 30;
    float dt = 1e-2;
    int numSteps = static_cast<int>(T / dt) + 1;
    RowVectorXf t = RowVectorXf::LinSpaced(numSteps, 0.0f, T);

    MatrixXf control = MatrixXf::Zero(numSteps, 2);

    MatrixXf boat_data = MatrixXf::Zero(numSteps, 9);

    for (int i = 0; i < numSteps; i++) {
        control(i,0) = 1000;
        control(i, 1) = -1 * PI / 9;

        boat->propogate(control.row(i), dt);
        Vector3f tmp_pos = boat->get_pos();
        Vector3f tmp_vel = boat->get_vel();
        
        boat_data.row(i) << tmp_pos(0), tmp_pos(1), tmp_pos(2),  // Position X, Y, Theta
                            tmp_vel(0), tmp_vel(1), tmp_vel(2),  // Velocity X, Y, Angular velocity
                            t(i),                               // Time step
                            control(i, 0), control(i, 1);       // Control inputs (force, eta)
    }

  
   // Write the matrix to a file
    ofstream file("output.txt");
    if (file.is_open()) {
        cout << "File opened successfully." << endl; // Debugging statement

        // Directly write the matrix to the file
        file << boat_data << endl;

        file.close();
        cout << "File written and closed successfully." << endl; // Debugging statement
    } else {
        cerr << "Unable to open file for writing." << endl;
    }
    
    delete boat;
    return 0;

}