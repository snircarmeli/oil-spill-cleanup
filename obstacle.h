// This object will define the convex hull of  an obstacle, like an oil rig,
// a ship, etc. It will be used to prevent the boat from colliding with the obstacle,
// and will have an option to define this obstacle as crucial, like a reef area
// or a shoreline, where the spill is more dangerous.

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "helper_funcs.h" // Helper functions

#include <fstream> // For file I/O
// Parsing json parameters
#include "json/json.hpp"
using json = nlohmann::json;

using namespace std;
using std::vector;
using std::cout;
using std::endl;
using std::string;
using std::ifstream;

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::VectorXi;

class Obstacle {
private:
    // Member variables
    MatrixXd perimeter_points; // Perimeter of the obstacle [m]
    MatrixXd convex_hull; // Convex hull of the obstacle
    // json obstacle_params; // Parameters for the obstacle

    bool crucial; // Flag to indicate if the obstacle is crucial (like a reef or shoreline)
    double weight; // Importance of the obstacle in the simulation

public:
    // Constructors
    Obstacle(); // Default constructor
    Obstacle(const Obstacle &obstacle); // Copy constructor
    Obstacle(MatrixXd perimeter_points, MatrixXd convex_hull, bool crucial); // Parameterized constructor
    Obstacle(string filename); // Constructor with filename

    // Destructor
    ~Obstacle();

    // Assignment operator
    Obstacle &operator=(const Obstacle &obstacle);

    // // Load obstacle parameters from a file
    // void load_obstacle_params(string filename);

    // Getter and Setter for perimeter_points
    MatrixXd get_perimeter_points() const;
    void set_perimeter_points(const MatrixXd &points);

    // Getter and Setter for convex_hull
    MatrixXd get_convex_hull() const;
    void set_convex_hull(const MatrixXd &hull);

    // Getter and Setter for crucial flag
    bool is_crucial() const;
    void set_crucial(bool is_crucial);

    // Getter and Setter for weight
    double get_weight() const;
    void set_weight(double weight);

};

#endif // OBSTACLE_H