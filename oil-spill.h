#ifndef OIL_SPILL_H
#define OIL_SPILL_H

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>
#include <iostream>


#include "helper_funcs.h" // Helper functions
#include "obstacle.h" // Obstacle class

// Parsing json parameters
#include "json/json.hpp"
using json = nlohmann::json;

using std::cout;
using std::endl;
using std::string;
using std::ifstream;
using std::vector;
using std::sort;
using std::ofstream;
using std::pair;
using std::make_pair;

using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::VectorXi;
using Eigen::VectorXd;
using Eigen::Matrix2d;

using std::istringstream;

class OilSpill {
private:
    // Member variables
    MatrixXd perimeter_points; // Perimeter of the oil spill [m]
    MatrixXd convex_hull; // Convex hull of the oil spill
    double volume; // Volume of the oil spill [m^3]
    json oil_spill_params;

public:
    // Constructors
    OilSpill(); // Default constructor
    OilSpill(const OilSpill &oil_spill); // Copy constructor
    OilSpill(MatrixXd perimeter_points, MatrixXd convex_hull, double volume); // Parameterized constructor
    OilSpill(string filename); // Constructor with filename

    // Destructor
    ~OilSpill();

    // Assignment operator
    OilSpill &operator=(const OilSpill &oil_spill);

    // load oil spill parameters from a file
    void load_oil_spill_params(string filename);

    // Getter and Setter for perimeter_points
    MatrixXd get_perimeter_points() const;
    void set_perimeter_points(MatrixXd perimeter_points);

    // Getter and Setter for convex_hull
    MatrixXd get_convex_hull() const;
    void set_convex_hull(MatrixXd convex_hull);

    // Getter and Setter for volume
    double get_volume() const;
    void set_volume(double volume);

    // Get convex hull of the oil spill
    void calculate_convex_hull();

    // Getter for the spill centroid
    Vector2d get_spill_centroid() const;

    // Get the convex hull radius
    double get_convex_hull_radius() const;

    // Getter for the convex hull centroid
    Vector2d get_convex_hull_centroid() const;

    // Getter for the risk factor of the oil spill
    double get_risk_factor(const vector<Obstacle>& obstacles) const;

    // Expand the oil spill by a factor, change the convex hull also if not empty
    void expand_spill(double factor);

    // Check if sequence of points is valid: if it does not intersect itself,
    //  and has at least 3 points
    bool is_valid_sequence() const;

    // Utility function to display OilSpill status
    void print_status() const;

    // Print convex hull to a file
    void print_convex_hull_to_file(string foldername, string filename) const;

    // Length of boom needed to contain the oil spill from a given orientation 
    double convex_hull_line_length(double orientation) const;

    // Get the angle of attack of the oil spill and the boom length needed
    pair<double, double> angle_of_attack() const;
};

#endif // OIL_SPILL_H