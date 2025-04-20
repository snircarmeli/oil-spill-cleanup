// CPP file for obstacle class

#include "obstacle.h"

// Default constructor
Obstacle::Obstacle() : crucial(false), weight(0.0) {
    // Set perimeter_points to an empty matrix
    this->perimeter_points = MatrixXd(0, 2);
    // Set convex_hull to an empty matrix
    this->convex_hull = MatrixXd(0, 2);
    // this->load_obstacle_params("params.json");
}

// Copy constructor
Obstacle::Obstacle(const Obstacle &obstacle) {
    this->perimeter_points = obstacle.perimeter_points;
    this->convex_hull = obstacle.convex_hull;
    this->crucial = obstacle.crucial;
    this->weight = obstacle.weight;
    this->convex_hull = obstacle.convex_hull;
}
    // this->load_obstacle_params("params.json");


// Parameterized constructor
Obstacle::Obstacle(MatrixXd perimeter_points, MatrixXd convex_hull, bool crucial) {
    this->perimeter_points = perimeter_points;
    this->convex_hull = convex_hull;
    this->crucial = crucial;
    this->weight = 0.0; // Default weight
    this->convex_hull = calculate_convex_hull(this->perimeter_points); // From helper_funcs.h
    // this->load_obstacle_params("params.json");
}

// Constructor with filename
Obstacle::Obstacle(string filename) {
    // First line is the crucial flag 0/1
    // Second line is the weight of the obstacle
    // Third line is number of points
    // Fourth line and onwards are the coordinates of points
    ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file at Obstacle::Obstacle");
    }
    file >> this->crucial;
    // Check if crucial is 0 or 1
    if (this->crucial != 0 && this->crucial != 1) {
        file.close();
        throw std::runtime_error("Invalid crucial flag at Obstacle::Obstacle");
    }
    // Read the weight of the obstacle
    file >> this->weight;
    // Check if weight is non-negative
    if (this->weight < 0) {
        file.close();
        throw std::runtime_error("Negative weight at Obstacle::Obstacle");
    }
    // Read the number of points
    int num_points;
    file >> num_points;
    // Check if num_points is an integer
    if (file.fail()) {
        file.close();
        throw std::runtime_error("Incomplete number of points at Obstacle::Obstacle");
    }
    // Check if num_points is non-negative
    if (num_points < 0) {
        file.close();
        throw std::runtime_error("Negative number of points at Obstacle::Obstacle");
    }

    // The rest of the file contains the coordinates of the points
    MatrixXd points = MatrixXd::Zero(num_points, 2);
    int cnt = 0;
    while (!file.eof()) {
        file >> points(cnt, 0) >> points(cnt, 1);
        cnt++;
        if (file.fail()) {
            break; // Break if reading fails
        }
    }
    file.close();
    // Check if cnt is less than num_points
    if (cnt < num_points) {
        throw std::runtime_error("Greater number of points reported at file " + filename + " at Obstacle::Obstacle");
    }
    // Check if cnt is greater than num_points
    if (cnt > num_points) {
        throw std::runtime_error("Smaller number of points reported at file " + filename + " at Obstacle::Obstacle");
    }

    this->perimeter_points = points;
    this->convex_hull = calculate_convex_hull(this->perimeter_points); // From helper_funcs.h
    // this->load_obstacle_params("params.json");
}

// Destructor
Obstacle::~Obstacle() {}

// Assignment operator
Obstacle &Obstacle::operator=(const Obstacle &obstacle) {
    if (this != &obstacle) {
        this->perimeter_points = obstacle.perimeter_points;
        this->convex_hull = obstacle.convex_hull;
        this->crucial = obstacle.crucial;
        this->weight = obstacle.weight;
        this->convex_hull = obstacle.convex_hull;
    }
    return *this;
}

// Getter and Setter for perimeter_points
MatrixXd Obstacle::get_perimeter_points() const {
    return this->perimeter_points;
}

void Obstacle::set_perimeter_points(const MatrixXd &points) {
    this->perimeter_points = points;
}

// Getter and Setter for convex_hull
MatrixXd Obstacle::get_convex_hull() const {
    return this->convex_hull;
}

void Obstacle::set_convex_hull(const MatrixXd &hull) {
    this->convex_hull = hull;
}

// Getter and Setter for crucial flag
bool Obstacle::is_crucial() const {
    return this->crucial;
}
void Obstacle::set_crucial(bool is_crucial) {
    this->crucial = is_crucial;
}

// Getter and Setter for weight
double Obstacle::get_weight() const {
    // If the obstacle is crucial, return the weight
    // Else, return 1
    if (this->crucial) {
        return this->weight;
    } else {
        return 1.0;
    }
}
void Obstacle::set_weight(double weight) {
    // Check if weight is less than 1
    if (weight < 1) {
        throw std::runtime_error("Weight is less than 1 at Obstacle::set_weight");
    }
    this->weight = weight;
}
