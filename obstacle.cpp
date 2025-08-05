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
    if (file.fail()) {
        file.close();
        throw std::runtime_error("Expected crucial flag at Obstacle::Obstacle(filename)");
    }
    // Check if crucial is 0 or 1
    if (this->crucial != 0 && this->crucial != 1) {
        file.close();
        throw std::runtime_error("Invalid crucial flag at Obstacle::Obstacle(filename)");
    }
    // Read the weight of the obstacle
    file >> this->weight;
    // Check if weight is non-negative
    if (this->weight < 0) {
        file.close();
        throw std::runtime_error("Negative weight at Obstacle::Obstacle(filename)");
    }

    int num_points;
    file >> num_points;
    // Check if num_points is an integer
    if (file.fail()) {
        file.close();
        throw std::runtime_error("Incomplete number of points at Obstacle::Obstacle(filename)");
    }

    // Check if num_points is non-negative
    if (num_points < 0) {
        file.close();
        throw std::runtime_error("Negative number of points at Obstacle::Obstacle(filename)");
    }
    
    // Consume the remaining newline character
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    // file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // The rest of the file contains the coordinates of the points
    this->perimeter_points = MatrixXd::Zero(num_points, 2);

    // --------------

    // Read the points
    string line;
    int cnt = 0;
    string test;
    int cnt2 = 0;
    
    
    while (getline(file, line)) {
        if (line.empty()) {
            continue; // Skip empty lines
        }
        // Check if line has only two numbers
        istringstream iss(line);
        // Read all data in iss to check if it has only two numbers
        cnt2 = 0;
        test = "";
        while(!iss.eof()) {
            iss >> test;
            // Check if test is a number
            if (canConvertToFloat(test)) {
                cnt2++;
                if (cnt2 > 2) {
                    throw std::runtime_error("Too many points in file " + filename + " at line " + std::to_string(cnt + 3) + " at Obstacle::Obstacle(filename)");
                }
            }
            else {
                throw std::runtime_error("Invalid data in file " + filename + " at line " + std::to_string(cnt + 3) + " at Obstacle::Obstacle(filename)");
            }
        }
        if (cnt2 < 2) {
            throw std::runtime_error("Invalid point in file " + filename + " at line " + std::to_string(cnt + 3) + " at Obstacle::Obstacle(filename)");
        }
        // If we reach here, iss has only two numbers
        double x, y;
        iss = istringstream(line);
        iss >> x >> y;
        if (cnt >= num_points) {
            throw std::runtime_error("Smaller number of points reported at file " + filename + " at line " + std::to_string(cnt + 3) + " at Obstacle::Obstacle(filename)");
        }
        this->perimeter_points.row(cnt) << x, y;
        cnt++;
    }
    // --------------
    file.close();
    // Check if cnt is less than num_points
    if (cnt < num_points) {
        throw std::runtime_error("Greater number of points reported at file " + filename + " at Obstacle::Obstacle(filename)");
    }

    // Calculate convex hull of the obstacle
    this->calculate_convex_hull();
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

// Calculate convex hull of the obstacle
void Obstacle::calculate_convex_hull() {
    // Check if the perimeter_points matrix is empty
    if (this->perimeter_points.rows() == 0) {
        throw std::runtime_error("Empty perimeter points matrix at OilSpill::get_convex_hull");
    }

    int n = perimeter_points.rows();

    // If there is 0-2 points, throw an error
    if (n <= 2){
        throw std::runtime_error("Less than 3 points in the perimeter points matrix at OilSpill::get_convex_hull");
    }

    // If there are 3 points, return the perimeter_points matrix
    if (n == 3) {
        this->convex_hull = this->perimeter_points;
        return;
    }
    
    MatrixXd pts(n, 2);
    // Copy perimeter_points to pts
    pts = perimeter_points;

    // Sort points lexicographically (first by x, then by y)
    VectorXi indices = VectorXi::LinSpaced(n, 0, n - 1);
    sort(indices.data(), indices.data() + indices.size(), [&](int a, int b) {
        return (pts(a, 0) < pts(b, 0)) || ((pts(a, 0) == pts(b, 0)) && (pts(a, 1) < pts(b, 1)));
    });

    // Lambda to compute the cross product of OA and OB
    auto cross = [&](int O, int A, int B) -> double {
        return (pts(A, 0) - pts(O, 0)) * (pts(B, 1) - pts(O, 1)) - (pts(A, 1) - pts(O, 1)) * (pts(B, 0) - pts(O, 0));
    };

    // Build the lower hull
    VectorXi lower(n);
    int lower_size = 0;
    for (int i = 0; i < n; i++) {
        int idx = indices(i);
        while (lower_size >= 2 && cross(lower(lower_size - 2), lower(lower_size - 1), idx) <= 0) {
            lower_size--;
        }
        lower(lower_size++) = idx;
    }

    // Build the upper hull
    VectorXi upper(n);
    int upper_size = 0;
    for (int i = n - 1; i >= 0; i--) {
        int idx = indices(i);
        while (upper_size >= 2 && cross(upper(upper_size - 2), upper(upper_size - 1), idx) <= 0) {
            upper_size--;
        }
        upper(upper_size++) = idx;
    }

    // Remove the last point of each list to avoid duplication of the first/last points.
    if (lower_size > 0) lower_size--;
    if (upper_size > 0) upper_size--;

    // Concatenate lower and upper to form the full convex hull.
    VectorXi hull(lower_size + upper_size);
    hull << lower.head(lower_size), upper.head(upper_size);

    // Convert the hull indices back to an MatrixXd
    MatrixXd hull_matrix(hull.size(), 2);
    for (int i = 0; i < hull.size(); i++) {
        hull_matrix(i, 0) = pts(hull(i), 0);
        hull_matrix(i, 1) = pts(hull(i), 1);
    }

    this->convex_hull = hull_matrix;
}

// Getter and Setter for convex_hull
MatrixXd Obstacle::get_convex_hull() const {
    return this->convex_hull;
}

void Obstacle::set_convex_hull(const MatrixXd &hull) {
    this->convex_hull = hull;
}

// Getter for obstacle centroid
Vector2d Obstacle::get_obstacle_centroid() const {
    // Compute the centroid of the obstacle
    Vector2d centroid = Vector2d::Zero();
    // Check if the perimeter_points matrix is empty
    if (this->perimeter_points.rows() == 0) {
        throw std::runtime_error("Empty perimeter points matrix at Obstacle::get_obstacle_centroid");
    }
    for (int i = 0; i < this->perimeter_points.rows(); i++) {
        centroid += this->perimeter_points.row(i);
    }
    centroid /= this->perimeter_points.rows();
    return centroid;
}

// Getter for obstacle convex hull centroid
Vector2d Obstacle::get_convex_hull_centroid() const {
    // Compute the centroid of the convex hull
    Vector2d centroid = Vector2d::Zero();
    // Check if the convex_hull matrix is empty
    if (this->convex_hull.rows() == 0) {
        throw std::runtime_error("Empty convex hull matrix at Obstacle::get_convex_hull_centroid");
    }
    for (int i = 0; i < this->convex_hull.rows(); i++) {
        centroid += this->convex_hull.row(i);
    }
    centroid /= this->convex_hull.rows();
    return centroid;
}

// Check if sequence of points is valid: if it does not intersect itself,
//  and has at least 3 points
bool Obstacle::is_valid_sequence() const {
    // Check if the number of points is less than 3
    if (this->perimeter_points.rows() < 3) {
        cout << "Spill invalid - Less than 3 points in file" << endl;
        cout.flush();
        return false;
    }

    // Check for duplicate points
    for (int i = 0; i < this->perimeter_points.rows(); i++) {
        for (int j = i + 1; j < this->perimeter_points.rows(); j++) {
            if ((this->perimeter_points.row(i) - this->perimeter_points.row(j)).norm() < 1e-6) {
                cout << "Spill invalid - Duplicate points found in file" << endl;
                cout.flush();
                return false;
            }
        }
    }

    if (this->perimeter_points.rows() == 3) {
        return true;
    }
    // Check if the sequence of points intersects itself
    for (int i = 0; i < this->perimeter_points.rows() - 2; i++) {
        Vector2d p1 = this->perimeter_points.row(i);
        Vector2d p2 = this->perimeter_points.row((i + 1) % this->perimeter_points.rows());
        for (int j = i + 1; j < this->perimeter_points.rows(); j++) {
            Vector2d p3 = this->perimeter_points.row(j);
            Vector2d p4 = this->perimeter_points.row((j + 1) % this->perimeter_points.rows());
            if (check_intersection(p1.x(), p1.y(), p2.x(), p2.y(), p3.x(), p3.y(), p4.x(), p4.y(), 1e-6)) {
                cout << "Spill invalid - Line segments intersect." << endl;
                cout.flush();
                return false;
            }
        }
    }
    return true;
}

// Print obstacle convex hull to a file
void Obstacle::print_convex_hull_to_file(string foldername, string filename) const {
    // Check if convex hull matrix is empty
    if (this->convex_hull.rows() == 0) {
        throw std::runtime_error("Empty convex hull matrix at Obstacle::print_convex_hull_to_file");
    }
    
    // Create the file or reset it
    ofstream file(foldername + "/" + filename + ".txt");
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file at Obstacle::print_convex_hull_to_file");
    }
    // First row is the obstacle crucial quality flag 0/1
    file << this->crucial << endl;
    // second row is the weight of the obstacle
    file << this->weight << endl;
    
    // third row is the number of points in the convex hull
    file << this->convex_hull.rows() << endl;
    // Write the points
    for (int i = 0; i < this->convex_hull.rows(); i++) {
        file << this->convex_hull(i, 0) << " " << this->convex_hull(i, 1) << endl;
    }
    file.close();
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
