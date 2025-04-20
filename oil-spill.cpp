#include "oil-spill.h"

// Default constructor
OilSpill::OilSpill() : oil_spill_mass(0.0) {
    // Set perimeter_points to an empty matrix
    this->perimeter_points = MatrixXd(0, 2);
    // Set convex_hull to an empty matrix
    this->convex_hull = MatrixXd(0, 2);
    this->load_oil_spill_params("params.json");
}

// Copy constructor
OilSpill::OilSpill(const OilSpill &oil_spill) {
    this->perimeter_points = oil_spill.perimeter_points;
    this->convex_hull = oil_spill.convex_hull;
    this->oil_spill_mass = oil_spill.oil_spill_mass;
    this->load_oil_spill_params("params.json");
}


// Parameterized constructor
OilSpill::OilSpill(MatrixXd perimeter_points, MatrixXd convex_hull,
 double oil_spill_mass) {
    this->perimeter_points = perimeter_points;
    this->convex_hull = convex_hull;
    this->oil_spill_mass = oil_spill_mass;
    this->load_oil_spill_params("params.json");
}

// Constructor with filename
OilSpill::OilSpill(string filename) {
    // Load parameters from txt file filename
    
    ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file at OilSpill::OilSpill");
    }
    // First line is the oil spill mass
    file >> this->oil_spill_mass;
    // Check if mass is non-negative
    if (this->oil_spill_mass < 0) {
        file.close();
        throw std::runtime_error("Negative oil spill mass at OilSpill::OilSpill");
    }
    // Second line is the number of points
    int num_points;
    file >> num_points;

    // Check if num_points is an integer
    if (file.fail()) {
        file.close();
        throw std::runtime_error("Incomplete number of points at OilSpill::OilSpill");
    }
    // Check if num_points is non-negative
    if (num_points < 0) {
        file.close();
        throw std::runtime_error("Negative number of points at OilSpill::OilSpill");
    }

    // Consume the remaining newline character
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    this->perimeter_points = MatrixXd(num_points, 2);

    // Read the points
    string line;
    int cnt = 0;
    string test;
    int cnt2 = 0;
    // Enter the next line of file into line
    getline(file, line);
    while (!line.empty()) {
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
                    throw std::runtime_error("Too many points in file " + filename + " at line " + std::to_string(cnt + 3) + " at OilSpill::OilSpill");
                }
            }
            else {
                throw std::runtime_error("Invalid data in file " + filename + " at line " + std::to_string(cnt + 3) + " at OilSpill::OilSpill");
            }
        }
        if (cnt2 < 2) {
            throw std::runtime_error("Invalid point in file " + filename + " at line " + std::to_string(cnt + 3) + " at OilSpill::OilSpill");
        }
        
        // If we reach here, iss has only two numbers
        double x, y;
        iss = istringstream(line);
        iss >> x >> y;
        this->perimeter_points.row(cnt) << x, y;
        cnt++;
        getline(file, line);
    }
    file.close();
    // Check if cnt is less than num_points
    if (cnt < num_points) {
        throw std::runtime_error("Greater number of points reported at file " + filename + " at OilSpill::OilSpill");
    }
    // Check if cnt is greater than num_points
    if (cnt > num_points) {
        throw std::runtime_error("Smaller number of points reported at file " + filename + " at OilSpill::OilSpill");
    }

    this->load_oil_spill_params("params.json");
}

// Destructor
OilSpill::~OilSpill() {}

// Assignment operator
OilSpill &OilSpill::operator=(const OilSpill &oil_spill) {
    if (this != &oil_spill) {
        this->oil_spill_mass = oil_spill.oil_spill_mass;
        this->perimeter_points = oil_spill.perimeter_points;
    }
    
    // Check if the perimeter_points is valid
    if (!this->is_valid_sequence()) {
        throw std::runtime_error("Invalid sequence of points at OilSpill::operator=");
    }
    this->load_oil_spill_params("params.json");
    return *this;
}

// load oil spill parameters from a file
void OilSpill::load_oil_spill_params(string filename) {
    // Load parameters from txt file filename
    ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file at OilSpill::load_oil_spill_params");
    }
    this->oil_spill_params = json();
    file >> this->oil_spill_params;
    this->oil_spill_params = this->oil_spill_params["oil_spill"];
    file.close();
}

// Getter for perimeter_points
MatrixXd OilSpill::get_perimeter_points() const {
    return this->perimeter_points;
}

// Setter for perimeter_points
void OilSpill::set_perimeter_points(MatrixXd perimeter_points) {
    this->perimeter_points = perimeter_points;
}

// Getter for convex_hull
MatrixXd OilSpill::get_convex_hull() const {
    // If convex_hull is empty, calculate it
    if (this->convex_hull.rows() == 0) {
        throw std::runtime_error("Empty convex hull matrix at OilSpill::get_convex_hull"); 
    }
    return this->convex_hull;
}

// Setter for convex_hull
void OilSpill::set_convex_hull(MatrixXd convex_hull) {
    this->convex_hull = convex_hull;
}

// Getter for oil_spill_mass
double OilSpill::get_oil_spill_mass() const {
    return this->oil_spill_mass;
}

// Setter for oil_spill_mass
void OilSpill::set_oil_spill_mass(double oil_spill_mass) {
    this->oil_spill_mass = oil_spill_mass;
}

// Get convex hull of the oil spill
void OilSpill::calculate_convex_hull() {

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

// Getter for the spill centroid
Vector2d OilSpill::get_spill_centroid() const {
    // Compute the centroid of the oil spill
    Vector2d centroid = Vector2d::Zero();
    // Check if the perimeter_points matrix is empty
    if (this->perimeter_points.rows() == 0) {
        throw std::runtime_error("Empty perimeter points matrix at OilSpill::get_spill_centroid");
    }
    for (int i = 0; i < this->perimeter_points.rows(); i++) {
        centroid += this->perimeter_points.row(i);
    }
    centroid /= this->perimeter_points.rows();
    return centroid;
}

// Get the convex hull radius
double OilSpill::get_convex_hull_radius() const {
    // Check if the convex_hull matrix is empty
    if (this->convex_hull.rows() == 0) {
        throw std::runtime_error("Empty convex hull matrix at OilSpill::get_convex_hull_radius");
    }
    // Compute the centroid of the convex hull
    Vector2d centroid = this->get_convex_hull_centroid();
    // Compute the maximum distance from the centroid
    double dist;
    double radius = 0.0;
    for (int i = 0; i < this->convex_hull.rows(); i++) {
        dist = (this->convex_hull.row(i).transpose() - centroid).norm();
        if (dist > radius) {
            radius = dist;
        }
    }
    return radius;
}

// Getter for the convex hull centroid
Vector2d OilSpill::get_convex_hull_centroid() const {
    // Compute the centroid of the convex hull
    Vector2d centroid = Vector2d::Zero();
    // Check if the convex_hull matrix is empty
    if (this->convex_hull.rows() == 0) {
        throw std::runtime_error("Empty convex hull matrix at OilSpill::get_convex_hull_centroid");
    }
    for (int i = 0; i < this->convex_hull.rows(); i++) {
        centroid += this->convex_hull.row(i);
    }
    centroid /= this->convex_hull.rows();
    return centroid;
}

// Expand the oil spill by a factor
void OilSpill::expand_spill(double factor) {
    // Check if the perimeter_points matrix is empty
    if (this->perimeter_points.rows() == 0) {
        throw std::runtime_error("Empty perimeter points matrix at OilSpill::expand_spill");
    }
    // Check if the factor is negative
    if (factor <= 0) {
        throw std::runtime_error("Factor is non-positive at OilSpill::expand_spill");
    }
    // Compute the centroid of the oil spill
    Vector2d centroid = this->get_spill_centroid();
    // Expand the oil spill by the factor
    for (int i = 0; i < this->perimeter_points.rows(); i++) {
        this->perimeter_points.row(i) = (centroid + factor * (this->perimeter_points.row(i).transpose() - centroid)).transpose();
    }

    // Check if the convex_hull matrix is empty
    if (this->convex_hull.rows() == 0) {
        return;
    }
    // Compute the centroid of the convex hull
    centroid = this->get_convex_hull_centroid();
    // Expand the convex hull by the factor
    for (int i = 0; i < this->convex_hull.rows(); i++) {
        this->convex_hull.row(i) = (centroid + factor * (this->convex_hull.row(i).transpose() - centroid)).transpose();
    }
}

// Check if sequence of points is valid: if it does not intersect itself,
//  and has at least 3 points
bool OilSpill::is_valid_sequence() const {
    // Check if the number of points is less than 3
    if (this->perimeter_points.rows() < 3) {
        cout << "Spill invalid - Less than 3 points in file" << endl;
        cout.flush();
        return false;
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

// Utility function to display OilSpill status
void OilSpill::print_status() const {
    cout << "Oil spill mass: " << this->oil_spill_mass << " kg" << endl;
    cout << "Perimeter points: " << endl;
    cout << this->perimeter_points << endl;
    cout << "Convex hull: " << endl;
    cout << this->get_convex_hull() << endl;
    cout << "Spill centroid: " << this->get_spill_centroid().transpose() << endl;
    cout << "Is valid sequence: " << this->is_valid_sequence() << endl;
}

// Print convex hull to a file
void OilSpill::print_convex_hull_to_file(string foldername, 
string filename) const {
    // Create the file or reset it
    ofstream file(foldername + "/" + filename + ".txt");
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file at OilSpill::print_convex_hull_to_file");
    }
    // First row is the oil spill mass
    file << this->oil_spill_mass << endl;
    // Second row is the number of points
    file << this->convex_hull.rows() << endl;
    // Write the points
    for (int i = 0; i < this->convex_hull.rows(); i++) {
        file << this->convex_hull(i, 0) << " " << this->convex_hull(i, 1) << endl;
    }
    file.close();
}

// Length of line that passes through the convex centroid and is perpendicular to the line
double OilSpill::convex_hull_line_length(double orientation) const
{
    // Check if the convex_hull matrix is empty
    if (this->convex_hull.rows() == 0) {
        throw std::runtime_error("Empty convex hull matrix at OilSpill::convex_hull_line_length");
    }

    // Rotate the convex hull by -orientation
    MatrixXd rotated_convex_hull = MatrixXd::Zero(this->convex_hull.rows(), 2);
    Matrix2d R;
    R = rot_mat(-orientation);
    rotated_convex_hull = (R * this->convex_hull.transpose()).transpose();

    // Find highest and lowest points
    double max_y = rotated_convex_hull.col(1).maxCoeff();
    double min_y = rotated_convex_hull.col(1).minCoeff();
    return max_y - min_y;
}

// Get the angle of attack of the oil spill
pair<double, double> OilSpill::angle_of_attack() const
{
    // Check if the convex_hull matrix is empty
    if (this->convex_hull.rows() == 0) {
        throw std::runtime_error("Empty convex hull matrix at OilSpill::angle_of_attack");
    }
    
    // Rotating calipers algorithm (Or something like it)
    MatrixXd Rotated_convex_hull = MatrixXd::Zero(this->convex_hull.rows(), 2);
    Matrix2d R; // Rotation matrix
    double min_length = std::numeric_limits<double>::max();
    double angle_of_attack = 0.0;
    // Iterate Each edge and find its orientation
    for (int i = 0; i < this->convex_hull.rows(); i++) {
        Vector2d p1 = this->convex_hull.row(i);
        Vector2d p2 = this->convex_hull.row((i + 1) % this->convex_hull.rows());
        Vector2d edge = p2 - p1;
        double orientation = atan2(edge.y(), edge.x());
        // Rotate the convex hull by -orientation
        R = rot_mat(PI - orientation);
        Rotated_convex_hull = (R * this->convex_hull.transpose()).transpose();
        // If rotated by (PI - orientation), the edge (p1, p2) is now horizontal, and
        // all other points are above it.
        Rotated_convex_hull.col(1) -= p1.y() * VectorXd::Ones(this->convex_hull.rows());
        if (Rotated_convex_hull.col(1).maxCoeff() < min_length) {
            min_length = Rotated_convex_hull.col(1).maxCoeff();
            angle_of_attack = orientation;
        }
    }
    return make_pair(angle_of_attack, min_length);
}
