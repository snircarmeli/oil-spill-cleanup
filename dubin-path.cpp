#include "dubin-path.h"
#include <cmath>

#define PI 3.141592653589793

// Load Dubin Path parameters
void DubinPath::load_dubin_path_params(string filename) {
    // Read parameters from JSON file
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open params file at DubinPath::load_dubin_path_params");
    }
    json params;
    file >> params;
    file.close();
    this->dubin_path_params = params["dubin_path"];
}

// Default Constructor
DubinPath::DubinPath() {
    qi = Vector2f::Zero();
    theta_i = 0;
    qf = Vector2f::Zero();
    theta_f = 0;
    min_turning_radius = 1;
    path = MatrixXf::Zero(0, 0);
    // Load default parameters
    this->load_dubin_path_params("params.json");
}

// Copy Constructor
DubinPath::DubinPath(const DubinPath &dubin_path) {
    qi = dubin_path.qi;
    theta_i = dubin_path.theta_i;
    qf = dubin_path.qf;
    theta_f = dubin_path.theta_f;
    min_turning_radius = dubin_path.min_turning_radius;
    path = dubin_path.path;
    dubin_path_params = dubin_path.dubin_path_params;
}

// Parameterized Constructor
DubinPath::DubinPath(Vector2f qi, float theta_i, Vector2f qf, float theta_f, float min_turning_radius) {
    this->qi = qi;
    this->theta_i = theta_i;
    this->qf = qf;
    this->theta_f = theta_f;
    this->min_turning_radius = min_turning_radius;
    this->path = MatrixXf::Zero(0, 0);
    // Load default parameters
    this->load_dubin_path_params("params.json");
}

// Destructor
DubinPath::~DubinPath() {}

// Assignment Operator
DubinPath& DubinPath::operator=(const DubinPath &other) {
    if (this != &other) {
        qi = other.qi;
        theta_i = other.theta_i;
        qf = other.qf;
        theta_f = other.theta_f;
        min_turning_radius = other.min_turning_radius;
        path = other.path;
        dubin_path_params = other.dubin_path_params;
    }
    return *this;
}

// Accessors
Vector2f DubinPath::get_qi() const {
    return qi;
}

float DubinPath::get_theta_i() const {
    return theta_i;
}

Vector2f DubinPath::get_qf() const {
    return qf;
}

float DubinPath::get_theta_f() const {
    return theta_f;
}

float DubinPath::get_min_turning_radius() const {
    return min_turning_radius;
}

MatrixXf DubinPath::get_path() const {
    return path;
}

// Calculate different Dubin's paths
pair<MatrixXf, float> DubinPath::calculate_RSR() {
    // Right circles with radius min_turning_radius
    Vector2f circ_i = qi + min_turning_radius * Vector2f(cos(theta_i - PI / 2), sin(theta_i - PI / 2));
    Vector2f circ_f = qf + min_turning_radius * Vector2f(cos(theta_f - PI / 2), sin(theta_f - PI / 2));
    // Vector from initial to final position
    Vector2f dir = circ_f - circ_i;
    float theta_dir = atan2(dir(1), dir(0));

    // 4 points of possible tangent
    Vector2f p1 = circ_i + min_turning_radius * Vector2f(cos(theta_dir + PI / 2), sin(theta_dir + PI / 2));
    Vector2f p2 = circ_f + min_turning_radius * Vector2f(cos(theta_dir + PI / 2), sin(theta_dir + PI / 2));
    // Vector2f p3 = circ_i + min_turning_radius * Vector2f(cos(theta_dir - PI / 2), sin(theta_dir - PI / 2));
    // Vector2f p4 = circ_f + min_turning_radius * Vector2f(cos(theta_dir - PI / 2), sin(theta_dir - PI / 2));

    // Calculate the path length
    // First arc
    float theta_p1 = atan2(p1(1) - circ_i(1), p1(0) - circ_i(0));
    float theta_r1 = theta_i + PI / 2;
    float arc1 = abs(theta_p1 - theta_r1);
    float d1 = min_turning_radius * arc1;

    // Second arc
    float theta_p2 = atan2(p2(1) - circ_f(1), p2(0) - circ_f(0));
    float theta_r2 = theta_f + PI / 2;
    float arc2 = abs(theta_p2 - theta_r2);
    float d2 = min_turning_radius * arc2;

    // Straight line
    float d3 = dir.norm();

    float total_length = d1 + d2 + d3;

    // Calculate the path
    float res = dubin_path_params["resolution"];
    if (res < d1 || res < d2 || res < d3) {
        throw std::runtime_error("Resolution is too small for the given path at RSR");
    }

    // int num_points_1 = d1 / res;

}