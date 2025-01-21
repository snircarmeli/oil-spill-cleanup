#ifndef DUBIN_PATH
#define DUBIN_PATH

#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem> // For filesystem operations (C++17 and later)

using Eigen::VectorXf;
using Eigen::Vector2f;
using Eigen::MatrixXf;
using Eigen::Matrix2f;
using std::cout;
using std::endl;
using std::string;
using std::pair;

// For JSON parameters parsing
#include "json/json.hpp"
using json = nlohmann::json;

class DubinPath {
    private:
        Vector2f qi;
        float theta_i;
        Vector2f qf;
        float theta_f;
        float min_turning_radius;

        MatrixXf path;
        json dubin_path_params;

    public:

        // Load Dubin Path parameters
        void load_dubin_path_params(string filename);

        // Constructors
        DubinPath(); // Basic setup constructor, set every value to 0, set min_turning_radius to 1, and path to 0x0
        DubinPath(const DubinPath &dubin_path); // Copy constructor
        DubinPath(Vector2f qi, float theta_i, Vector2f qf, float theta_f,
         float min_turning_radius); // Parameterized constructor
        
        // Destructor
        ~DubinPath();

        // Assignment Operator
        DubinPath& operator=(const DubinPath &other);

        // Accessors
        Vector2f get_qi() const;
        float get_theta_i() const;
        Vector2f get_qf() const;
        float get_theta_f() const;
        float get_min_turning_radius() const;
        MatrixXf get_path() const;

        // Calculate different Dubin's paths
        pair<MatrixXf, float> calculate_RSR();
        pair<MatrixXf, float> calculate_LSL();
        pair<MatrixXf, float> calculate_RSL();
        pair<MatrixXf, float> calculate_LSR();
        pair<MatrixXf, float> calculate_RLR();
        pair<MatrixXf, float> calculate_LRL();

        // Calculate the shortest Dubin's path
        MatrixXf calculate_shortest_path();


        // Print parameters
        void print_params();
        
        
};

#endif // DUBIN_PATH
