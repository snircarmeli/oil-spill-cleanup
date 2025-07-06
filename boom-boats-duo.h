#ifndef BOOM_BOATS_DUO_H
#define BOOM_BOATS_DUO_H

#include "boom-boat.h"
#include "helper_funcs.h"
#include <Eigen/Dense>

// For JSON parameters parsing
#include "json/json.hpp"
using json = nlohmann::json;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using std::string;

class Boom {
private:
    MatrixXd links_states; // Matrix representing all links and their states: N rows x 6 columns (length, x, y, theta, x_dot, y_dot, omega)
    double L; // Default length of each link
    double mu_l; // Linear drag coefficient
    double mu_ct; // Cross-track drag coefficient
    double mu_r; // Rotational drag coefficient
    double I; // Moment of inertia of each link
    double m; // Mass of each link

    // Parameters for the spring-damper model between links
    double k; // Spring constant
    double c; // Damping coefficient



    json boom_params;


public:
    // Constructor
    Boom(size_t num_links, double L, double mu_l, double mu_ct, double mu_r,
         double I, double m, double k, double c);
    Boom(size_t num_links, double L);
    // Default constructor
    Boom();
    
    // Destructor
    ~Boom();

    // Assignment operator
    Boom &operator=(const Boom &boom);

    // Accessors
    void set_link_state(size_t index, const VectorXd &state); // sets the state of the link at the given index
    VectorXd get_link_state(size_t index) const; // returns the state of the link at the given index
    void set_links_states(MatrixXd &states); // set the state of all links
    double get_L() const;
    double get_I() const;
    double get_m() const;
    double get_mu_l() const;
    double get_mu_ct() const;
    double get_mu_r() const;
    double get_k() const;
    double get_c() const;   

    void load_boom_params(std::string filename);

    // Utility
    int get_num_links() const;
    void print_links_states() const;
    void print_link_state(size_t index) const;

    // Validation of state
    bool is_valid_state() const; // Check if boom doesn't intersect itself
    // State derivative function
    MatrixXd state_der(const MatrixXd &state, const Vector2d Boom_force1,
     const Vector2d Boom_force2) const;

};

class BoomBoatsDuo {
private:
    BoomBoat boat1;
    BoomBoat boat2;
    Boom boom;
    double t; // Time [s]

    // Volume of the holding tank and available volume
    double V_max; // Maximum volume of the holding tank
    double V_available; // Available volume of the holding tank

    // Cleaning rate
    double cleaning_rate; // Rate of cleaning [m^3/s]
    
    json boom_boats_duo_params;
    json simulation_params;

public:
    // Constructor
    BoomBoatsDuo(const BoomBoat &b1, const BoomBoat &b2, size_t num_links,
     double L, double mu_l, double mu_ct, double mu_r, double I, double m,
      double k, double c, Vector2d center, double orientation, double V_max,
       double V_available, double cleaning_rate);
    BoomBoatsDuo(Vector2d center, double orientation, size_t num_links, double L);
    // Default constructor
    BoomBoatsDuo();

    // Destructor
    ~BoomBoatsDuo();

    // Assignment operator
    BoomBoatsDuo &operator=(const BoomBoatsDuo &boom_boats_duo);
    
    // Utility functions
    void print_status() const;
    void print_to_file(const string &filename, const string &foldername) const;

    void load_boom_boats_duo_params(std::string filename);

    // Validation of state
    bool is_valid_state() const; // Check if boom doesn't intersect itself and
    bool are_boats_close() const ;// if the boats are not colliding

    // Propagation function
    MatrixXd state_der(const Vector2d &control1, const Vector2d &control2,
     MatrixXd state) const;
    void propagate(double dt, const Vector2d &control1,
     const Vector2d &control2, std::string integration_method); // Temporary fix - without control for now

    // Getters
    BoomBoat get_boat1() const;
    BoomBoat get_boat2() const;
    Vector2d get_center() const;
    Boom get_boom() const;
    json get_simulation_params() const;
    double get_time() const;
    double get_V_max() const;
    double get_V_available() const;
    double get_cleaning_rate() const;

    // Setters
    bool decrease_V_available(double delta); // Returns true if successful
    void set_cleaning_rate(double rate); // Set the cleaning rate
};


#endif
