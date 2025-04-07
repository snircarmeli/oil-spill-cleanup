#ifndef BOOM_BOAT_H
#define BOOM_BOAT_H

#include "generic-boat.h"
#include <iostream>

// Parsing json parameters
#include "json/json.hpp"
using json = nlohmann::json;

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class BoomBoat : public GenericBoat {
private:
    // Member variables
    double fuel; // Fuel [kg]
    double tank_curr; // Waste oil in tank [L]
    double cap; // Waste oil capacity [L]
    // const double F_max; // Maximum force applied by engine [N]
    // const double eta_max; // Maximum steering angle [rad]

    json boom_boat_params;

public:
    // Constructors
    BoomBoat(); // Default constructor
    BoomBoat(const BoomBoat &boom_boat); // Copy constructor
    BoomBoat(Vector3d pos);
    BoomBoat(double radius, double mass, double inertia, double mu_l, double mu_ct, 
             double mu_r, Vector3d pos, Vector3d vel, double fuel, double cap,
              double F_max, double eta_max); // Parameterized constructor

    // Destructor
    ~BoomBoat();

    // Assignment operator
    BoomBoat &operator=(const BoomBoat &boom_boat);

    // Getter and Setter for fuel
    double get_fuel() const;
    double get_cap() const;
    double get_tank_curr() const;
    
    void set_fuel(double fuel);

    void load_boom_boat_params(std::string filename);

    // bool is_valid_control(Vector2d control) const;
    
    // Utility function to display BoomBoat status
    void print_status() const;

    // state derivative function
    VectorXd state_der(VectorXd state, Vector2d control,
     Vector2d boom_force) const;
};

#endif // BOOM_BOAT_H
