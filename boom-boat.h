#ifndef BOOM_BOAT_H
#define BOOM_BOAT_H

#include "generic-boat.h"
#include <iostream>

// Parsing json parameters
#include "json/json.hpp"
using json = nlohmann::json;

using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::VectorXf;
using Eigen::MatrixXf;

class BoomBoat : public GenericBoat {
private:
    // Member variables
    float fuel; // Fuel [kg]
    float tank_curr; // Waste oil in tank [L]
    float cap; // Waste oil capacity [L]
    // const float F_max; // Maximum force applied by engine [N]
    // const float eta_max; // Maximum steering angle [rad]

    json boom_boat_params;

public:
    // Constructors
    BoomBoat(); // Default constructor
    BoomBoat(const BoomBoat &boom_boat); // Copy constructor
    BoomBoat(Vector3f pos);
    BoomBoat(float radius, float mass, float inertia, float mu_l, float mu_ct, 
             float mu_r, Vector3f pos, Vector3f vel, float fuel, float cap,
              float F_max, float eta_max); // Parameterized constructor

    // Destructor
    ~BoomBoat();

    // Assignment operator
    BoomBoat &operator=(const BoomBoat &boom_boat);

    // Getter and Setter for fuel
    float get_fuel() const;
    float get_cap() const;
    float get_tank_curr() const;
    
    void set_fuel(float fuel);

    void load_boom_boat_params(std::string filename);

    // bool is_valid_control(Vector2f control) const;
    
    // Utility function to display BoomBoat status
    void print_status() const;

    // state derivative function
    VectorXf state_der(VectorXf state, Vector2f control,
     Vector2f boom_force) const;
};

#endif // BOOM_BOAT_H
