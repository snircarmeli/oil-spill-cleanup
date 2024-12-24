#ifndef BOOM_BOAT_H
#define BOOM_BOAT_H

#include "generic-boat.h"
#include <iostream>

class BoomBoat : public GenericBoat {
private:
    // Member variables
    float fuel; // Fuel [kg]
    float tank_curr; // Waste oil in tank [L]
    const float cap; // Waste oil capacity [L]
    // const float F_max; // Maximum force applied by engine [N]
    // const float eta_max; // Maximum steering angle [rad]

public:
    // Constructors
    BoomBoat(); // Default constructor
    BoomBoat(const BoomBoat &boom_boat); // Copy constructor
    BoomBoat(float radius, float mass, float inertia, float mu_l, float mu_ct, 
             float mu_r, float fuel, float cap, float F_max, float eta_max); // Parameterized constructor

    // Destructor
    ~BoomBoat();

    // Getter and Setter for fuel
    float get_fuel() const;
    float get_cap() const;
    float get_tank_curr() const;
    
    void set_fuel(float fuel);

    // Utility function to display BoomBoat status
    void print_status();
};

#endif
