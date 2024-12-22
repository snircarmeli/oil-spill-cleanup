#ifndef BOOM_BOAT_H
#define BOOM_BOAT_H

#include "generic-boat.h"
#include <iostream>

class BoomBoat : public GenericBoat {
private:
    // Member variables
    float fuel; // Fuel [kg]

public:
    // Constructors
    BoomBoat(); // Default constructor
    BoomBoat(const BoomBoat &boom_boat); // Copy constructor
    BoomBoat(float radius, float mass, float inertia, float mu_l, float mu_ct, 
             float mu_r, float boom_L, float fuel); // Parameterized constructor

    // Destructor
    ~BoomBoat();

    // Getter and Setter for fuel
    float get_fuel();
    void set_fuel(float fuel);

    // Utility function to display BoomBoat status
    void print_status();
};

#endif
