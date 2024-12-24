#include "boom-boat.h"
#include <iostream>
#include <Eigen/Dense>

// Default constructor
BoomBoat::BoomBoat() : GenericBoat(), fuel(0.0), cap(0.0) {
    this->pos << 0, 0, 0;
    this->vel << 0, 0, 0;
}

// Copy constructor
BoomBoat::BoomBoat(const BoomBoat &boom_boat) : GenericBoat(boom_boat),
 fuel(boom_boat.fuel), cap(boom_boat.cap) {}

// Parameterized constructor
BoomBoat::BoomBoat(float radius, float mass, float inertia, float mu_l, float mu_ct, 
                   float mu_r, float fuel, float cap, float F_max, float eta_max) 
    : GenericBoat(radius, mass, inertia, mu_l, mu_ct, mu_r, Vector3f(0, 0, 0),
     Vector3f(0, 0, 0), F_max, eta_max), cap(cap), fuel(fuel), tank_curr(0.0) {}

// Destructor
BoomBoat::~BoomBoat() {}

// Getter for fuel
float BoomBoat::get_fuel() const { return fuel; }

// Getter for cap
float BoomBoat::get_cap() const { return cap; }


// Getter for tank_curr
float BoomBoat::get_tank_curr() const { return tank_curr; }

// Setter for fuel
void BoomBoat::set_fuel(float fuel) {
    if (fuel < 0) {
        std::cerr << "Fuel cannot be negative." << std::endl;
        return;
    }
    this->fuel = fuel;
}

// Example specialized function
void BoomBoat::print_status() {
    std::cout << "Fuel: " << this->fuel << " kg\n" << std::endl;
}
