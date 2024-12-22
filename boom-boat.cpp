#include "boom-boat.h"
#include <iostream>
#include <Eigen/Dense>

// Default constructor
BoomBoat::BoomBoat() : GenericBoat(), fuel(0.0) {
    this->pos << 0, 0, 0;
    this->vel << 0, 0, 0;
}

// Copy constructor
BoomBoat::BoomBoat(const BoomBoat &boom_boat) : GenericBoat(boom_boat), fuel(boom_boat.fuel) {}

// Parameterized constructor
BoomBoat::BoomBoat(float radius, float mass, float inertia, float mu_l, float mu_ct, 
                   float mu_r, float boom_L, float fuel) 
    : GenericBoat(radius, mass, inertia, mu_l, mu_ct, mu_r, Vector3f(0, 0, 0), Vector3f(0, 0, 0), Vector2f(boom_L, 0)), 
      fuel(fuel) {}

// Destructor
BoomBoat::~BoomBoat() {}

// Getter for fuel
float BoomBoat::get_fuel() {
    return fuel;
}

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
