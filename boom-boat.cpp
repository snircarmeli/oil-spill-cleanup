#include "boom-boat.h"
#include <iostream>
#include <Eigen/Dense>

const float DEFAULT_CAPACITY = 100;
const float DEFAULT_INITIAL_FUEL = DEFAULT_CAPACITY;


// Default constructor
BoomBoat::BoomBoat() : GenericBoat(), fuel(0.0), tank_curr(0.0), cap(0.0) {
    this->pos << 0, 0, 0;
    this->vel << 0, 0, 0;
}

// Constructor with position
BoomBoat::BoomBoat(Vector3f pos) : GenericBoat(), fuel(DEFAULT_INITIAL_FUEL),
 tank_curr(DEFAULT_INITIAL_FUEL), cap(DEFAULT_CAPACITY) {
    this->pos = pos;
    this->vel << 0, 0, 0;
}

// Copy constructor
BoomBoat::BoomBoat(const BoomBoat &boom_boat) : GenericBoat(boom_boat),
 fuel(boom_boat.fuel), tank_curr(boom_boat.tank_curr), cap(boom_boat.cap) {}


// Parameterized constructor
BoomBoat::BoomBoat(float radius, float mass, float inertia, float mu_l,
 float mu_ct, float mu_r, Vector3f pos, Vector3f vel, float fuel, float cap,
  float F_max, float eta_max) : GenericBoat(radius, mass, inertia, mu_l, mu_ct,
   mu_r, pos, vel, F_max, eta_max), fuel(fuel), tank_curr(fuel), cap(cap) {}



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
void BoomBoat::print_status() const {
    std::cout << "Fuel: " << this->fuel << " kg\n" << std::endl;
}
