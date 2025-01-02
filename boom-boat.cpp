#include "boom-boat.h"
#include <iostream>
#include <Eigen/Dense>

const float DEFAULT_CAPACITY = 100;
const float DEFAULT_INITIAL_FUEL = DEFAULT_CAPACITY;


// Default constructor
BoomBoat::BoomBoat() : GenericBoat(), fuel(0.0), tank_curr(0.0), cap(0.0) {
    this->pos << 0, 0, 0;
    this->vel << 0, 0, 0;
    this->set_control(Vector2f(0.0, 0.0));
}

// Constructor with position
BoomBoat::BoomBoat(Vector3f pos) : GenericBoat(), fuel(DEFAULT_INITIAL_FUEL),
 tank_curr(DEFAULT_INITIAL_FUEL), cap(DEFAULT_CAPACITY) {
    this->pos = pos;
    this->vel << 0, 0, 0;
    this->set_control(Vector2f(0.0, 0.0));
}

// Copy constructor
BoomBoat::BoomBoat(const BoomBoat &boom_boat) : GenericBoat(boom_boat),
 fuel(boom_boat.fuel), tank_curr(boom_boat.tank_curr), cap(boom_boat.cap) {
    this->set_control(Vector2f(0.0, 0.0));
 }


// Parameterized constructor
BoomBoat::BoomBoat(float radius, float mass, float inertia, float mu_l,
 float mu_ct, float mu_r, Vector3f pos, Vector3f vel, float fuel, float cap,
  float F_max, float eta_max) : GenericBoat(radius, mass, inertia, mu_l, mu_ct,
   mu_r, pos, vel, F_max, eta_max), fuel(fuel), tank_curr(fuel), cap(cap) {
    this->set_control(Vector2f(0.0, 0.0));
}



// Destructor
BoomBoat::~BoomBoat() {}

// Assignment operator
BoomBoat &BoomBoat::operator=(const BoomBoat &boom_boat) {
    if (this != &boom_boat) {
        GenericBoat::operator=(boom_boat);
        this->fuel = boom_boat.fuel;
        this->tank_curr = boom_boat.tank_curr;
        this->cap = boom_boat.cap;
        }
    return *this;
}

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
    std::cout << "Tank Current: " << this->tank_curr << " kg\n"
                << std::endl;
    std::cout << "Capacity: " << this->cap << " kg\n" << std::endl;

    // position and velocity
    std::cout << "Position: " << this->get_pos().transpose() << std::endl;
    std::cout << "Velocity: " << this->get_vel().transpose() << std::endl;
}
