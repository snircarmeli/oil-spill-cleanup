#ifndef CONTAINER_BOAT_H
#define CONTAINER_BOAT_H

#include "generic-boat.h"

class ContainerBoat : public GenericBoat {
    private:
    float total_cap; // Total volume of oil tamk [m^3]
    float avail_cap; // Available volume of oil tamk [m^3]

    public:
    // Constructors
    ContainerBoat(); // Basic setup constructor
    ContainerBoat(const ContainerBoat &container_boat); // Copy constructor
    ContainerBoat(float radius, float mass, float inertia, float mu_l, float mu_ct, float mu_r,
    float total_cap, float avail_cap);
    // Destructor
    ~ContainerBoat();
};

#endif