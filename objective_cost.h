#ifndef OBJECTIVE_COST_H
#define OBJECTIVE_COST_H

#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <limits>

#include "helper_funcs.h"
#include "boom-boats-duo.h"
#include "oil-spill.h"
#include "obstacle.h"

using std::min;


class ObjectiveCost {
    private:
    double ts; // Time step
    double cost; // Cost

    // Weights
    double w_time; // Weight for time
    double w_fuel; // Weight for fuel use
    double w_duo_duo_dist; // Weight for distance between duos
    double w_duo_obstacle_dist; // Weight for distance between duo and obstacle
    double w_duo_spill_dist; // Weight for distance between duo and oil spill

    // Minimal distances
    double d_min_duo_duo; // Minimal distance between duos
    double d_min_duo_obstacle; // Minimal distance between duo and obstacle
    double d_min_duo_spill; // Minimal distance between duo and oil spill

    double kappa; // Exponential cost parameter


public:
    // Constructor
    ObjectiveCost(std::string filename);

    // Destructor
    ~ObjectiveCost();

    // Function to calculate the cost for a single duo
    void update_cost(vector<BoomBoatsDuo> &duos,
                            vector<Obstacle> &obstacles,
                            vector<OilSpill> &oil_spills);

    // Function to get the cost
    double get_cost() const;
    // Function to reset the cost
    void reset_cost();
};

// Function which returns an exponential cost if a certain ditance is below a certain threshold
double phi(double d, double d_min, double kappa);


#endif // OBJECTIVE_COST_H