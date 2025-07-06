#include "objective_cost.h"

// Constructor
ObjectiveCost::ObjectiveCost(std::string filename) {
    // Read parameters from JSON file
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open params file at ObjectiveCost::ObjectiveCost");
    }
    json params;
    file >> params;
    file.close();

    json objective_params = params["ObjectiveCost"];

    this->ts = params["simulation"]["time_step"];
    
    // Initialize weights
    this->w_time = objective_params["w_time"];
    this->w_fuel = objective_params["w_fuel"];
    this->w_duo_duo_dist = objective_params["w_duo_duo_dist"];
    this->w_duo_obstacle_dist = objective_params["w_duo_obstacle_dist"];
    this->w_duo_spill_dist = objective_params["w_duo_spill_dist"];

    // Initialize minimal distances
    this->d_min_duo_duo = objective_params["d_min_duo_duo"];
    this->d_min_duo_obstacle = objective_params["d_min_duo_obstacle"];
    this->d_min_duo_spill = objective_params["d_min_duo_spill"];

    // Initialize kappa
    this->kappa = objective_params["kappa"];

    // Zero cost
    this->cost = 0;
}

// Destructor
ObjectiveCost::~ObjectiveCost() {}

// Function to calculate the cost for a single duo
void ObjectiveCost::update_cost(vector<BoomBoatsDuo> &duos,
    vector<Obstacle> &obstacles, vector<OilSpill> &oil_spills) {

    // Calculate the cost of distance between duos
    // cout << "[DEBUG] Start duo-duo distance cost calculation" << endl;
    // cout.flush();
    BoomBoat boat1;
    BoomBoat boat2;
    BoomBoat boat3;
    BoomBoat boat4;
    double min_dist = numeric_limits<double>::infinity();
    for (size_t i = 0; i < duos.size(); i++) {
        for (size_t j = i + 1; j < duos.size(); j++) {
            // cout << "[DEBUG] Checking duos " << i << " and " << j << endl;
            // cout.flush();
            boat1 = duos[i].get_boat1();
            boat2 = duos[i].get_boat2();
            boat3 = duos[j].get_boat1();
            boat4 = duos[j].get_boat2();
            double dist_13 = (boat1.get_pos().head(2) - boat3.get_pos().head(2)).norm();
            double dist_14 = (boat1.get_pos().head(2) - boat4.get_pos().head(2)).norm();
            double dist_23 = (boat2.get_pos().head(2) - boat3.get_pos().head(2)).norm();
            double dist_24 = (boat2.get_pos().head(2) - boat4.get_pos().head(2)).norm();
            min_dist = min(min_dist, min(dist_13, dist_24));
            min_dist = min(min_dist, min(dist_14, dist_23));
            // cout << "[DEBUG] min_dist after duo-duo check: " << min_dist << endl;
            // cout.flush();
            this->cost += this->w_duo_duo_dist * phi(min_dist, this->d_min_duo_duo, this->kappa) * this->ts;
            // cout << "[DEBUG] cost after duo-duo check: " << this->cost << endl;
            // cout.flush();
        }
    }
    // cout << "[DEBUG] End duo-duo distance cost calculation" << endl;
    // cout.flush();

    // Calculate the cost of distance between duos and obstacles
    // cout << "[DEBUG] Start duo-obstacle distance cost calculation" << endl;
    // cout.flush();
    MatrixXd hull;
    double dist;
    BoomBoatsDuo duo;
    Obstacle obstacle;
    Vector2d point1;
    Vector2d point2;
    for (size_t i = 0; i < duos.size(); i++) {
        for (size_t j = 0; j < obstacles.size(); j++) {
            // cout << "[DEBUG] Checking duo " << i << " and obstacle " << j << endl;
            // cout.flush();
            duo = duos[i];
            obstacle = obstacles[j];
            hull = obstacle.get_convex_hull();
            point1 = duo.get_boat1().get_pos().head(2);
            point2 = duo.get_boat2().get_pos().head(2);
            dist = calculate_point_convex_hull_distance(hull, point1);
            dist = min(dist, calculate_point_convex_hull_distance(hull, point2));
            // cout << "[DEBUG] min_dist after duo-obstacle check: " << dist << endl;
            // cout.flush();
            this->cost += this->w_duo_obstacle_dist * phi(dist, this->d_min_duo_obstacle, this->kappa) * this->ts;
            // cout << "[DEBUG] cost after duo-obstacle check: " << this->cost << endl;
            // cout.flush();
        }
    }
    // cout << "[DEBUG] End duo-obstacle distance cost calculation" << endl;
    // cout.flush();

    // Calculate the cost of distance between duos and oil spills
    // cout << "[DEBUG] Start duo-spill distance cost calculation" << endl;
    // cout.flush();
    OilSpill spill;
    for (size_t i = 0; i < duos.size(); i++) {
        for (size_t j = 0; j < oil_spills.size(); j++) {
            // cout << "[DEBUG] Checking duo " << i << " and spill " << j << endl;
            // cout.flush();
            duo = duos[i];
            spill = oil_spills[j];
            hull = spill.get_convex_hull();
            point1 = duo.get_boat1().get_pos().head(2);
            point2 = duo.get_boat2().get_pos().head(2);
            dist = calculate_point_convex_hull_distance(hull, point1);
            dist = min(dist, calculate_point_convex_hull_distance(hull, point2));
            // cout << "[DEBUG] min_dist after duo-spill check: " << dist << endl;
            // cout.flush();
            this->cost += this->w_duo_spill_dist * phi(dist, this->d_min_duo_spill, this->kappa) * this->ts;
            // cout << "[DEBUG] cost after duo-spill check: " << this->cost << endl;
            // cout.flush();
        }
    }
    // cout << "[DEBUG] End duo-spill distance cost calculation" << endl;
    // cout.flush();

    // Calculate the cost of time
    // cout << "[DEBUG] Start time cost calculation" << endl;
    // cout.flush();
    for (size_t i = 0; i < duos.size(); i++) {
        // cout << "[DEBUG] Checking time for duo " << i << endl;
        // cout.flush();
        this->cost += this->w_time * this->ts;
        // cout << "[DEBUG] cost after time check: " << this->cost << endl;
        // cout.flush();
    }
    // cout << "[DEBUG] End time cost calculation" << endl;
    // cout.flush();

    // Calculate the cost of fuel
    // cout << "[DEBUG] Start fuel cost calculation" << endl;
    // cout.flush();
    for (size_t i = 0; i < duos.size(); i++) {
        // cout << "[DEBUG] Checking fuel for duo " << i << endl;
        // cout.flush();
        this->cost += this->w_fuel * abs(duos[i].get_boat1().get_control()[0]) * this->ts;
        // cout << "[DEBUG] cost after boat1 fuel check: " << this->cost << endl;
        // cout.flush();
        this->cost += this->w_fuel * abs(duos[i].get_boat2().get_control()[0]) * this->ts;
        // cout << "[DEBUG] cost after boat2 fuel check: " << this->cost << endl;
        // cout.flush();
    }
    // cout << "[DEBUG] End fuel cost calculation" << endl;
    // cout.flush();
}

// Function to get the cost
double ObjectiveCost::get_cost() const {
    return this->cost;
}

// Function to reset the cost
void ObjectiveCost::reset_cost() {
    this->cost = 0;
}

// Function which returns an exponential cost if a certain distance is below a certain threshold
double phi(double d, double d_min, double kappa) {
    if (d < d_min) {
        return exp(-kappa * (d - d_min));
    } else {
        return 0;
    }
}