#ifndef BOAT_H
#define BOAT_H

#include <vector>
#include <Eigen/Dense>

#include <fstream> // For file I/O
// Parsing json parameters
#include "json/json.hpp"
using json = nlohmann::json;

using namespace std;
using std::vector;

using Eigen::Vector3d;
using Eigen::Vector2d;
using Matrix2x3d = Eigen::Matrix<double, 2, 3>;


class BoomBoat;
class ContainerBoat;

class GenericBoat {
    private:
        double radius; // Distance between applied force point and Center of mass [m]
        double mass; // Mass of Boat [kg]
        double inertia; // Moment of inertia [kg*m^2]
        // Drag coefficient of movement
        double mu_l; // Drag of advancing [kg / m]
        double mu_ct; // Drag of drifting sideways (Cross Track) [kg / m] 
        double mu_r; // drag while rotating [kg * m^2]
        
        // Max control values
        double F_max; // Maximum force applied by engine [N]
        double eta_max; // Maximum steering angle [rad]

        

    protected:
        // State variables
        Vector3d pos; // Position [m], [m], [rad]
        Vector3d vel; // Velocity [m/s], [m/s], [rad/s]

        // Current control values
        double F; // Force applied by engine [N]
        double eta; // Steering angle [rad]

        json generic_boat_params;
        

    public:
        // Constructors
        GenericBoat(); // Basic setup constructor, set every value to 1
        GenericBoat(const GenericBoat &gen_boat); // Copy constructor
        GenericBoat(double radius, double mass, double inertia, double mu_l, 
        double mu_ct, double mu_r, Vector3d pos, Vector3d vel, double F_max,
         double eta_max); // Parameterized constructor
        // Destructor
        ~GenericBoat();

        // Assignment operator
        GenericBoat& operator=(const GenericBoat &other);

        // Accessors
        Vector3d get_pos() const;
        Vector3d get_vel() const;
        Vector3d get_frame_vel() const; // Velocities in the body frame
        Vector2d get_control() const;
        double get_radius() const;
        double get_mass() const;
        double get_inertia() const;
        double get_mu_l() const;
        double get_mu_ct() const;
        double get_mu_r() const;
        double get_F_max() const;
        double get_eta_max() const;
        double get_ship_size() const;
        void print_params();

        // Setters
        void set_pos(Vector3d pos);
        void set_vel(Vector3d vel);
        void set_control(Vector2d control);

        bool is_valid_control(Vector2d control) const;

        void load_params(std::string filename);
    
};

#endif