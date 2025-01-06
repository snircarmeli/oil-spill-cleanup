#ifndef BOAT_H
#define BOAT_H

#include <vector>
#include <Eigen/Dense>

using namespace std;
using std::vector;

using Eigen::Vector3f;
using Eigen::Vector2f;
using Matrix2x3f = Eigen::Matrix<float, 2, 3>;


class BoomBoat;
class ContainerBoat;

class GenericBoat {
    private:
        float radius; // Distance between applied force point and Center of mass [m]
        float mass; // Mass of Boat [kg]
        float inertia; // Moment of inertia [kg*m^2]
        // Drag coefficient of movement
        float mu_l; // Drag of advancing [kg / m]
        float mu_ct; // Drag of drifting sideways (Cross Track) [kg / m] 
        float mu_r; // drag while rotating [kg * m^2]
        
        // Max control values
        float F_max; // Maximum force applied by engine [N]
        float eta_max; // Maximum steering angle [rad]


        // EOM for the propogate function (RK4)
        Matrix2x3f state_der(Matrix2x3f state, Vector2f control);

    protected:
        // State variables
        Vector3f pos; // Position [m], [m], [rad]
        Vector3f vel; // Velocity [m/s], [m/s], [rad/s]

        // Current control values
        float F; // Force applied by engine [N]
        float eta; // Steering angle [rad]

        

    public:
        // Constructors
        GenericBoat(); // Basic setup constructor, set every value to 1
        GenericBoat(const GenericBoat &gen_boat); // Copy constructor
        GenericBoat(float radius, float mass, float inertia, float mu_l, 
        float mu_ct, float mu_r, Vector3f pos, Vector3f vel, float F_max,
         float eta_max); // Parameterized constructor
        // Destructor
        ~GenericBoat();

        // Assignment operator
        GenericBoat& operator=(const GenericBoat &other);

        // RK-4 method for propogation
        void propogate(Vector2f control, float dt);

        // Accessors
        Vector3f get_pos() const;
        Vector3f get_vel() const;
        Vector2f get_control() const;
        float get_radius() const;
        float get_mass() const;
        float get_inertia() const;
        float get_mu_l() const;
        float get_mu_ct() const;
        float get_mu_r() const;
        float get_F_max() const;
        float get_eta_max() const;
        void print_params();

        // Setters
        void set_pos(Vector3f pos);
        void set_vel(Vector3f vel);
        void set_control(Vector2f control);
    
};
// Helper functions
float wrap_theta(float theta);
int sign(float x);
#endif