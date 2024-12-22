#ifndef BOAT_H
#define BOAT_H

#include <array>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using std::vector;

using Eigen::Vector3f;
using Eigen::Vector2f;
using Matrix2x3f = Eigen::Matrix<float, 2, 3>;

using std::array;
#define N 6

class BoomBoat;
class ContainerBoat;

class GenericBoat {
    private:
        float radius; // Distance between applied force point and Center of Mass [m]
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
        Vector3f pos; // Position [m]
        Vector3f vel; // Velocity [m/s]

        // Control variables
        Vector2f u_max; // Max control values
        

    public:
        // Constructors
        GenericBoat(); // Basic setup constructor, set every value to 1
        GenericBoat(const GenericBoat &gen_boat); // Copy constructor
        GenericBoat(float radius, float mass, float inertia, float mu_l, 
        float mu_ct, float mu_r, Vector3f pos, Vector3f vel, Vector2f u_max);
        // Destructor
        ~GenericBoat();

        // RK-4 method for propogation
        void propogate(Vector2f control, float dt);

        // Info getting
        Vector3f get_pos();
        Vector3f get_vel();
        void print_params();
    
};
// Helper functions
float wrap_theta(float theta);
int sign(float x);
#endif