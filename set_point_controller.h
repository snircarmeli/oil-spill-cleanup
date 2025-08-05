#ifndef SET_POINT_CONTROLLER_H
#define SET_POINT_CONTROLLER_H

#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem> // For file operations
#include <cmath>
#include "boom-boats-duo.h"
#include "helper_funcs.h"

// For JSON parameters parsing
#include "json/json.hpp"
using json = nlohmann::json;

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix;

using Matrix2X2d = Matrix<double, 2, 2>;
using Matrix3X2d = Matrix<double, 3, 2>;
using Matrix3X2d = Matrix<double, 3, 2>;

using std::string;
using std::endl;
using std::cout;

class SetPointController {
    private:
        json setpoint_params;
        // Controller parameters for orientation
        double wc_theta; // Crossover frequency
        double Kp_theta; // Gain controller - set by wc_theta
        double alpha_theta; // Lead controller parameter
        // Anti windup parameters
        double Tt_theta; // Anti windup for theta controller, au and av
        double saturation_theta; // Saturation for theta controller, au and av
        
        // Controller parameters for position
        double wc_u; // Crossover frequency
        double Kp_u; // Gain controller - set by wc_u
        double alpha_u; // Lead controller parameter
        // Anti windup parameters
        double Tt_u; // Anti windup for u controller, au and av
        double saturation_u; // Saturation for u controller, au and av

        // Integrators
        Matrix2X2d integ_lag;
        Matrix2X2d integ_lead;

        // Last output values
        Matrix2X2d last_output_lag;
        Matrix2X2d last_output_lead;

        int enable_lag; // Flag to indicate if lag controller is used
        int enable_windup_protection; // Flag to indicate if windup protection is used

        // Tolerance parameter for achieving the setpoint
        double tolerance_d; // Tolerance for distance from setpoint

        // Path parameters
        MatrixXd path_1;
        MatrixXd path_2;

        // Path division parameters
        int skip_index; // Index of the path to skip
        int current_index; // Current index of the path

        // Current setpoints
        Vector3d setpoint1; // Current setpoint for boat 1
        Vector3d setpoint2; // Current setpoint for boat 2

        double ts; // Time step

        // Error values for each ship
        Vector2d e_theta;
        Vector2d e_u;

        Vector2d ref_ang;

        // Max error in theta that above it, the u-controller is not used
        double max_error_theta;

        // Boolean to mark if the setpoint was updated
        bool setpoint_updated;

        int debug_a;
        int debug_v;

        // Flags to indicate if the new setpoints (position only) are the same as the previous ones
        bool setpoint1_same;
        bool setpoint2_same;

        // Parameters for unwrapping angles
        Vector2d last_ref_ang;
        Vector2d last_e_theta;
        bool first_update_ref_ang; // Flag to indicate if this is the first update of ref_ang
        bool first_update_e_theta; // Flag to indicate if this is the first update of e

        Vector2d u_ref; // Reference control signals for the boats
        vector<bool> waiting_for_other;
        bool EOP; // End of path flag

    public:
        // Constructor of parameters from JSON file
        SetPointController(string path_to_json, MatrixXd path_1,
            MatrixXd path_2, BoomBoatsDuo duo); 

        // Destructor
        ~SetPointController();

        // Getters
        double get_wc_theta() const;
        double get_Kp_theta() const;
        double get_alpha_theta() const;
        double get_wc_u() const;
        double get_Kp_u() const;
        double get_alpha_u() const;
        double get_ts() const;
        double get_tolerance_d() const;
        double get_skip_index() const;
        bool is_setpoint_updated() const;

        Vector3d get_setpoint1() const;
        Vector3d get_setpoint2() const;

        Vector2d get_e_theta() const;
        Vector2d get_e_u() const;

        // Unwrap angles
        Vector2d unwrap_angles(const Vector2d& current_angles, Vector2d& last_angles, bool is_first);

        // Update the error values
        // Update errors and step
        void update_errors(BoomBoatsDuo duo);

        // Calculate the closest links forces in local frame
        Matrix2X2d calculate_closest_links_forces(BoomBoatsDuo duo);

        // Calculate du and dv for each boat (Cancelling drag and link forces)
        Matrix2X2d calculate_du_dv(BoomBoatsDuo duo);


        // Calculate au and av with lead lag controllers
        Matrix2X2d calculate_au_av(BoomBoatsDuo duo);

        // Reset controllers
        void reset_controllers();
        

        // After calculating au and av, calculate the control signals
        Matrix2X2d calculate_control_signals(Matrix2X2d a, Matrix2X2d d);

        // Master update function which checks if the setpoint is reached. 
        // If it did, the setpoint is updated to the next one.
        // Either way, the control signals are calculated and returned.
        Matrix2X2d update(BoomBoatsDuo duo);

        // Function which prints data to a file
        void print_to_file(const string &filename, const string &foldername,
            const BoomBoatsDuo &duo) const;

};

#endif // SET_POINT_CONTROLLER_H