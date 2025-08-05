#include "set_point_controller.h"

// Constructor
SetPointController::SetPointController(string path_to_json, MatrixXd path_1,
    MatrixXd path_2, BoomBoatsDuo duo) : path_1(path_1), path_2(path_2) {
    // Load parameters from JSON file
    ifstream file(path_to_json);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file at SetPointController::SetPointController");
    }
    json params;
    file >> params;
    file.close();
    
    json setpoint_params;
    setpoint_params = params["SetPointController"];
    this->setpoint_params = setpoint_params;

    // Set parameters from JSON
    this->wc_theta = setpoint_params["wc_theta"];

    double Ir = duo.get_boat1().get_inertia() / duo.get_boat1().get_radius();

    this->Kp_theta = Ir * this->wc_theta * this->wc_theta;

    double phase_margin_theta = setpoint_params["phase_margin_theta"];
    double phi_lead_theta = phase_margin_theta + 5.7; // 5.7 because of lag controller
    phi_lead_theta *= PI / 180; // Convert to radians
    this->alpha_theta = (1 + sin(phi_lead_theta)) / (1 - sin(phi_lead_theta));

    this->Tt_theta = setpoint_params["Tt_theta"];
    this->saturation_theta = setpoint_params["sat_theta"];

    this->wc_u = setpoint_params["wc_u"];

    double m = duo.get_boat1().get_mass();

    if (setpoint_params["speed_control"] == 0) {
        this->Kp_u = m * this->wc_u * this->wc_u;
    } else {
        this->Kp_u = m * this->wc_u;
    }

    double phase_margin_u = setpoint_params["phase_margin_u"];
    double phi_lead_u = phase_margin_u; // + 5.7; // 5.7 because of lag controller
    phi_lead_u *= PI / 180; // Convert to radians
    this->alpha_u = (1 + sin(phi_lead_u)) / (1 - sin(phi_lead_u));

    this->Tt_u = setpoint_params["Tt_u"];

    this->saturation_u = setpoint_params["sat_u"];

    // Initialize path division parameters
    this->skip_index = setpoint_params["skip_index"];

    this->tolerance_d = setpoint_params["tolerance_d"];

    this->ts = params["simulation"]["time_step"];

    this->max_error_theta = setpoint_params["max_error_theta_deg"].get<double>() * PI / 180; // Convert to radians

    this->debug_a = setpoint_params["debug_a"];
    this->debug_v = setpoint_params["debug_v"];

    // Initialize flags for setpoints
    this->setpoint1_same = false;
    this->setpoint2_same = false;

    this->enable_lag = setpoint_params["enable_lag"];
    this->enable_windup_protection = setpoint_params["enable_windup_protection"];
    
    // Initialize integrators and last values of outputs
    this->reset_controllers();
   
    // Initialize error values
    this->e_theta.setZero(2, 1);
    this->e_u.setZero(2, 1);

    // Initialize reference angle
    this->ref_ang.setZero(2, 1);

    

    // Initialize setpoints to the second point of the path (to avoid overboarding the rope)
    this->setpoint1 = path_1.col(this->skip_index);
    this->setpoint2 = path_2.col(this->skip_index);
    this->current_index = this->skip_index;

    this->setpoint_updated = true;

    // Initialize parameters for unwrapping angles
    this->last_ref_ang.setZero(2, 1);
    this->last_e_theta.setZero(2, 1); 
    this->first_update_ref_ang = true;
    this->first_update_e_theta = true;

    this->u_ref.setZero(2, 1);
    this->waiting_for_other.resize(2, false); // Initialize waiting flags for both boats
    this->EOP = false; // Initialize end of path flag


    // // Update the initial errors
    // this->update_errors(duo);
}

// Destructor
SetPointController::~SetPointController() {
    // Destructor implementation (if needed)
}

// Getters
double SetPointController::get_wc_theta() const {
    return this->wc_theta;
}
double SetPointController::get_Kp_theta() const {
    return this->Kp_theta;
}
double SetPointController::get_alpha_theta() const {
    return this->alpha_theta;
}
double SetPointController::get_wc_u() const {
    return this->wc_u;
}
double SetPointController::get_Kp_u() const {
    return this->Kp_u;
}
double SetPointController::get_alpha_u() const {
    return this->alpha_u;
}
double SetPointController::get_ts() const {
    return this->ts;
}
double SetPointController::get_tolerance_d() const {
    return this->tolerance_d;
}
double SetPointController::get_skip_index() const {
    return this->skip_index;
}
bool SetPointController::is_setpoint_updated() const {
    return this->setpoint_updated;
}
Vector3d SetPointController::get_setpoint1() const {
    return this->setpoint1;
}
Vector3d SetPointController::get_setpoint2() const {
    return this->setpoint2;
}
Vector2d SetPointController::get_e_theta() const {
    return this->e_theta;
}
Vector2d SetPointController::get_e_u() const {
    return this->e_u;
}


// Unwrap angles
Vector2d SetPointController::unwrap_angles(const Vector2d& current_angles, Vector2d& last_angles, bool is_first) {
    Vector2d unwrapped = current_angles;

    // Print current and last angles of the first boat
    // cout << "Current angle: " << current_angles(0) << endl;
    // cout << "Last angle: " << last_angles(0) << endl;
    
    cout << endl;

    if (!is_first) {
        for (int i = 0; i < 2; i++) {
            double diff = current_angles(i) - last_angles(i);
            // if (i == 0) {
            //     cout << "Diff is " << diff << " for angle " << i << endl;
            // }
            
            // Accumulate the unwrapping offset
            static Vector2d offset = Vector2d::Zero();
            
            // If jump is greater than π, we've wrapped around positively
            if (diff > PI) {
                offset(i) -= 2*PI;
                unwrapped(i) = current_angles(i) + offset(i);
                // if (i == 0) {
                //     cout << "Unwrapping angle " << i << ": detected +2π jump" << endl;
                // }
            }
            // If jump is less than -π, we've wrapped around negatively  
            else if (diff < -PI) {
                offset(i) += 2*PI;
                unwrapped(i) = current_angles(i) + offset(i);
                // if (i == 0) {
                //     cout << "Unwrapping angle " << i << ": detected -2π jump" << endl;
                // }
            } else {
                // No jump detected, keep the current angle
                // unwrapped(i) = current_angles(i);
                // if (i == 0) {
                //     cout << "Unwrapping angle " << i << ": no jump detected" << endl;
                // }
            }
        }
    }

    // cout << "Unwrapped angle for boat 1: " << unwrapped(0) << endl;
    // cout.flush();
    
    return unwrapped;
}


// Update the error values
void SetPointController::update_errors(BoomBoatsDuo duo) {
    // Update the error values based on the current state of the boats and the setpoints

    // // Update the error in u (distance to the setpoint)
    // Matrix3X2d setpoint;
    // setpoint.col(0) = this->setpoint1;
    // setpoint.col(1) = this->setpoint2;

    // cout << "Current setpoint for boat 1: " << this->setpoint1.transpose() << endl;
    // cout << "Current setpoint for boat 2: " << this->setpoint2.transpose() << endl;
    // cout.flush();

    Matrix3X2d position;
    position.col(0) = duo.get_boat1().get_pos();
    position.col(1) = duo.get_boat2().get_pos();

    Vector2d vel_u;
    vel_u(0) = duo.get_boat1().get_frame_vel()(0);
    vel_u(1) = duo.get_boat2().get_frame_vel()(0);
    double u_ref = this->setpoint_params["speed_reference"];
    
    // for (int i = 0; i < 2; i++) {
    //     e_u(i) = (setpoint.col(i).head(2) - position.col(i).head(2)).norm();
    // }

    // Update the error in theta (angle to the setpoint)
    // If the error jumped from -pi to pi (or vice versa), we need to reset integrators
    // to avoid a jump in the output
    // double tmp;
    for (int i = 0; i < 2; i++) {
        // ref_ang is updated in the update() function

        this->e_theta(i) = this->ref_ang(i) - position(2, i);
        // // Check if the error jumped from -pi to pi (or vice versa)
        // if (abs(this->e_theta(i) - tmp) > 0.1 * PI) {
        //     this->reset_controllers();
        //     cout << "Resetting controllers due to angle jump" << endl;
        // }
    }

    if (this->debug_a) {
        // For each boat, print the current position, setpoint and error
        Matrix2X2d pos;
        pos.col(0) = duo.get_boat1().get_pos().head(2);
        pos.col(1) = duo.get_boat2().get_pos().head(2);
        Matrix2X2d setpoint;
        setpoint.col(0) = this->setpoint1.head(2);
        setpoint.col(1) = this->setpoint2.head(2);
        for (int i = 0; i < 2; i++) {
            cout << "Boat " << i + 1 << " position: " << pos.col(i).transpose() << endl;
            cout << "Boat " << i + 1 << " setpoint: " << setpoint.col(i).transpose() << endl;
            cout << "Boat " << i + 1 << " error in location: " << e_u(i) << endl;
            cout << "Boat " << i + 1 << " error in theta: " << e_theta(i) << endl << endl;
            cout << "Boat " << i + 1 << " error in velocity: " << u_ref - vel_u(i) << endl;
        cout << "----------------------------------------" << endl;
        cout.flush();
        }
    }
}

Matrix2X2d SetPointController::calculate_closest_links_forces(BoomBoatsDuo duo) {
    // Take the force and angle of the closest link into account
    BoomBoat boat1 = duo.get_boat1();
    BoomBoat boat2 = duo.get_boat2();

    // Extract each boat position and velocity without the heading angle
    Vector3d boat1_pos = boat1.get_pos();
    Vector3d boat2_pos = boat2.get_pos();
    Vector3d boat1_vel = boat1.get_vel();
    Vector3d boat2_vel = boat2.get_vel();

    Boom boom = duo.get_boom();
    VectorXd link_i = boom.get_link_state(0);
    VectorXd link_f = boom.get_link_state(boom.get_num_links() - 1);
    double L = boom.get_L();

    // P0: left end of link 0 - closest link to boat1
    double theta_link = link_i(2);
    Vector2d P0 = Vector2d(link_i(0), link_i(1)) - (L / 2) * Vector2d(cos(theta_link), sin(theta_link));
    Vector2d P0_dot = Vector2d(link_i(3), link_i(4)) - (L / 2) * link_i(5) * Vector2d(-sin(theta_link), cos(theta_link));
    
    Vector2d F_spring1 = boom.get_k() * (P0 - Vector2d(boat1_pos(0), boat1_pos(1)));
    // unit vector in the direction of (P0 - boat1_pos)
    Vector2d e1 = (P0 - Vector2d(boat1_pos(0), boat1_pos(1))).normalized();
    Vector2d F_damp1 = boom.get_c() * ((P0_dot - Vector2d(boat1_vel(0), boat1_vel(1))).dot(e1)) * e1;
    Vector2d boom_force1 = F_spring1 + F_damp1;

    // Print boom force1 size and direction
    // cout << "Boom force1 size in informed_control_update: " << boom_force1.norm() << endl;
    // cout << "Boom force2 direction in informed_control_update: " << atan2(boom_force1(1), boom_force1(0)) << endl;
    // cout.flush();
    
    // P1: right end of link num_links - closest link to boat2
    theta_link = link_f(2);
    Vector2d P1 = Vector2d(link_f(0), link_f(1)) + (L / 2) * Vector2d(cos(theta_link), sin(theta_link));
    Vector2d P1_dot = Vector2d(link_f(3), link_f(4)) + (L / 2) * link_f(5) * Vector2d(-sin(theta_link), cos(theta_link));

    Vector2d F_spring2 = boom.get_k() * (P1 - Vector2d(boat2_pos(0), boat2_pos(1)));
    // unit vector in the direction of (P1 - boat2_pos)
    Vector2d e2 = (P1 - Vector2d(boat2_pos(0), boat2_pos(1))).normalized();
    Vector2d F_damp2 = boom.get_c() * ((P1_dot - Vector2d(boat2_vel(0), boat2_vel(1))).dot(e2)) * e2;
    Vector2d boom_force2 = F_spring2 + F_damp2;
    // Print boom force2 size and direction
    // cout << "Boom force2 size in informed_control_update: " << boom_force2.norm() << endl;
    // cout << "Boom force2 direction in informed_control_update: " << atan2(boom_force2(1), boom_force2(0)) << endl;
    // cout.flush();

    // Rotate boom_force1 and boom_force2 to the boat frame
    Matrix2d R1;
    double theta1 = boat1_pos(2);
    R1 << sin(theta1), cos(theta1), cos(theta1), -sin(theta1);
    Vector2d boom_force1_rot = R1 * boom_force1;

    Matrix2d R2;
    double theta2 = boat2_pos(2);
    R2 << sin(theta2), cos(theta2), cos(theta2), -sin(theta2);
    Vector2d boom_force2_rot = R2 * boom_force2;

    double f_u1 = boom_force1_rot(0);
    double f_v1 = boom_force1_rot(1);

    double f_u2 = boom_force2_rot(0);
    double f_v2 = boom_force2_rot(1);

    // // print f_u2 and f_v2
    // cout << "f_u2: " << f_u2 << endl;
    // cout << "f_v2: " << f_v2 << endl;
    // cout.flush();

    // Return the forces in the local frame in the direction of u and v
    Matrix2X2d forces;
    forces(0, 0) = f_u1;
    forces(1, 0) = f_v1;
    forces(0, 1) = f_u2;
    forces(1, 1) = f_v2;
    return forces;
}

Matrix2X2d SetPointController::calculate_du_dv(BoomBoatsDuo duo) {
    // Extract the forces in the local frame
    Matrix2X2d forces = calculate_closest_links_forces(duo);

    // Extract the forces in the direction of u and v
    double f_u1 = forces(0, 0);
    double f_v1 = forces(1, 0);
    double f_u2 = forces(0, 1);
    double f_v2 = forces(1, 1);

    if (this->debug_v == 1) {
        cout << "[DEBUG] f_u1: " << f_u1 << ", f_v1: " << f_v1 << ", f_u2: " << f_u2 << ", f_v2: " << f_v2 << endl;
        cout.flush();
    }

    // Extract the velocities in the boat frame
    Vector3d boat1_vel_frame = duo.get_boat1().get_frame_vel();
    Vector3d boat2_vel_frame = duo.get_boat2().get_frame_vel();

    double u1 = boat1_vel_frame(0);
    double v1 = boat1_vel_frame(1);
    double w1 = boat1_vel_frame(2);
    double u2 = boat2_vel_frame(0);
    double v2 = boat2_vel_frame(1);
    double w2 = boat2_vel_frame(2);

    // Calculate the canceling force for each boat
    double mu_u1 = duo.get_boat1().get_mu_l();
    double mu_u2 = duo.get_boat2().get_mu_l();
    double mu_w1 = duo.get_boat1().get_mu_r();
    double mu_w2 = duo.get_boat2().get_mu_r();
    double m1 = duo.get_boat1().get_mass();
    double m2 = duo.get_boat2().get_mass();

    double r1 = duo.get_boat1().get_radius();
    double r2 = duo.get_boat2().get_radius();


    // Calculate du and dv for each boat (Cancelling drag and link forces)
    double du1 = -f_u1 + mu_u1 * u1 * abs(u1) - m1 * w1 * v1;
    double dv1 = f_v1 + mu_w1 * w1 * abs(w1) / r1 ;

    double du2 = -f_u2 + mu_u2 * u2 * abs(u2) - m2 * w2 * v2;
    double dv2 = f_v2 + mu_w2 * w2 * abs(w2) / r2;

    if (this->debug_v == 1) {
        cout << "[DEBUG] du1: " << du1 << ", dv1: " << dv1 << ", du2: " << du2 << ", dv2: " << dv2 << endl;
        cout.flush();
    }

    // Return the calculated du and dv values
    Matrix2X2d du_dv;
    du_dv(0, 0) = du1;
    du_dv(1, 0) = dv1;
    du_dv(0, 1) = du2;
    du_dv(1, 1) = dv2;

    return du_dv;
}


Matrix2X2d SetPointController::calculate_au_av(BoomBoatsDuo duo) {
    // Calculate the outputs from the controllers
    Matrix2X2d outputs; outputs << 0, 0, 0, 0;
    // First column is for boat 1, second column is for boat 2
    // First row is for au, second row is for av

    
    Matrix2X2d errors; errors << 0, 0, 0, 0;
    // First column is for boat 1, second column is for boat 2
    // First row is for e_u_corr, second row is for e_theta_corr

    // Main idea: Implement a lead-lag controller for each boat, but we divide 
    // the lead controller into two parts: the static gain 1/sqrt(alpha) and the
    // normed controller, which will go in to the feedback branch.
    // This implementation is designed to use all of the advantages of the
    // lead-lag controller, and reduce the impact of the lead zero on the system,
    // thus reducing the overshoot substantially.

    Vector2d theta_curr;
    theta_curr(0) = duo.get_boat1().get_pos()(2);
    theta_curr(1) = duo.get_boat2().get_pos()(2);

    Matrix3X2d setpoint;
    setpoint.col(0) = this->setpoint1;
    setpoint.col(1) = this->setpoint2;

    Matrix2X2d pos;
    pos.col(0) = duo.get_boat1().get_pos().head(2);
    pos.col(1) = duo.get_boat2().get_pos().head(2);
    
    Vector2d vel_u;
    vel_u(0) = duo.get_boat1().get_frame_vel()(0);
    vel_u(1) = duo.get_boat2().get_frame_vel()(0);

    double tmp_y;

    for (int i = 0; i < 2; ++i) {
        // Ready the input to the loop:
        // error_u_corr = ref - c_lead_u_norm * u 
        // error_theta_corr = ref - c_lead_theta_norm * theta

        // u
        // if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - u: setpoint = " << setpoint.col(i).head(2).transpose()
                    //  << ", pos = " << pos.col(i).head(2).transpose() << endl;

        if (setpoint_params["speed_control"] == 1) {
            tmp_y = vel_u(i);
        } else {
            // If we are in location control mode, we will use the boat's position
            tmp_y = (setpoint.col(i).head(2) - pos.col(i).head(2)).norm();
        }

        this->integ_lead(0, i) += tmp_y - last_output_lead(0, i);

        // if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - integ_lead(0, " << i << ") = " << this->integ_lead(0, i) << endl;

        this->last_output_lead(0, i) = this->alpha_u * tmp_y + sqrt(this->alpha_u) * this->wc_u * this->integ_lead(0, i) * this->ts;

        // if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - last_output_lead(0, " << i << ") = " << this->last_output_lead(0, i) << endl;

        if (this->setpoint_params["speed_control"] == 0) {
            errors(0, i) = 0 + this->last_output_lead(0, i); // reference is 0.
        }
        else {
            // double v_ref = this->setpoint_params["speed_reference"];
            errors(0, i) = this->u_ref(i) - this->last_output_lead(0, i); // Reference velocity is predetermined, but can be set to 0 if needed
        }
        

        // if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - errors(0, " << i << ") = " << errors(0, i) << endl;

        // theta
        // if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - theta_curr = " << theta_curr(i) << endl;

        this->integ_lead(1, i) += theta_curr(i) - last_output_lead(1, i);

        // if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - integ_lead(1, " << i << ") = " << this->integ_lead(1, i) << endl;

        this->last_output_lead(1, i) = this->alpha_theta * theta_curr(i) + sqrt(this->alpha_theta) * this->wc_theta * this->integ_lead(1, i) * this->ts;

        // if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - last_output_lead(1, " << i << ") = " << this->last_output_lead(1, i) << endl;

        // if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " Error in theta: " << this->e_theta(i) << endl;

        errors(1, i) = this->ref_ang(i) - this->last_output_lead(1, i); // ref - output of the normed lead controller
        errors(1, i) = wrap_theta(errors(1, i));

        // if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - errors(1, " << i << ") = " << errors(1, i) << endl;
        
        // ----------------------------------------------------------------------------------

        if (this->debug_a == 1) cout << endl;
        // if (this->debug_a == 1) cout << "[debug_a] Calculating outputs for boat " << i + 1 << endl;
        // 1 - au: Gain controller for u
        outputs(0, i) = this->Kp_u * (1/sqrt(this->alpha_u)) * errors(0, i);
        // if (this->debug_a == 1) cout << "[debug_a] outputs(0, " << i << ") after Kp_u gain: " << outputs(0, i) << endl;

        // 1 - av: Gain controller for theta
        outputs(1, i) = this->Kp_theta * (1/sqrt(this->alpha_theta)) * errors(1, i);
        // if (this->debug_a == 1) cout << "[debug_a] outputs(1, " << i << ") after Kp_theta gain: " << outputs(1, i) << endl;

        // 2 - au: Lag controller for u (With anti-windup)
        // If last output was saturated, we will add to the outputs matrix the expression:
        // outputs(0, i) = outputs(0, i) + (saturation_u - last_output_lag(0, i) / Tt_u)

        // if (i == 0) {
        //     // Print the last output lag for u
        //     cout << "Boat " << i + 1 << " - last_output_lag(0, 0) = " << this->last_output_lag(0, 0) << endl;
        //     cout << "Saturation u: " << this->saturation_u << endl;
        //     cout << endl;
        //     cout.flush();
        // }

        if (this->enable_windup_protection == 1) {
            if (abs(this->last_output_lag(0, i)) >= this->saturation_u) {
                outputs(0, i) += (this->saturation_u * sign(this->last_output_lag(0, i)) - this->last_output_lag(0, i)) / this->Tt_u;
                if (this->debug_a == 1) cout << "Added to C_lag input in boat " << i + 1 << ": " << (this->saturation_u * sign(this->last_output_lag(0, i)) - this->last_output_lag(0, i)) / this->Tt_u << endl;
            } // windup protection
        }
        

        if (this->enable_lag == 1) {
            // output = input + (wc / 10) * integ_lag, integ_lag = input
            this->integ_lag(0, i) += outputs(0, i) * this->ts;
            if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - integ_lag(0, " << i << ") = " << this->integ_lag(0, i) << endl;
            outputs(0, i) = outputs(0, i) + (this->wc_u / 10) * this->integ_lag(0, i);
            if (this->debug_a == 1) cout << "[debug_a] outputs(0, " << i << ") after lag controller: " << outputs(0, i) << endl;
        }
            this->last_output_lag(0, i) = outputs(0, i);
        


        // Apply saturation to the outputs
        if (abs(outputs(0, i)) > this->saturation_u) {
            outputs(0,i) = sign(outputs(0, i)) * this->saturation_u;
            if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - Saturation applied to outputs(0, " << i << ") = " << outputs(0, i) << endl;
            cout.flush();
        }
        // 2 - av: Lag controller for theta (With anti-windup)
        // If last output was saturated, we will add to the outputs matrix the expression:
        // outputs(1, i) = outputs(1, i) + (saturation_theta - last_output_lag(1, i) / Tt_theta)

        if (this->enable_windup_protection == 1) {
            if (abs(this->last_output_lag(1, i)) >= this->saturation_theta) {
                outputs(1, i) += (this->saturation_theta * sign(this->last_output_lag(1, i)) - this->last_output_lag(1, i)) / this->Tt_theta;
                if (this->debug_a == 1) cout << "Added to C_lag input: " << (this->saturation_theta * sign(this->last_output_lag(1, i)) - this->last_output_lag(1, i)) / this->Tt_theta << endl;
            } // windup protection
        }
        

        if (this->enable_lag == 1) {
            // output = input + (wc / 10) * integ_lag, integ_lag = input
            this->integ_lag(1, i) += outputs(1, i) * this->ts;
            if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - integ_lag(1, " << i << ") = " << this->integ_lag(1, i) << endl;
            outputs(1, i) = outputs(1, i) + (this->wc_theta / 10) * this->integ_lag(1, i);
            if (this->debug_a == 1) cout << "[debug_a] outputs(1, " << i << ") after lag controller: " << outputs(1, i) << endl;
            this->last_output_lag(1, i) = outputs(1, i);
            // Apply saturation to the outputs
            if (abs(outputs(1, i)) > this->saturation_theta) {
                outputs(1,i) = sign(outputs(1, i)) * this->saturation_theta;
                if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - Saturation applied to outputs(1, " << i << ") = " << outputs(1, i) << endl;
                cout.flush();
            }
        }

        // // Apply saturation to the outputs
        // if (abs(outputs(1, i)) > this->saturation_theta) {
        //     outputs(1, i) = sign(outputs(1, i)) * this->saturation_theta;
        //     if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - Saturation applied to outputs(1, " << i << ") = " << outputs(1, i) << endl;
        //     cout.flush();
        // }
        

        if (this->debug_a == 1) cout << endl;
    }

    // if the error in theta is too high, we will not use the u controller,
    // to prevent the boat from moving in a wrong direction.
    // for (int i = 0; i < 2; ++i) {
    //     if (abs(this->e_theta(i)) > this->max_error_theta) {
    //         outputs(0, i) = 0;
    //         this->integ_lag(0, i) = 0; // Reset the lag integrator for u
    //         this->last_output_lag(0, i) = 0; // Reset the last output for u
    //         this->integ_lead(0, i) = 0; // Reset the lead integrator for u
    //         this->last_output_lead(0, i) = 0; // Reset the last output for u
    //         if (this->debug_a == 1) cout << "[debug_a] Boat " << i + 1 << " - Error in theta is too high, setting outputs(0, " << i << ") to 0." << endl;
    //         if (this->debug_a == 1) cout << "[debug_a] Outputs(1, " << i << ") = " << outputs(1, i) << endl;
    //     }
    // }

    if (this->debug_a == 1) cout.flush();

    return outputs;
}

// Reset the controllers
void SetPointController::reset_controllers() {
    // Reset the integrators and last values
    this->integ_lag.setZero(2, 2);
    // this->integ_lead.setZero(2, 2);
    // this->last_output_lag.setZero(2, 2);
    // this->last_output_lead.setZero(2, 2);
    // Print reset message
    cout << "SetPointController: Resetting controllers." << endl;
    cout.flush();
}

Matrix2X2d SetPointController::calculate_control_signals(Matrix2X2d a, Matrix2X2d d) {
    // Extract the control signals
    double au1 = a(0, 0);
    double av1 = a(1, 0);
    double au2 = a(0, 1);
    double av2 = a(1, 1);

    // double au1 = 1000;
    // double av1 = 0;
    // double au2 = 1000;
    // double av2 = 0;

    // Extract the du and dv values
    double du1 = d(0, 0);
    double dv1 = d(1, 0);
    double du2 = d(0, 1);
    double dv2 = d(1, 1);


    // Correct the control input
    double F1_pos = sqrt(pow(au1 + du1, 2) + pow(av1 + dv1, 2));
    double F1_neg = -F1_pos;
    double F1;
    double eta1;
    double eta1_pos;
    double eta1_neg;
    // If F1 is zero, eta1 will be output(1, 0)
    if (F1_pos == 0) {
        F1 = 0;
        eta1 = 0;
    } else {
        eta1_pos = atan2((av1 + dv1) / F1_pos, (au1 + du1) / F1_pos);
        eta1_neg = atan2((av1 + dv1) / F1_neg, (au1 + du1) / F1_neg);
        // Choose the eta value that is already wrapped
        if (wrap_eta(eta1_pos) == eta1_pos) {
            eta1 = eta1_pos;
            F1 = F1_pos;
        } else {
            eta1 = eta1_neg;
            F1 = F1_neg;
        }
    }

    double F2_pos = sqrt(pow(au2 + du2, 2) + pow(av2 + dv2, 2));
    double F2_neg = -F2_pos;
    double F2;
    double eta2;
    double eta2_pos;
    double eta2_neg;
    // If F2 is zero, eta2 will be output(1, 1)
    if (F2_pos == 0) {
        F2 = 0;
        eta2 = 0;
    } else {
        eta2_pos = atan2((av2 + dv2) / F2_pos, (au2 + du2) / F2_pos);
        eta2_neg = atan2((av2 + dv2) / F2_neg, (au2 + du2) / F2_neg);
        // Choose the eta value that is already wrapped
        if (wrap_eta(eta2_pos) == eta2_pos) {
            eta2 = eta2_pos;
            F2 = F2_pos;
        } else {
            eta2 = eta2_neg;
            F2 = F2_neg;
        }
    }


    // Return the control signals
    Matrix2X2d control_signals;
    control_signals(0, 0) = F1;
    control_signals(1, 0) = eta1;
    control_signals(0, 1) = F2;
    control_signals(1, 1) = eta2;


    if (this->debug_a) {
        cout << "----------------------------------------" << endl;
        cout << "Control signals for boat 1: F = " << F1 << ", eta = " << eta1 << endl;
        cout << "Control signals for boat 2: F = " << F2 << ", eta = " << eta2 << endl;
        cout.flush();
    }

    return control_signals;
}

Matrix2X2d SetPointController::update(BoomBoatsDuo duo) {
    // this->update_errors(duo);
    
    cout << "At time: " << duo.get_time() << " [s]" << endl;

    Matrix3X2d pos;
    pos.col(0) = duo.get_boat1().get_pos();
    pos.col(1) = duo.get_boat2().get_pos();
    pos(2, 0) = wrap_theta(pos(2, 0)); // Wrap the angle to [-pi, pi]
    pos(2, 1) = wrap_theta(pos(2, 1)); // Wrap the angle to [-pi, pi]

    Matrix3X2d setpoint;    
    setpoint.col(0) = this->setpoint1;
    setpoint.col(1) = this->setpoint2;

    // Set reference angles based on the setpoints, and later change them if needed
    this->ref_ang(0) = atan2(setpoint(1, 0) - pos(1, 0), setpoint(0, 0) - pos(0, 0));
    this->ref_ang(0) = PI / 2 - this->ref_ang(0);
    this->ref_ang(1) = atan2(setpoint(1, 1) - pos(1, 1), setpoint(0, 1) - pos(0, 1));
    this->ref_ang(1) = PI / 2 - this->ref_ang(1);
    // Set the reference velocity to the speed reference, and later change it if needed
    this->u_ref(0) = this->setpoint_params["speed_reference"];
    this->u_ref(1) = this->setpoint_params["speed_reference"];

    // Calculate e_u
    for (int i = 0; i < 2; ++i) {
        // Calculate the error in position
        this->e_u(i) = (setpoint.col(i).head(2) - pos.col(i).head(2)).norm();
    }

    bool cond1 = (this->e_u(0) < this->tolerance_d);
    bool cond2 = (this->e_u(1) < this->tolerance_d);

    if (cond1) {
        this->waiting_for_other[0] = true;
    }
    if (cond2) {
        this->waiting_for_other[1] = true;
    }

    // If one error is under tolerance and the other is not, we will set the velocity reference to 0
    // If both errors are above tolerance, we will set the velocity reference to the speed reference and set the ref_ang
    // If a boat is under the tolerance, we will set the ref_ang to the angle of the next setpoint
    if (this->setpoint_params["speed_control"] == 1) {

        if (this->waiting_for_other[0] && !this->waiting_for_other[1]) {
            cout << "Boat 1 is waiting for boat 2 to reach the setpoint." << endl;
            cout.flush();
            this->u_ref(0) = 0;

            this->ref_ang(0) = setpoint(2, 0); // Set the reference angle to the angle of the next setpoint
            this->ref_ang(0) = PI / 2 - this->ref_ang(0); // Convert to the correct angle

            // if (this-> waiting_for_other[1] ) { // If boat 2 is waiting for boat 1, boat 2 will stop
            //     this->u_ref(1) = 0; // Set the velocity reference to 0
            //     cout << "Boat 2 is waiting for boat 1 to reach the setpoint." << endl;
            // } 

        } else if (!this->waiting_for_other[0] && this->waiting_for_other[1]) {
            cout << "Boat 2 is waiting for boat 1 to reach the setpoint." << endl;
            cout.flush();
            this->u_ref(1) = 0;

            this->ref_ang(1) = setpoint(2, 1); // Set the reference angle to the angle of the next setpoint
            this->ref_ang(1) = PI / 2 - this->ref_ang(1); // Convert to the correct angle

        } else if (this->waiting_for_other[0] && this->waiting_for_other[1]) { // later we will update setpoint to the next one
            cout << "Both boats are waiting for each other to reach the setpoint." << endl;
            cout.flush();
            // Do nothing, later we will update the setpoint to the next one

        } else {
            cout << "Both boats are advancing towards their setpoints." << endl;
            cout.flush();
            // Do nothing, both boats are advancing towards their setpoints.            
        }



        // In addition, we will check if the orientation error is above the threshold
        for (int i = 0; i < 2; ++i) {
            if (abs(wrap_theta(this->e_theta(i))) > this->max_error_theta) {
                this->u_ref(i) = 0; // Set the velocity reference to 0 if the orientation error is too high
                // reset_controllers(); // Reset the controllers to prevent overshoot
                cout << "Boat " << i + 1 << " orientation error is too high: " << this->e_theta(i) << ". Setting velocity reference to 0." << endl;
                cout.flush();
            }
        }

        // Check if both boats are close to the setpoint
        if (this->waiting_for_other[0] && this->waiting_for_other[1]) {
            // If the setpoint is reached, update the setpoint to the next one
            if (this->current_index == this->path_1.cols() - 1) {
                cout << "Both boats reached the setpoint. No more setpoints to update to." << endl;
                cout << "Boat 1: (" << this->path_1.col(this->current_index)[0] << ", " << this->path_1.col(this->current_index)[1] << ")" << endl;
                cout << "Boat 2: (" << this->path_2.col(this->current_index)[0] << ", " << this->path_2.col(this->current_index)[1] << ")" << endl;
                cout.flush();

                if (!this->EOP) { // If no mark of end of path, set the reference angles to the last setpoint, otherwise
                    this->u_ref.setZero(); // Set the velocity reference to 0
                    this->ref_ang(0) = this->setpoint1(2);
                    this->ref_ang(0) = PI / 2 - this->ref_ang(0);
                    this->ref_ang(1) = this->setpoint2(2);
                    this->ref_ang(1) = PI / 2 - this->ref_ang(1);
                } else {
                    this->EOP = true;
                }
                

            } else if (this->current_index + this->skip_index >= this->path_1.cols()) {
                this->current_index = this->path_1.cols() - 1; // Reache the last point if we are already there
                cout << "Boat 1: (" << this->path_1.col(this->current_index)[0] << ", " << this->path_1.col(this->current_index)[1] << ")" << endl;
                cout << "Boat 2: (" << this->path_2.col(this->current_index)[0] << ", " << this->path_2.col(this->current_index)[1] << ")" << endl;
                cout << "Next setpoint is the last one. No more setpoints to update to." << endl;
                cout.flush();

                // Update the reference angles to the last setpoint
                this->setpoint1 = this->path_1.col(this->current_index);
                this->setpoint2 = this->path_2.col(this->current_index);

                // Update the reference angles
                this->ref_ang(0) = atan2(this->setpoint1(1) - pos(1, 0), this->setpoint1(0) - pos(0, 0));
                this->ref_ang(0) = PI / 2 - this->ref_ang(0);
                this->ref_ang(1) = atan2(this->setpoint2(1) - pos(1, 1), this->setpoint2(0) - pos(0, 1));
                this->ref_ang(1) = PI / 2 - this->ref_ang(1);

            } else {
                this->current_index = this->current_index + this->skip_index;

                this->setpoint1 = this->path_1.col(this->current_index);
                this->setpoint2 = this->path_2.col(this->current_index); 

                cout << "Setpoint reached at time " << duo.get_time() << " [s]. Updating to the next one. " << endl;
                cout << "Boat 1: (" << this->path_1.col(this->current_index)[0] << ", " << this->path_1.col(this->current_index)[1] << ")" << endl;
                cout << "Boat 2: (" << this->path_2.col(this->current_index)[0] << ", " << this->path_2.col(this->current_index)[1] << ")" << endl;
                cout.flush();

                // Update the reference angles
                this->ref_ang(0) = atan2(this->setpoint1(1) - pos(1, 0), this->setpoint1(0) - pos(0, 0));
                this->ref_ang(0) = PI / 2 - this->ref_ang(0);
                this->ref_ang(1) = atan2(this->setpoint2(1) - pos(1, 1), this->setpoint2(0) - pos(0, 1));
                this->ref_ang(1) = PI / 2 - this->ref_ang(1);
            }

            this->waiting_for_other[0] = false;
            this->waiting_for_other[1] = false;  
            
            

            // cout << endl;
            // cout << "Current setpoint boat 1: (" << this->setpoint1.head(2).transpose() << ")" << endl;
            // cout << "Next setpoint boat 1: (" << this->path_1.col(this->current_index).head(2).transpose() << ")" << endl;
            // cout << endl;

            // // Update the setpoints' flags if needed
            // if (this->setpoint1.head(2) == this->path_1.col(this->current_index).head(2)) {
            //     this->setpoint1_same = true;
            //     cout << "Boat 1 setpoint is the same as the current path point." << endl;
            // } else {
            //     this->setpoint1_same = false;
            //     cout << "Boat 1 setpoint is different from the current path point." << endl;
            // }

            // if (this->setpoint2.head(2) == this->path_2.col(this->current_index).head(2)) {
            //     this->setpoint2_same = true;
            // } else {
            //     this->setpoint2_same = false;
            // }

            // Reset
            this->reset_controllers();
            // cout.flush();
            this->setpoint_updated = true;
        }
        else {
            this->setpoint_updated = false;
        }
    }

    
    

    for (int i = 0; i < 2; ++i) {
            // Calculate the orientation error
            // pos(2, i) = wrap_theta(pos(2, i)); // Wrap the angle to [-pi, pi]
            // this->ref_ang(i) = wrap_theta(this->ref_ang(i)); // Wrap the reference angle to [-pi, pi]
            this->e_theta(i) = (this->ref_ang(i) - pos(2, i));
            cout << "Boat " << i + 1 << " ref_ang: " << this->ref_ang(i) << endl;
            cout << "Boat " << i + 1 << " heading: " << pos(2, i) << endl;
            cout << "Boat " << i + 1 << " e_theta: " << this->e_theta(i) << endl;
            cout << endl;
            cout.flush();
    }

    // Check if the next setpoint is the same as last one ( if a boat is at the end of the path )

    if (this->setpoint_params["speed_control"] == 0) {
        

        Matrix3X2d next_setpoint; // If current_index is the last index, next_setpoint will be the same as setpoint
        next_setpoint.col(0) = this->path_1.col(this->current_index);
        next_setpoint.col(1) = this->path_2.col(this->current_index);

        

        // Vector of 2 bools
        vector<bool> same_setpoint(2);
        same_setpoint[0] = this->setpoint1_same;
        same_setpoint[1] = this->setpoint2_same;

        

        // Calculate angles first
        for (int i = 0; i < 2; i++) {
            if (same_setpoint[i]) {
                this->ref_ang(i) = setpoint(2, i);
                this->ref_ang(i) = PI / 2 - this->ref_ang(i);
                if (i ==0) {
                    cout << "Boat 1 - ref_ang is the same as setpoint: " << this->ref_ang(i) << endl;
                }
                this->e_u(i) = 0;
            }
            else {
                this->ref_ang(i) = atan2(next_setpoint(1, i) - pos(1, i), next_setpoint(0, i) - pos(0, i));
                this->ref_ang(i) = PI / 2 - this->ref_ang(i);
                if (i == 0) {
                    cout << "Boat 1 - ref_ang is calculated from next setpoint: " << this->ref_ang(i) << endl;
                }
                this->e_u(i) = (setpoint.col(i).head(2) - pos.col(i).head(2)).norm();
            }
            this->e_theta(i) = (this->ref_ang(i) - pos(2, i));
        }
        cout << endl;
    }
    
    // Print both reference angles
    // cout << "Boat 1 - ref_ang: " << fixed << setprecision(5) << this->ref_ang(0) << endl;
    // cout << "Boat 1 - heading: " << fixed << setprecision(5) << pos(2, 0) << endl;
    // cout << "Boat 1 - e_theta: " << fixed << setprecision(5) << this->e_theta(0) << endl;

    // cout << "Boat 2 - ref_ang: " << fixed << setprecision(5) << this->ref_ang(1) << endl;
    // cout << "Boat 2 - heading: " << fixed << setprecision(5) << pos(2, 1) << endl;
    // cout << "Boat 2 - e_theta: " << fixed << setprecision(5) << this->e_theta(1) << endl;
    

    // Unwrap angles
    // Print last ref angle for boat 1 and new ref angle
    // cout << "Boat 1 - last ref_ang: " << fixed << setprecision(5) << this->last_ref_ang(0) << endl;
    // cout << "Boat 1 - new ref_ang (before unwrap): " << fixed << setprecision(5) << this->ref_ang(0) << endl;
    // this->ref_ang = unwrap_angles(this->ref_ang, this->last_ref_ang, this->first_update_ref_ang);
    // this->e_theta = unwrap_angles(this->e_theta, this->last_e_theta, this->first_update_e_theta);
    // wrap e_theta
    cout << "After wrapping angles:" << endl;
    for (int i = 0; i < 2; ++i) {
        this->ref_ang(i) = wrap_theta(this->ref_ang(i));
        this->e_theta(i) = wrap_theta(this->e_theta(i));
        cout << "Boat " << i + 1 << " - ref_ang: " << fixed << setprecision(5) << this->ref_ang(i) << endl;
        cout << "Boat " << i + 1 << " - e_theta: " << fixed << setprecision(5) << this->e_theta(i) << endl;
        cout << endl;
        cout.flush();
    }

    // print u velocities
    cout << "Boat 1 - u: " << duo.get_boat1().get_frame_vel()(0) << endl;
    cout << "Boat 2 - u: " << duo.get_boat2().get_frame_vel()(0) << endl;
    // cout << "Boat 1 - ref_ang (after unwrap): " << fixed << setprecision(5) << this->ref_ang(0) << " at time: " << duo.get_time() << endl;

    // Update last values AFTER unwrapping
    this->last_ref_ang = this->ref_ang;
    this->last_e_theta = this->e_theta;
    this->first_update_ref_ang = false;
    this->first_update_e_theta = false;

    // for (int i = 0; i < 2; ++i) {
    // Print heading of boat 1
    // cout << "Boat 1 - heading: " << fixed << setprecision(5) << pos(2, 0) << endl;
    // cout << "Boat 1 - position: (" << fixed << setprecision(5) << pos(0, 0) << ", " << pos(1, 0) << ")" << endl;
    // cout << "Boat 1 - next setpoint: (" << fixed << setprecision(5) << next_setpoint(0, 0) << ", " << next_setpoint(1, 0) << ", " << next_setpoint(2, 0) << ")" << endl;
    // cout << "Boat 1 - e_u: " << this->e_u(0) << endl;
    // cout << "Boat 2 - e_u: " << this->e_u(1) << endl;
    // cout << "Boat 1 - e_theta: " << fixed << setprecision(5) << this->e_theta(0) << endl;
    cout << endl;
    cout << "----------------------------------------" << endl;
    cout.flush();
    // }
    // cout.flush();

    // this->setpoint1 = next_setpoint.col(0);
    // this->setpoint2 = next_setpoint.col(1);


    // Calculate the du and dv values
    Matrix2X2d du_dv = this->calculate_du_dv(duo);
    // Calculate the au and av values
    Matrix2X2d au_av = this->calculate_au_av(duo);
    // Calculate the control signals
    Matrix2X2d control_signals = this->calculate_control_signals(au_av, du_dv);
    // Return the control signals
    return control_signals;
}

void SetPointController::print_to_file(const string &filename, const string &foldername,
            const BoomBoatsDuo &duo) const {
                // check if the folder exists, if not create it
                if (!filesystem::exists(foldername)) {
                    filesystem::create_directory(foldername);
                    cout << "Created folder: " << foldername << endl;
                    cout.flush();
                    // Need to create the file in the folder
                    ofstream file(foldername + "/" + filename + ".txt");
                    if (!file.is_open()) {
                        throw std::runtime_error("Could not open file for writing in print_to_file in controller - error a");
                    }
                }

                // Open file in append mode
                ofstream file(foldername + "/" + filename + ".txt", ios::app);
                if (!file.is_open()) {
                    throw std::runtime_error("Could not open file for writing in print_to_file in controller - error b");
                }

                // If setpoint is updated, add a note to the file
                if (this->setpoint_updated) {
                    file << "# New setpoint" << endl;
                }

                // write e_u and e_theta to the file
                file << e_u(0) << " " << e_u(1) << " " << e_theta(0) << " " << e_theta(1) << endl;
            }