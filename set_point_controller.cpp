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

    this->Kp_u = m * this->wc_u * this->wc_u;

    double phase_margin_u = setpoint_params["phase_margin_u"];
    double phi_lead_u = phase_margin_u + 5.7; // 5.7 because of lag controller
    phi_lead_u *= PI / 180; // Convert to radians
    this->alpha_u = (1 + sin(phi_lead_u)) / (1 - sin(phi_lead_u));

    this->Tt_u = setpoint_params["Tt_u"];

    this->saturation_u = setpoint_params["sat_u"];

    this->tolerance_d = setpoint_params["tolerance_d"];

    this->ts = params["simulation"]["time_step"];

    this->max_error_theta = setpoint_params["max_error_theta_deg"].get<double>() * PI / 180; // Convert to radians

    this->debug = setpoint_params["debug"];
    
    // Initialize integrators and last values of outputs
    this->reset_controllers();
   
    // Initialize error values
    this->e_theta.setZero(2, 1);
    this->e_u.setZero(2, 1);

    // Initialize path division parameters
    this->skip_index = setpoint_params["skip_index"];

    // Initialize setpoints to the second point of the path (to avoid overboarding the rope)
    this->setpoint1 = path_1.col(this->skip_index);
    this->setpoint2 = path_2.col(this->skip_index);
    this->current_index = this->skip_index;
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
// Update the error values
void SetPointController::update_errors(BoomBoatsDuo duo) {
    // Update the error values based on the current state of the boats and the setpoints

    // Update the error in u (distance to the setpoint)
    Matrix3X2d setpoint;
    setpoint.col(0) = this->setpoint1;
    setpoint.col(1) = this->setpoint2;

    // cout << "Current setpoint for boat 1: " << this->setpoint1.transpose() << endl;
    // cout << "Current setpoint for boat 2: " << this->setpoint2.transpose() << endl;
    // cout.flush();

    Matrix3X2d position;
    position.col(0) = duo.get_boat1().get_pos();
    position.col(1) = duo.get_boat2().get_pos();
    
    for (int i = 0; i < 2; i++) {
        e_u(i) = (setpoint.col(i).head(2) - position.col(i).head(2)).norm();
    }

    // Update the error in theta (angle to the setpoint)
    // If the error jumped from -pi to pi (or vice versa), we need to reset integrators
    // to avoid a jump in the output
    double ref_ang;
    // double tmp;
    for (int i = 0; i < 2; i++) {
        ref_ang = atan2(setpoint(1, i) - position(1, i), setpoint(0, i) - position(0, i));
        ref_ang = PI / 2 - ref_ang;
        // tmp = this->e_theta(i);
        
        this->e_theta(i) = ref_ang - position(2, i);
        // // Check if the error jumped from -pi to pi (or vice versa)
        // if (abs(this->e_theta(i) - tmp) > 0.1 * PI) {
        //     this->reset_controllers();
        //     cout << "Resetting controllers due to angle jump" << endl;
        // }
    }

    if (this->debug) {
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
        cout << "Boat " << i + 1 << " error in u: " << e_u(i) << endl;
        cout << "Boat " << i + 1 << " error in theta: " << e_theta(i) << endl << endl;
    }
    cout << "----------------------------------------" << endl;
    cout.flush();
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

    // // print f_u1 and f_u2
    // cout << "f_u1: " << f_u1 << endl;
    // cout << "f_u2: " << f_u2 << endl;
    // cout.flush();

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

    double ref_ang;


    for (int i = 0; i < 2; ++i) {
        // Ready the input to the loop:
        // error_u_corr = ref - c_lead_u_norm * u 
        // error_theta_corr = ref - c_lead_theta_norm * theta

        // u
        if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " - u: setpoint = " << setpoint.col(i).head(2).transpose()
                     << ", pos = " << pos.col(i).head(2).transpose() << endl;

        this->integ_lead(0, i) += (setpoint.col(i).head(2) - pos.col(i).head(2)).norm() - last_output_lead(0, i);

        if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " - integ_lead(0, " << i << ") = " << this->integ_lead(0, i) << endl;

        this->last_output_lead(0, i) = this->alpha_u * (setpoint.col(i).head(2) - pos.col(i).head(2)).norm() + sqrt(this->alpha_u) * this->wc_u * this->integ_lead(0, i) * this->ts;

        if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " - last_output_lead(0, " << i << ") = " << this->last_output_lead(0, i) << endl;

        errors(0, i) = 0 + this->last_output_lead(0, i); // reference is 0.

        if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " - errors(0, " << i << ") = " << errors(0, i) << endl;

        // theta
        if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " - theta_curr = " << theta_curr(i) << endl;

        this->integ_lead(1, i) += theta_curr(i) - last_output_lead(1, i);

        if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " - integ_lead(1, " << i << ") = " << this->integ_lead(1, i) << endl;

        this->last_output_lead(1, i) = this->alpha_theta * theta_curr(i) + sqrt(this->alpha_theta) * this->wc_theta * this->integ_lead(1, i) * this->ts;

        if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " - last_output_lead(1, " << i << ") = " << this->last_output_lead(1, i) << endl;

        if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " Error in theta: " << this->e_theta(i) << endl;


        ref_ang = atan2(setpoint(1, i) - pos(1, i), setpoint(0, i) - pos(0, i));
        ref_ang = PI / 2 - ref_ang;
        errors(1, i) = ref_ang - this->last_output_lead(1, i); // ref - output of the normed lead controller

        if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " - errors(1, " << i << ") = " << errors(1, i) << endl;
        
        // ----------------------------------------------------------------------------------

        if (this->debug == 1) cout << "[DEBUG] Calculating outputs for boat " << i + 1 << endl;
        // 1 - au: Gain controller for u
        outputs(0, i) = this->Kp_u * (1/sqrt(this->alpha_u)) * errors(0, i);
        if (this->debug == 1) cout << "[DEBUG] outputs(0, " << i << ") after Kp_u gain: " << outputs(0, i) << endl;

        // 1 - av: Gain controller for theta
        outputs(1, i) = this->Kp_theta * (1/sqrt(this->alpha_theta)) * errors(1, i);
        if (this->debug == 1) cout << "[DEBUG] outputs(1, " << i << ") after Kp_theta gain: " << outputs(1, i) << endl;

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

        if (abs(this->last_output_lag(0, i)) >= this->saturation_u) {
            outputs(0, i) += (this->saturation_u * sign(this->last_output_lag(0, i)) - this->last_output_lag(0, i)) / this->Tt_u;
            if (this->debug == 1) cout << "Added to C_lag input in boat " << i + 1 << ": " << (this->saturation_u * sign(this->last_output_lag(0, i)) - this->last_output_lag(0, i)) / this->Tt_u << endl;
        } // windup protection

        // output = input + (wc / 10) * integ_lag, integ_lag = input
        this->integ_lag(0, i) += outputs(0, i) * this->ts;
        if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " - integ_lag(0, " << i << ") = " << this->integ_lag(0, i) << endl;
        outputs(0, i) = outputs(0, i) + (this->wc_u / 10) * this->integ_lag(0, i);
        if (this->debug == 1) cout << "[DEBUG] outputs(0, " << i << ") after lag controller: " << outputs(0, i) << endl;
        this->last_output_lag(0, i) = outputs(0, i);


        // Apply saturation to the outputs
        if (abs(outputs(0, i)) > this->saturation_u) {
            outputs(0,i) = sign(outputs(0, i)) * this->saturation_u;
            if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " - Saturation applied to outputs(0, " << i << ") = " << outputs(0, i) << endl;
            cout.flush();
        }
        // 2 - av: Lag controller for theta (With anti-windup)
        // If last output was saturated, we will add to the outputs matrix the expression:
        // outputs(1, i) = outputs(1, i) + (saturation_theta - last_output_lag(1, i) / Tt_theta)

        if (abs(this->last_output_lag(1, i)) >= this->saturation_theta) {
            outputs(1, i) += (this->saturation_theta * sign(this->last_output_lag(1, i)) - this->last_output_lag(1, i)) / this->Tt_theta;
            if (this->debug == 1) cout << "Added to C_lag input: " << (this->saturation_theta * sign(this->last_output_lag(1, i)) - this->last_output_lag(1, i)) / this->Tt_theta << endl;
        } // windup protection

        // output = input + (wc / 10) * integ_lag, integ_lag = input
        this->integ_lag(1, i) += outputs(1, i) * this->ts;
        if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " - integ_lag(1, " << i << ") = " << this->integ_lag(1, i) << endl;
        outputs(1, i) = outputs(1, i) + (this->wc_theta / 10) * this->integ_lag(1, i);
        if (this->debug == 1) cout << "[DEBUG] outputs(1, " << i << ") after lag controller: " << outputs(1, i) << endl;
        this->last_output_lag(1, i) = outputs(1, i);
        // Apply saturation to the outputs
        if (abs(outputs(1, i)) > this->saturation_theta) {
            outputs(1,i) = sign(outputs(1, i)) * this->saturation_theta;
            if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " - Saturation applied to outputs(1, " << i << ") = " << outputs(1, i) << endl;
            cout.flush();
        }

        if (this->debug == 1) cout << endl;
    }

    // if the error in theta is too high, we will not use the u controller,
    // to prevent the boat from moving in a wrong direction.
    for (int i = 0; i < 2; ++i) {
        if (abs(this->e_theta(i)) > this->max_error_theta) {
            outputs(0, i) = 0;
            this->integ_lag(0, i) = 0; // Reset the lag integrator for u
            this->last_output_lag(0, i) = 0; // Reset the last output for u
            this->integ_lead(0, i) = 0; // Reset the lead integrator for u
            this->last_output_lead(0, i) = 0; // Reset the last output for u
            if (this->debug == 1) cout << "[DEBUG] Boat " << i + 1 << " - Error in theta is too high, setting outputs(0, " << i << ") to 0." << endl;
        }
    }

    if (this->debug == 1) cout.flush();

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


    if (this->debug) {
        cout << "----------------------------------------" << endl;
        cout << "Control signals for boat 1: F = " << F1 << ", eta = " << eta1 << endl;
        cout << "Control signals for boat 2: F = " << F2 << ", eta = " << eta2 << endl;
        cout.flush();
    }

    return control_signals;
}

Matrix2X2d SetPointController::update(BoomBoatsDuo duo) {
    // Update the errors
    this->update_errors(duo);

    // Check if both boats are close to the setpoint
    if (this->e_u(0) < this->tolerance_d && this->e_u(1) < this->tolerance_d) {
        // If the setpoint is reached, update the setpoint to the next one
        if (this->current_index + this->skip_index >= this->path_1.cols()) {
            this->current_index = this->path_1.cols() - 1; // Stay at the last point if we are already there
            cout << "Setpoint reached. No more setpoints to update to." << endl;
            cout << "Boat 1: (" << this->path_1.col(this->current_index)[0] << ", " << this->path_1.col(this->current_index)[1] << ")" << endl;
            cout << "Boat 2: (" << this->path_2.col(this->current_index)[0] << ", " << this->path_2.col(this->current_index)[1] << ")" << endl;
            cout.flush();
        }
        else {
            this->current_index = this->current_index + this->skip_index;
            cout << "Setpoint reached. Updating to the next one. " << endl;
            cout << "Boat 1: (" << this->path_1.col(this->current_index)[0] << ", " << this->path_1.col(this->current_index)[1] << ")" << endl;
            cout << "Boat 2: (" << this->path_2.col(this->current_index)[0] << ", " << this->path_2.col(this->current_index)[1] << ")" << endl;
            cout.flush();
        }
        this->setpoint1 = this->path_1.col(this->current_index);
        this->setpoint2 = this->path_2.col(this->current_index);
        // Reset all integrators and last values
        this->reset_controllers();
        // Update the errors again after changing the setpoint
        this->update_errors(duo);
        // cout.flush();
    }

    // Calculate the du and dv values
    Matrix2X2d du_dv = this->calculate_du_dv(duo);
    // Calculate the au and av values
    Matrix2X2d au_av = this->calculate_au_av(duo);
    // Calculate the control signals
    Matrix2X2d control_signals = this->calculate_control_signals(au_av, du_dv);
    // Return the control signals
    return control_signals;
}