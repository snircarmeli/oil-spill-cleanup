#include "non_linear_controller.h"

NonLinearController::NonLinearController(string path_to_json) {
    // Open and parse the JSON file
    std::ifstream file(path_to_json);
    if (!file.is_open()) {
        std::cerr << "Failed to open JSON file: " << path_to_json << endl;
        exit(1); // or throw an exception
    }
    json params;
    file >> params;
    file.close();

    // Extract controller parameters.
    // Adjust the JSON keys as needed—here we assume they are stored under "NonLinearController".
    json controller_params = params["NonLinearController"];
    this->k_u = controller_params["k_u"];
    this->k_v = controller_params["k_v"];
    this->k_theta = controller_params["k_theta"];
    this->epsilon = controller_params["epsilon"]; // For the calculation of denominators
    this->ts = params["simulation"]["time_step"]; // Time step

    // Initialize the error values
    this->e_u << 0.0, 0.0;
    this->e_v << 0.0, 0.0;
    this->e_r << 0.0, 0.0;
    this->e_theta << 0.0, 0.0;
    this->int_e_theta << 0.0, 0.0;
    this->e_u_dot << 0.0, 0.0;
    this->e_v_dot << 0.0, 0.0;
    // Initialize the output (control signals) to zero
    this->output = Matrix2X2d::Zero();

    // Additional initialization (if needed) involving the duo can go here.
}

NonLinearController::~NonLinearController() {
    // Cleanup code if needed
}


Vector2d NonLinearController::get_e_u() {
    return e_u;
}

Vector2d NonLinearController::get_e_v() {
    return e_v;
}

Vector2d NonLinearController::get_e_r() {
    return e_r;
}

Vector2d NonLinearController::get_e_theta() {
    return e_theta;
}

void NonLinearController::update_errors(BoomBoatsDuo duo, Vector3d setpoint1,
    Vector3d setpoint2, Vector3d setpoint1_dot, Vector3d setpoint2_dot) {
   // For each boat, get orientation and position - theta, x, y
   Vector3d boat1_pos = duo.get_boat1().get_pos();
   Vector3d boat2_pos = duo.get_boat2().get_pos();

   double x1 = boat1_pos(0);
   double y1 = boat1_pos(1);
   double theta1 = boat1_pos(2);
   double x2 = boat2_pos(0);
   double y2 = boat2_pos(1);
   double theta2 = boat2_pos(2);

    // Get the velocities
    double x1_dot = duo.get_boat1().get_vel()(0);
    double y1_dot = duo.get_boat1().get_vel()(1);
    double theta1_dot = duo.get_boat1().get_vel()(2);
    double x2_dot = duo.get_boat2().get_vel()(0);
    double y2_dot = duo.get_boat2().get_vel()(1);
    double theta2_dot = duo.get_boat2().get_vel()(2);

   // Get the setpoints
   double x1_ref = setpoint1(0);
   double y1_ref = setpoint1(1);
   double theta1_ref = setpoint1(2);
   double x2_ref = setpoint2(0);
   double y2_ref = setpoint2(1);
    double theta2_ref = setpoint2(2);

   // Get the setpoint velocities
   double x1_ref_dot = setpoint1_dot(0);
   double y1_ref_dot = setpoint1_dot(1);
   double x2_ref_dot = setpoint2_dot(0);
   double y2_ref_dot = setpoint2_dot(1);

   // Calculate the errors
   this->e_u(0) = -sin(theta1) * (x1_ref-x1) - cos(theta1) * (y1_ref-y1);
   this->e_u(1) = -sin(theta2) * (x2_ref-x2) - cos(theta2) * (y2_ref-y2);

   this->e_v(0) = -cos(theta1) * (x1_ref-x1) + sin(theta1) * (y1_ref-y1);
   this->e_v(1) = -cos(theta2) * (x2_ref-x2) + sin(theta2) * (y2_ref-y2);

   this->e_r(0) = x1_ref_dot * (x1_ref - x1) + y1_ref_dot * (y1_ref - y1);
   this->e_r(1) = x2_ref_dot * (x2_ref - x2) + y2_ref_dot * (y2_ref - y2);



   this->e_theta(0) = wrap_theta(theta1_ref - (PI / 2 - theta1));
   this->e_theta(1) = wrap_theta(theta2_ref - (PI / 2 - theta2));

   // Print e_theta
   cout << "e_theta1: " << e_theta(0) << endl;
   cout << "e_theta2: " << e_theta(1) << endl;
   cout << endl;
   cout.flush();

   this->e_u_dot(0) = theta1_dot * this->e_v(0) - (sin(theta1) * (x1_ref_dot - x1_dot) + cos(theta1) * (y1_ref_dot - y1_dot));
   this->e_u_dot(1) = theta2_dot * this->e_v(1) - (sin(theta2) * (x2_ref_dot - x2_dot) + cos(theta2) * (y2_ref_dot - y2_dot));

   this->e_v_dot(0) = -theta1_dot * this->e_u(0) - (cos(theta1) * (x1_ref_dot - x1_dot) - sin(theta1) * (y1_ref_dot - y1_dot));
   this->e_v_dot(1) = -theta2_dot * this->e_u(1) - (cos(theta2) * (x2_ref_dot - x2_dot) - sin(theta2) * (y2_ref_dot - y2_dot));

   this->int_e_theta(0) += this->e_theta(0) * this->ts;
   this->int_e_theta(1) += this->e_theta(1) * this->ts;

    // // Print all errors
    // cout << "e_u1: " << e_u(0) << endl;
    // cout << "e_u2: " << e_u(1) << endl;
    // cout << "e_v1: " << e_v(0) << endl;
    // cout << "e_v2: " << e_v(1) << endl;
    // cout << "e_r1: " << e_r(0) << endl;
    // cout << "e_r2: " << e_r(1) << endl;
    // cout << "e_theta1: " << e_theta(0) << endl;
    // cout << "e_theta2: " << e_theta(1) << endl;
    // cout << "\n" << endl;
    // cout.flush();
}


Matrix2X2d NonLinearController::calculate_closest_links_forces(BoomBoatsDuo duo) {
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

// Private function – Calculate du and dv for each boat (Cancelling drag and link forces)
Matrix2X2d NonLinearController::calculate_du_dv(BoomBoatsDuo duo) {
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
    double w1 = boat1_vel_frame(2);
    double u2 = boat2_vel_frame(0);
    double w2 = boat2_vel_frame(2);

    // Calculate the canceling force for each boat
    double mu_u1 = duo.get_boat1().get_mu_l();
    double mu_u2 = duo.get_boat2().get_mu_l();
    double mu_w1 = duo.get_boat1().get_mu_r();
    double mu_w2 = duo.get_boat2().get_mu_r();

    double r1 = duo.get_boat1().get_radius();
    double r2 = duo.get_boat2().get_radius();


    // Calculate du and dv for each boat (Cancelling drag and link forces)
    double du1 = -f_u1 + mu_u1 * u1 * abs(u1);
    double dv1 = f_v1 + mu_w1 * w1 * abs(w1) / r1;

    double du2 = -f_u2 + mu_u2 * u2 * abs(u2);
    double dv2 = f_v2 + mu_w2 * w2 * abs(w2) / r2;

    // Return the calculated du and dv values
    Matrix2X2d du_dv;
    du_dv(0, 0) = du1;
    du_dv(1, 0) = dv1;
    du_dv(0, 1) = du2;
    du_dv(1, 1) = dv2;

    return du_dv;
}

// Private function – Calculate the control signals after calculating du and dv and the control signals
Matrix2X2d NonLinearController::calculate_control_signals(Matrix2X2d a, Matrix2X2d d) {
    // Extract the control signals
    double au1 = a(0, 0);
    double av1 = a(1, 0);
    double au2 = a(0, 1);
    double av2 = a(1, 1);

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
        eta2_neg = atan2((av2 + dv2) / F2_neg, (av2 + du2) / F2_neg);
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

    // print f1 and f2
    // cout << "F1: " << F1 << endl;
    // cout << "F2: " << F2 << endl;
    // cout.flush();

    return control_signals;
}

// Calculate the au and av values
Matrix2X2d NonLinearController::calculate_au_av(BoomBoatsDuo duo,
     Vector3d setpoint1_dot, Vector3d setpoint2_dot) {

    double m1 = duo.get_boat1().get_mass();
    double I1 = duo.get_boat1().get_inertia();
    double r1 = duo.get_boat1().get_radius();
    double m2 = duo.get_boat2().get_mass();
    double I2 = duo.get_boat2().get_inertia();
    double r2 = duo.get_boat2().get_radius();

    // // Calculate k_theta
    // Vector2d k_theta;
    // Vector2d denom;
    // // Get theta_ref_dot
    // Vector2d theta_ref_dot;
    // theta_ref_dot(0) = setpoint1_dot(2);
    // theta_ref_dot(1) = setpoint2_dot(2);
    // for (int i = 0; i < 2; ++i) {
    //     if (this->e_theta(i) * this->e_theta(i) < this->epsilon) {
    //         denom(i) = this->epsilon;
    //     } else {
    //         denom(i) = this->e_theta(i) * this->e_theta(i);
    //     }
    //     // k_theta = delta * (1/denom) * (e_theta * theta_ref_dot + e_r)
    //     if ((this->e_theta(i) * theta_ref_dot(i) + this->e_r(i)) < 0)
    //         k_theta(i) = this->default_k_theta;
    //     else {
    //     k_theta(i) = this->delta * (1 / denom(i)) * (this->e_theta(i) * theta_ref_dot(i) + this->e_r(i));
    //     }
    // }

    // // Calculate k_u
    // Vector2d k_u;
    // for (int i = 0; i < 2; ++i) {
    //     if (this->e_u(i) * this->e_u(i) < this->epsilon) {
    //         denom(i) = this->epsilon;
    //     } else {
    //         denom(i) = this->e_u(i) * this->e_u(i);
    //     }
    //     // k_u = gamma * (1/denom) * e_v * r * k_theta / (m * I) * int_e_theta
    //     if ((this->e_v(i) * r1 * k_theta(i) / (m1 * I1) * this->int_e_theta(i)) < 0)
    //         k_u(i) = this->default_k_u;
    //     else {
    //     k_u(i) = this->gamma * (1 / denom(i)) * this->e_v(i) * r1 * k_theta(i) / (m1 * I1) * this->int_e_theta(i);
    //     }
    // }

    

    // // print e_theta, setpoint1_dot(2), e_r, e_u, denom
    // cout << "e_theta1: " << e_theta(0) << endl;
    // cout << "e_theta2: " << e_theta(1) << endl;
    // cout << endl;
    // cout << "setpoint1_dot(2): " << setpoint1_dot(2) << endl;
    // cout << "setpoint2_dot(2): " << setpoint2_dot(2) << endl;
    // cout << endl;
    // cout << "e_r1: " << e_r(0) << endl;
    // cout << "e_r2: " << e_r(1) << endl;
    // cout << endl;
    // cout << "e_u1: " << e_u(0) << endl;
    // cout << "e_u2: " << e_u(1) << endl;
    // cout << endl;
    // cout << "denom1: " << denom(0) << endl;
    // cout << "denom2: " << denom(1) << endl;
    // cout << endl;
    // cout << "--------------------------------" << endl;
    // cout.flush();

    // double mu_v1 = duo.get_boat1().get_mu_ct();
    // double mu_w1 = duo.get_boat1().get_mu_r();
    // double mu_v2 = duo.get_boat2().get_mu_ct();
    // double mu_w2 = duo.get_boat2().get_mu_r();
    // double omega1 = duo.get_boat1().get_frame_vel()(2);
    // double omega2 = duo.get_boat2().get_frame_vel()(2);
    double v1 = duo.get_boat1().get_frame_vel()(1);
    double v2 = duo.get_boat2().get_frame_vel()(1);
    Vector2d v = Vector2d(v1, v2);

    Matrix2X2d au_av; // First column is for boat 1 and second column is for boat 2
    // a_u = -k_u * m * e_u
    au_av(0, 0) = -this->k_u * m1 * e_u(0);
    au_av(0, 1) = -this->k_u * m2 * e_u(1);
    
    // au_av(0, 0) = 1000;
    // au_av(0, 1) = 1000;

    // a_v = -k_theta * (I / r) * e_theta - (I / r) * theta_ref_dot  + (I / r) * gamma * (v * e_v + e_r)
    // Calculate gamma
    // if (v * e_v + e_r < 0) then gamma = (1/e_theta) * (1 + k_v), else gamma = (1/e_theta) * (1 - k_v)
    Vector2d gamma;
    Vector2d denom; // denom is e_theta

    // for (int i = 0; i < 2; i++) {
    //     if (abs(this->e_theta(i)) < this->epsilon) {
    //         if (this->e_theta(i) == 0) {
    //             denom(i) = 1 / this->epsilon;
    //         } else {
    //             denom(i) = this->epsilon * sign(this->e_theta(i));
    //         }
    //     } else {
    //         denom(i) = this->e_theta(i);
    //     }

    //     if ((v(i) * this->e_v(i) + this->e_r(i)) <= 0) {
    //         gamma(i) = (1 / denom(i)) * (1 - this->k_v);
    //         // gamma(i) = 0;
    //     } else {
    //         // Print v(i) * this->e_v(i) + this->e_r(i)) and e_theta(i)
    //         cout << "v(i) * this->e_v(i) + this->e_r(i): " << v(i) * this->e_v(i) + this->e_r(i) << endl;
    //         cout << "e_theta(i): " << this->e_theta(i) << endl;
    //         gamma(i) = (1 / denom(i)) * (1 + this->k_v);
    //     }
    // }

    for (int i = 0; i < 2; i++) {
        if ((v(i) * this->e_v(i) + this->e_r(i)) < 0) {
            gamma(i) = -(1 / this->epsilon) * this->e_theta(i) * this->k_v;
        } else if ((v(i) * this->e_v(i) + this->e_r(i)) > 0) {
            gamma(i) = (1 / this->epsilon) * this->e_theta(i) * this->k_v;
        } else {
            gamma(i) = 0;
        }
    }
    
    // // print gamma
    // cout << "gamma1: " << gamma(0) << endl;  
    // cout << "gamma2: " << gamma(1) << endl;
    // cout.flush();

    au_av(1, 0) = (I1 / r1) * (-this->k_theta * this->e_theta(0) - setpoint1_dot(2) + gamma(0) * (v1 * this->e_v(0) + e_r(0)));
    au_av(1, 1) = (I2 / r2) * (-this->k_theta * this->e_theta(1) - setpoint2_dot(2) + gamma(1) * (v2 * this->e_v(1) + e_r(1)));
    
    // // print a_u1 and a_u2
    // cout << "au1: " << au_av(0, 0) << endl;
    // cout << "au2: " << au_av(0, 1) << endl;
    // cout << endl;
    // cout.flush();

    // // // print a_v1 and a_v2
    // cout << "av1: " << au_av(1, 0) << endl;
    // cout << "av2: " << au_av(1, 1) << endl;
    // cout << endl;
    // cout << endl;
    // cout.flush();

    return au_av;
}

// Calculate the control signals and update the controller
Matrix2X2d NonLinearController::update(BoomBoatsDuo duo, Vector3d setpoint1,
     Vector3d setpoint2, Vector3d setpoint1_dot, Vector3d setpoint2_dot) {

    // Update the errors and step before calculating du and dv
    update_errors(duo, setpoint1, setpoint2, setpoint1_dot, setpoint2_dot); 

    // Calculate du and dv for each boat (Cancelling drag and link forces)
    Matrix2X2d du_dv = calculate_du_dv(duo);
    
    Matrix2X2d au_av = calculate_au_av(duo, setpoint1_dot, setpoint2_dot);

    // Calculate the control signals after calculating du and dv
    Matrix2X2d control_signals = calculate_control_signals(au_av, du_dv);

    // Update the control signals
    output = control_signals;

    // Return the control signals
    return control_signals;

}