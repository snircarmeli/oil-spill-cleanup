// MILP_Allocator.cpp  — allocate_duos()
// -----------------------------------------------------------
// ⓒ 2025   Snir Carmeli – Technion, EE  |  Uses Gurobi ≥10.0
// -----------------------------------------------------------

#include "MILP_Allocator.h"

// ────────────────────────────────────────────────────────────
vector<MatrixXi> allocate_duos(const vector<BoomBoatsDuo>& duos,
                       const vector<OilSpill>&     spills,
                       const vector<Obstacle>&     obstacles)
{
    // 0.  PARAMETERS  ────────────────────────────────────────
    json params;
    ifstream file("params.json");
    if (!file.is_open())
        throw std::runtime_error("Could not open params.json");
    file >> params;
    file.close();

    const json   MILP_params = params["MILP_Solver"];
    const int    n           = spills.size() + 1;       // depot+spills
    const int    m           = duos .size();            // agents
    const double u           = MILP_params["velocity"]; // boat speed
    const double SR          = MILP_params["SR"];       // suction rate

    // Print all parameters
    cout << "Parameters:" << endl;
    cout << "  Number of duos: " << m << endl;
    cout << "  Number of spills: " << n-1 << endl;
    cout << "  Velocity: " << u << " m/s" << endl;
    cout << "  Suction rate: " << SR << " m³/s" << endl;

    // 1.  PRE-PROCESSING  (distances, times, risk, Big-M) ────
    MatrixXd distances   = MatrixXd::Zero(n, n);
    MatrixXd travel_time = MatrixXd::Zero(n, n);
    VectorXd service_time(n);  service_time.setZero();
    VectorXd Risk        (n);  Risk.setZero();

    // centroid of all duos = depot
    Vector2d depot = Vector2d::Zero();
    for (const auto& d : duos) depot += d.get_center();
    depot /= m;

    /* fill distance (upper triangle)/time matrices and spill info */
    for (int i = 1; i < n; i++) {
        service_time(i) = spills[i-1].get_volume() / SR;
        Risk(i) = spills[i-1].get_risk_factor(obstacles);
        MatrixXd hull_i = spills[i-1].get_convex_hull();
        for (int j = i + 1; j < n; j++) {
            MatrixXd hull_j = spills[j-1].get_convex_hull();
            distances(i, j) = calculate_convex_hull_distance(hull_i, hull_j);
            travel_time(i,j) = distances(i,j) / u;
        }
    }

    // fill lower triangle of distance matrix
    for (int i = 1; i < n; i++)
        for (int j = 0; j < i; j++)
            distances(j, i) = distances(i, j);

    // fill depot distances
    for (int j = 1; j < n; ++j) {
        MatrixXd hull_j = spills[j-1].get_convex_hull();
        distances(0, j) = calculate_point_convex_hull_distance(hull_j, depot);
        distances(j, 0) = distances(0, j); // symmetric
        travel_time(0,j) = distances(0,j) / u;
        travel_time(j,0) = travel_time(0,j); // symmetric
    }

    const double big_M = travel_time.maxCoeff() + service_time.sum();

    // 2.  GUROBI MODEL  ──────────────────────────────────────
    // cout << "Creating Gurobi model..." << endl; cout.flush();

    GRBEnv env(true);
    try {
        // ───── existing code up to env.start() … ─────
        env.set(GRB_IntParam_OutputFlag, 1);
        env.set(GRB_StringParam_LogFile, "gurobi.log");
        env.set(GRB_IntParam_Threads, 0);
        env.set(GRB_DoubleParam_TimeLimit, 3600);
        env.set(GRB_DoubleParam_MIPGap, 1e-4);
        env.start();                 // throws if licence/problem

        GRBModel model(env);

    }
    catch (const GRBException& e) {
        std::cerr << "Gurobi error " << e.getErrorCode()
                  << ": " << e.getMessage() << std::endl;
        return {};                   // or re-throw / propagate
    }
    catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
        return {};
    }


    // Create model with initialized environment
    GRBModel model(env);

    // cout << "Adding decision variables..." << endl; cout.flush();
    // 2.1 Decision variables – index order  x[i][j][k]  (i,j∈0..n-1, k∈0..m-1)
    vector<vector<vector<GRBVar>>> x(n,
        vector<vector<GRBVar>>(n, vector<GRBVar>(m)));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            for (int k = 0; k < m; ++k)
                x[i][j][k] = model.addVar(0,1,0,GRB_BINARY,
                    "x_"+std::to_string(i)+'_'+std::to_string(j)+'_'+std::to_string(k));

    // z[j][k]  spill-assignment
    vector<vector<GRBVar>> z(n, vector<GRBVar>(m));
    for (int j = 0; j < n; ++j)
        for (int k = 0; k < m; ++k)
            z[j][k] = model.addVar(0,1,0,GRB_BINARY,
                "z_"+std::to_string(j)+'_'+std::to_string(k));

    // arrival times T_j
    vector<GRBVar> T(n);
    for (int j = 0; j < n; ++j)
        T[j] = model.addVar(0,GRB_INFINITY,0,GRB_CONTINUOUS,
                "T_"+std::to_string(j));

    // flow variables  q[i][j][k] (only i≠j)
    vector<vector<vector<GRBVar>>> q(n,
        vector<vector<GRBVar>>(n, vector<GRBVar>(m)));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            if (i != j)
                for (int k = 0; k < m; ++k)
                    q[i][j][k] = model.addVar(0,n,0,GRB_INTEGER,
                        "q_"+std::to_string(i)+'_'+std::to_string(j)+'_'+std::to_string(k));


    // cout << "Setting constraints..." << endl; cout.flush();
    // 2.2  CONSTRAINTS  (same names as PDF) ──────────────────
    // C1: depot → exactly one first spill per k
    for (int k = 0; k < m; ++k) {
        GRBLinExpr e = 0;
        for (int j = 1; j < n; ++j) e += x[0][j][k];
        model.addConstr(e == 1, "C1_"+std::to_string(k));
    }
    // C2: exactly one return arc per k
    for (int k = 0; k < m; ++k) {
        GRBLinExpr e = 0;
        for (int i = 1; i < n; ++i) e += x[i][0][k];
        model.addConstr(e == 1, "C2_"+std::to_string(k));
    }
    // C3: each spill served by one agent
    for (int j = 1; j < n; ++j) {
        GRBLinExpr e = 0;
        for (int k = 0; k < m; ++k) e += z[j][k];
        model.addConstr(e == 1, "C3_"+std::to_string(j));
    }
    // C4: flow balance = assignment
    for (int j = 1; j < n; ++j) {
        for (int k = 0; k < m; ++k) {
            GRBLinExpr in=0,out=0;
            for (int i = 0; i < n; ++i) if (i!=j) in  += x[i][j][k];
            for (int h = 0; h < n; ++h) if (h!=j) out += x[j][h][k];
            model.addConstr(in  == z[j][k], "C4in_"+std::to_string(j)+'_'+std::to_string(k));
            model.addConstr(out == z[j][k], "C4out_"+std::to_string(j)+'_'+std::to_string(k));
        }
    }

    // C5: forbid self loops
    for (int i = 0; i < n; ++i)
        for (int k = 0; k < m; ++k)
            model.addConstr(x[i][i][k] == 0, "C5_"+std::to_string(i)+'_'+std::to_string(k));

    // TIME constraints (T1, T2)
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i!=j)
                for (int k = 0; k < m; ++k) {   
                    model.addConstr(
                        T[j] >= T[i] + travel_time(i,j) + service_time(i)
                                 - big_M*(1 - x[i][j][k]),
                        "T1_"+std::to_string(i)+'_'+std::to_string(j)+'_'+std::to_string(k));
                }
        }
        // depot time constraint
        for (int k = 0; k < m; ++k) {
            model.addConstr(T[i] >= travel_time(0,i) - big_M*(1 - x[0][i][k]),
                             "T2_"+std::to_string(i)+'_'+std::to_string(k));
        }
    }

    // // F1: flow conservation
    // for (int k = 0; k < m; ++k) {
    //     for (int i = 0; i < n; ++i) {
    //         for (int j = 0; j < n; ++j) {
    //             if (i != j) {
    //                 model.addConstr(
    //                     q[i][j][k] <= (n - 1) * x[i][j][k],
    //                     "F1a_"+std::to_string(i)+'_'+std::to_string(k)
    //                 );
    //                 model.addConstr(
    //                     q[i][j][k] >= 0,
    //                     "F1b_"+std::to_string(i)+'_'+std::to_string(k)
    //                 );
    //             }
    //         }
    //     }
    // }

    // // F2: Initial flow capacity
    // for (int k = 0; k < m; ++k) {
    //     GRBLinExpr e1 = 0;
    //     GRBLinExpr e2 = 0;
    //     for (int j = 1; j < n; ++j) {
    //         e1 += q[0][j][k]; // flow from depot to spills
    //         e2 += z[j][k]; // spills assigned to k
    //     }
    //     model.addConstr(e1 == e2, "F2_"+std::to_string(k));
    

    // // F3: Flow conservation
    // for (int k = 0; k < m; ++k) {
    //     for (int j = 0; j < n; ++j) {
    //         GRBLinExpr in = 0, out = 0;
    //         for (int i = 0; i < n; ++i) if (i != j) in += q[i][j][k];
    //         for (int h = 0; h < n; ++h) if (h != j) out += q[j][h][k];
    //         model.addConstr(in - out == z[j][k], "F3_"+std::to_string(j)+'_'+std::to_string(k));
    //     }
    // }

    // // F4: return to depot empty
    // for (int k = 0; k < m; ++k) {
    //     GRBLinExpr e = 0;
    //     for (int i = 1; i < n; ++i) e += q[i][0][k];
    //     model.addConstr(e == 0, "F4_"+std::to_string(k));
    // }

    // cout << "Setting objective function..." << endl; cout.flush();
    // Define objective function
    GRBLinExpr objective = 0;
    for (int j = 1; j < n; ++j) {
        objective += Risk(j) *T[j]; // minimize risk
    }
    model.setObjective(objective, GRB_MINIMIZE);

    // Add before model.optimize():
    model.set(GRB_IntParam_InfUnbdInfo, 1);
    
    
    // cout << "Optimizing model..." << endl; cout.flush();
    // 3.  OPTIMIZE  ──────────────────────────────────────────
    model.optimize();
    if (model.get(GRB_IntAttr_Status) != GRB_OPTIMAL) {
        cout << "No optimal solution found." << endl;
        return vector<MatrixXi>(); // return empty matrix
    }

    

    if (model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
        cout << "\nComputing IIS (Irreducible Inconsistent Subsystem)..." << endl;
        model.computeIIS();
        cout << "\nConstraints in the IIS:" << endl;
        for (int c = 0; c < model.get(GRB_IntAttr_NumConstrs); ++c) {
            GRBConstr constr = model.getConstr(c);
            if (constr.get(GRB_IntAttr_IISConstr)) {
                cout << constr.get(GRB_StringAttr_ConstrName) << endl;
            }
        }
    }

    // cout << "Extracting optimal solution..." << endl; cout.flush();
    // 4.  EXTRACT SOLUTION  ──────────────────────────────────
    // Extract m vectors of matrix x
    vector<MatrixXi> x_solutions(m, MatrixXi::Zero(n, n));
    for (int k = 0; k < m; ++k) {
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (x[i][j][k].get(GRB_DoubleAttr_X) > 0.5) {
                    x_solutions[k](i, j) = 1;
                }
            }
        }
    }

    // cout << "Returning optimal solutions..." << endl; cout.flush();
    return x_solutions;
}
