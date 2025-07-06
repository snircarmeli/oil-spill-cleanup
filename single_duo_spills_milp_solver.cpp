// spill_milp_solver.cpp  --- single‑duo weighted‑latency MILP with single‑commodity‑flow subtour elimination
// -----------------------------------------------------------------------------
//  * solver  : Google OR‑Tools MIP (CBC backend by default – change to SCIP/Gurobi
//              by setting the solver name below).
//  * license : Apache 2.0 (same as OR‑Tools).
//  * compile :
//      g++ -std=c++17 -I/usr/include/ortools \
//          spill_milp_solver.cpp -lortools -O2 -o spill_milp_solver
//  * run     : ./spill_milp_solver <instance.json >solution.txt
//
//  Input format (JSON) ---------------------------------------------------------
//  {
//    "u"      : <double>,              // boat speed (km / h)
//    "alpha"  : <double>,              // travel‑time trade‑off weight
//    "dist"   : [[0,d01,...],[d10,0,...], ...], // (n+1)x(n+1) matrix inc. depot 0
//    "volume" : [V1, V2, ... , Vn],    // spill volumes (m^3)
//    "risk"   : [R1, R2, ... , Rn],    // risk weights
//    "suction_rate" : <double>         // S·R  (m^3 / h)
//  }
// -----------------------------------------------------------------------------

#include <iostream>
#include <vector>
#include <limits>
#include "ortools/linear_solver/linear_solver.h"

// Parsing json parameters
#include "json/json.hpp"
using json = nlohmann::json;

using operations_research::MPSolver;
using operations_research::MPVariable;
using operations_research::MPConstraint;
using operations_research::MPObjective;

struct Instance {
  int n;                         // number of spills (nodes 1..n)
  double u;                      // speed
  double alpha;                  // travel weight
  std::vector<std::vector<double>> d; // (n+1)x(n+1) distances (0 = depot)
  std::vector<double> V;         // volumes size n
  std::vector<double> R;         // risks   size n
  double SR;                     // suction rate S·R
};

Instance ReadInstance(std::istream& in) {
  json j; in >> j;
  Instance inst;
  inst.u     = j.at("u").get<double>();
  inst.alpha = j.at("alpha").get<double>();
  inst.d     = j.at("dist").get<std::vector<std::vector<double>>>();
  inst.V     = j.at("volume").get<std::vector<double>>();
  inst.R     = j.at("risk").get<std::vector<double>>();
  inst.SR    = j.at("suction_rate").get<double>();
  inst.n     = static_cast<int>(inst.V.size());
  // quick checks
  if (inst.d.size() != inst.n + 1) throw std::runtime_error("distance dim mismatch");
  return inst;
}

void Solve(const Instance& ins) {
  const int n = ins.n;
  const int N = n + 1; // include depot 0

  MPSolver solver("spill_milp", MPSolver::CBC_MIXED_INTEGER_PROGRAMMING);
  const double INF = solver.infinity();

  // helper lambdas for indexing
  auto idx = [&](int i, int j) { return i*N + j; };

  // ------------------ variables ------------------
  std::vector<MPVariable*> x(N*N, nullptr);      // binary routing vars
  std::vector<MPVariable*> f(N*N, nullptr);      // flow vars
  std::vector<MPVariable*> T(n+1, nullptr);      // arrival times (index 0 unused)
  MPVariable* L = solver.MakeNumVar(0, INF, "L");

  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) if (i != j) {
      x[idx(i,j)] = solver.MakeIntVar(0, 1, "x("+std::to_string(i)+","+std::to_string(j)+")");
      f[idx(i,j)] = solver.MakeNumVar(0, n-1, "f("+std::to_string(i)+","+std::to_string(j)+")");
    }
  }
  for (int i = 1; i <= n; ++i)
    T[i] = solver.MakeNumVar(0, INF, "T("+std::to_string(i)+")");

  // ------------------ constraints ------------------
  // (1) + (2) depot out / in degree
  MPConstraint* c_out_depot = solver.MakeRowConstraint(1, 1, "out_depot");
  MPConstraint* c_in_depot  = solver.MakeRowConstraint(1, 1, "in_depot");
  for (int j = 1; j <= n; ++j) {
    c_out_depot->SetCoefficient(x[idx(0,j)], 1);
    c_in_depot ->SetCoefficient(x[idx(j,0)], 1);
  }

  // (3) + (4) spill degree =1
  for (int i = 1; i <= n; ++i) {
    MPConstraint* out = solver.MakeRowConstraint(1, 1, "out("+std::to_string(i)+")");
    MPConstraint* in  = solver.MakeRowConstraint(1, 1, "in("+std::to_string(i)+")");
    for (int j = 0; j <= n; ++j) if (i != j) {
      out->SetCoefficient(x[idx(i,j)], 1);
      in ->SetCoefficient(x[idx(j,i)], 1);
    }
  }

  // (5) link L
  MPConstraint* linkL = solver.MakeRowConstraint(0, 0, "linkL");
  linkL->SetCoefficient(L, -ins.u); // bring L to lhs times speed
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) if (i!=j) {
      linkL->SetCoefficient(x[idx(i,j)], ins.d[i][j]);
    }
  }

  // big-M
  double max_d = 0;
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j)
      if (i!=j) max_d = std::max(max_d, ins.d[i][j]);
  double Mbig = max_d/ins.u;
  double sum_s = 0;
  for (double V: ins.V) sum_s += V/ins.SR;
  Mbig += sum_s;

  // (6) time propagation
  for (int i = 0; i <= n; ++i) {
    for (int j = 1; j <= n; ++j) if (i != j) {
      MPConstraint* timeProp = solver.MakeRowConstraint(-INF, Mbig, "time("+std::to_string(i)+","+std::to_string(j)+")");
      if (i != 0) timeProp->SetCoefficient(T[i], 1); // +T_i
      timeProp->SetCoefficient(T[j], -1);            // -T_j
      double travel = ins.d[i][j]/ins.u + (i==0?0: ins.V[i-1]/ins.SR);
      timeProp->SetConstant(travel);
      timeProp->SetCoefficient(x[idx(i,j)], Mbig);
    }
  }

  // (7) initialize lower bounds T_i >= d0i/u
  for (int i = 1; i <= n; ++i) {
    MPConstraint* initT = solver.MakeRowConstraint(-INF, 0, "initT("+std::to_string(i)+")");
    initT->SetCoefficient(T[i], -1);
    initT->SetConstant(-ins.d[0][i]/ins.u);
  }

  // ------------------ flow constraints ------------------
  // (F1) f_ij <= (n-1)*x_ij already embedded via variable upper bound  (but we assert 0-(n-1))
  // (F2) depot outflow = n-1
  MPConstraint* depotFlow = solver.MakeRowConstraint(n-1, n-1, "depotFlow");
  for (int j = 1; j <= n; ++j)
    depotFlow->SetCoefficient(f[idx(0,j)], 1);
  // (F3) depot inflow = 0  (optional)
  MPConstraint* depotIn  = solver.MakeRowConstraint(0, 0, "depotIn");
  for (int i = 1; i <= n; ++i)
    depotIn->SetCoefficient(f[idx(i,0)], 1);
  // (F4) balance at each spill
  for (int k = 1; k <= n; ++k) {
    MPConstraint* bal = solver.MakeRowConstraint(1, 1, "bal("+std::to_string(k)+")");
    for (int i = 0; i <= n; ++i) if (i!=k) bal->SetCoefficient(f[idx(i,k)], 1);
    for (int j = 0; j <= n; ++j) if (j!=k) bal->SetCoefficient(f[idx(k,j)], -1);
  }

  // ------------------ objective ------------------
  MPObjective* obj = solver.MutableObjective();
  for (int i = 1; i <= n; ++i)
    obj->SetCoefficient(T[i], ins.R[i-1]);
  obj->SetCoefficient(L, ins.alpha);
  obj->SetMinimization();

  // ------------------ solve ------------------
  auto status = solver.Solve();
  if (status != MPSolver::OPTIMAL && status != MPSolver::FEASIBLE) {
    std::cerr << "No feasible solution found.\n";
    return;
  }
  std::cout << "Objective: " << obj->Value() << "\n";
  // retrieve tour
  int current = 0;
  std::vector<int> tour; tour.reserve(n+1);
  std::vector<bool> visited(N,false);
  while (true) {
    visited[current] = true;
    int next = -1;
    for (int j = 0; j < N; ++j) if (current!=j && x[idx(current,j)] && x[idx(current,j)]->solution_value() > 0.5) { next = j; break; }
    if (next == -1) break;
    tour.push_back(next);
    if (next == 0) break; // returned to depot
    current = next;
  }
  std::cout << "Tour (0‑based, depot=0): 0 ";
  for (int node: tour) std::cout << "-> " << node;
  std::cout << "\n";
  for (int i = 1; i <= n; ++i)
    std::cout << "T["<<i<<"]="<<T[i]->solution_value()<<"\n";
}

int main() {
  try {
    Instance ins = ReadInstance(std::cin);
    Solve(ins);
  } catch(const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
  return 0;
}
