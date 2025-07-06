#ifndef MILP_ALLOCATOR_H
#define MILP_ALLOCATOR_H

#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>

#include "boom-boats-duo.h"
#include "oil-spill.h"
#include "obstacle.h"
#include "helper_funcs.h"

#include <gurobi_c++.h>
#include "json/json.hpp"

using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::ifstream;
using json = nlohmann::json;

using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::VectorXd;
using Eigen::Vector2d;

vector<MatrixXi> allocate_duos(const vector<BoomBoatsDuo>& duos,
                                 const vector<OilSpill>&     spills,
                                 const vector<Obstacle>&     obstacles);

#endif // MILP_ALLOCATOR_H