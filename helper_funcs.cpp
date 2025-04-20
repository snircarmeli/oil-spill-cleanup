#include "helper_funcs.h"

// Wrap theta between -pi and pi
double wrap_theta(double theta) {
    theta = fmod(theta, 2 * PI); // Normalize theta within [-2PI, 2PI]
    if (theta > PI) {
        theta -= 2 * PI; // Adjust if theta is in (PI, 2PI]
    } else if (theta < -PI) {
        theta += 2 * PI; // Adjust if theta is in [-2PI, -PI)
    }
    // cout << theta << endl;
    return theta;
}

// wrap eta between -pi / 2 and pi / 2
double wrap_eta(double eta) {
    eta = fmod(eta, PI); // Normalize eta within [-PI, PI]
    if (eta > PI / 2) {
        eta -= PI; // Adjust if eta is in (PI / 2, PI]
    } else if (eta < -PI / 2) {
        eta += PI; // Adjust if eta is in [-PI, -PI / 2)
    }
    return eta;
}

// Sign function
int sign(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

// Rotation matrix
Matrix2d rot_mat(double theta) {
    Matrix2d R;
    R << cos(theta), -sin(theta),
         sin(theta), cos(theta);
    return R;
}

// Check if point k is on segment ij
bool on_segment(double xi, double yi, double xj, double yj, double xk, double yk) {
    return min(xi, xj) <= xk && xk <= max(xi, xj) && min(yi, yj) <= yk && yk <= max(yi, yj);
}

// Check if two line segments intersect
bool check_intersection(double x_11, double y_11, double x_12, double y_12,
 double x_21, double y_21, double x_22, double y_22, double L) {
    // normalized direction vectors
    Vector2d m1 = Vector2d(x_12 - x_11, y_12 - y_11).normalized();
    Vector2d m2 = Vector2d(x_22 - x_21, y_22 - y_21).normalized();

    // Check if the two segments are parallel
    if (m1 == m2 || m1 == -m2) {
        // check if either point on the second segment is on the first segment
        // check if (x11, y11) is on the second segment
        double s1 = (x_11 - x_21) / m2.x();
        double s2 = (y_11 - y_21) / m2.y();
        if (s1 == s2 && s1 >= 0 && s1 <= L) {
            // cout << "Two segments parallel" << endl;
            // cout.flush();
            return true;
        }
        // check if (x12, y12) is on the second segment
        s1 = (x_12 - x_21) / m2.x();
        s2 = (y_12 - y_21) / m2.y();
        if (s1 == s2 && s1 >= 0 && s1 <= L) {
            // cout << "Two segments parallel" << endl;
            // cout.flush();
            return true;
        }
    }
    // If not, the determinant of the matrix A should be non-zero
    Matrix2d A;
    A << m1.x(), -m2.x(), m1.y(), -m2.y();
    Vector2d b = Vector2d(x_21 - x_11, y_21 - y_11);
    // s = inv(A) * b
    Vector2d s = A.inverse() * b;
    // Check if the intersection point is on both segments
    if (s.x() >= 0 && s.x() <= L && s.y() >= 0 && s.y() <= L) {
        return true;
    }
    return false;
}

bool canConvertToFloat(const string &s) {
    std::istringstream iss(s);
    double f;
    // noskipws ensures that whitespace is not ignored.
    iss >> std::noskipws >> f;
    // Check that we have reached the end of the stream and no error occurred.
    return iss.eof() && !iss.fail();
}

// Function which calculate the derivative of the path in the global frame
MatrixXd path_der_global(MatrixXd path_points, double ts) {
    MatrixXd path_der = MatrixXd::Zero(3, path_points.cols());
    Vector3d point1, point2;

    // Calculate for i = 0
    point1 << path_points(0, 0), path_points(1, 0), path_points(2, 0);
    point2 << path_points(0, 1), path_points(1, 1), path_points(2, 1);
    path_der.col(0) = (point2 - point1) / ts;

    // Calculate for i = 1 to n - 2
    for (int i = 1; i < path_points.cols() - 1; i++) {
        point1 << path_points(0, i - 1), path_points(1, i - 1), path_points(2, i - 1);
        point2 << path_points(0, i + 1), path_points(1, i + 1), path_points(2, i + 1);
        path_der.col(i) = (point2 - point1) / (2 * ts);
    }

    // Calculate for i = n - 1
    point1 << path_points(0, path_points.cols() - 2), path_points(1, path_points.cols() - 2), path_points(2, path_points.cols() - 2);
    point2 << path_points(0, path_points.cols() - 1), path_points(1, path_points.cols() - 1), path_points(2, path_points.cols() - 1);
    path_der.col(path_points.cols() - 1) = (point2 - point1) / ts;

    // Wrap theta
    for (int i = 0; i < path_der.cols(); i++) {
        path_der(2, i) = wrap_theta(path_der(2, i));
    }

    return path_der;
}

// Function which checks if jumps between two points are too big and interpolates them
MatrixXd check_path(MatrixXd path_points, double max_jump) {
    // Check if path_points is empty
    if (path_points.size() == 0) {
        return path_points;
    }
    int n = path_points.cols();
    // Initialize the matrix to store the new path points
    MatrixXd new_path_points(3, n);
    new_path_points = path_points;

    for (int i = 0; i < n - 1; i++) {
        // Check if the jump between two points is too big
        if ((new_path_points.col(i + 1) - new_path_points.col(i)).norm() > max_jump) {
            // cout << "Jump between points " << i << " and " << i + 1 << " is too big" << endl;
            // cout.flush();
            // The point i+1 is too far, we will use point i and point i+2 to interpolate
            // Check if i+2 is out of bounds
            if (i + 2 < n) {
                // Interpolate between point i and point i+2
                Vector3d point1 = new_path_points.col(i);
                Vector3d point2 = new_path_points.col(i + 2);
                Vector3d new_point = (point1 + point2) / 2;
                // Print point i, point i+1, point i+2
                // cout << "Point i: " << new_path_points.col(i).transpose() << endl;
                // cout << "Point i+1: " << new_path_points.col(i+1).transpose() << endl;
                // cout << "Point i+2: " << new_path_points.col(i + 2).transpose() << endl;
                // cout << endl;
                // cout << "New point: " << new_point.transpose() << endl;
                // cout << endl;
                // cout << endl;
                // cout.flush();
                new_point(2) = wrap_theta(new_point(2)); // Wrap theta
                new_path_points.col(i + 1) = new_point;

            } else {
                // If i+2 is out of bounds, just set the point to the last one
                new_path_points.col(i + 1) = new_path_points.col(i);
            }
        }
    }
    return new_path_points;
}

// Function that calculates the convex hull of a set of points
MatrixXd calculate_convex_hull(const MatrixXd &points) {
    // Check if the points matrix is empty
    if (points.rows() == 0) {
        throw std::runtime_error("Empty points matrix at convex_hull");
    }
    int n = points.rows();
    // If there is 0-2 points, throw an error
    if (n <= 2) {
        throw std::runtime_error("Less than 3 points in the points matrix at convex_hull");
    }
    // If there are 3 points, return the points matrix
    if (n == 3) {
        return points;
    }

    MatrixXd pts(n, 2);
    // Copy points to pts
    pts = points;

    // Sort points lexicographically (first by x, then by y)
    VectorXi indices = VectorXi::LinSpaced(n, 0, n - 1);
    sort(indices.data(), indices.data() + indices.size(), [&](int a, int b) {
        return (pts(a, 0) < pts(b, 0)) || ((pts(a, 0) == pts(b, 0)) && (pts(a, 1) < pts(b, 1)));
    });

    // Lambda to compute the cross product of OA and OB
    auto cross = [&](int O, int A, int B) -> double {
        return (pts(A, 0) - pts(O, 0)) * (pts(B, 1) - pts(O, 1)) - (pts(A, 1) - pts(O, 1)) * (pts(B, 0) - pts(O, 0));
    };

    // Build the lower hull
    VectorXi lower(n);
    int lower_size = 0;
    for (int i = 0; i < n; i++) {
        int idx = indices(i);
        while (lower_size >= 2 && cross(lower(lower_size - 2), lower(lower_size - 1), idx) <= 0) {
            lower_size--;
        }
        lower(lower_size++) = idx;
    }

    // Build the upper hull
    VectorXi upper(n);
    int upper_size = 0;
    for (int i = n - 1; i >= 0; i--) {
        int idx = indices(i);
        while (upper_size >= 2 && cross(upper(upper_size - 2), upper(upper_size - 1), idx) <= 0) {
            upper_size--;
        }
        upper(upper_size++) = idx;
    }

    // Remove the last point of each half because it's repeated at the beginning of the other half
    if (lower_size > 0) lower_size--;
    if (upper_size > 0) upper_size--;

    // Concatenate lower and upper hull to get the full hull
    VectorXi hull(lower_size + upper_size);
    hull << lower.head(lower_size), upper.head(upper_size);

    // Convert the hull indices back to a MatrixXd
    MatrixXd hull_matrix(hull.size(), 2);
    for (int i = 0; i < hull.size(); i++) {
        hull_matrix(i, 0) = pts(hull(i), 0);
        hull_matrix(i, 1) = pts(hull(i), 1);
    }

    return hull_matrix;
}

// Helper function for calculating distance between point and segment
double point_to_segment_distance(const Vector2d &p, const Vector2d &a, const Vector2d &b) {
    Vector2d ab = b - a;
    Vector2d ap = p - a;
    double t = ap.dot(ab) / ab.squaredNorm();
    t = std::max(0.0, std::min(1.0, t));
    Vector2d closest = a + t * ab;
    return (p - closest).norm();
}

// Compute minimal distance between two convex hulls using rotating calipers
// Both hulls are counter-clockwise and 2D (Nx2 matrices)
double calculate_convex_hull_distance(const MatrixXd &hull1, const MatrixXd &hull2) {
    int n = hull1.rows(), m = hull2.rows();
    if (n < 2 || m < 2) return numeric_limits<double>::infinity();

    int i = 0, j = 0;
    // Find the lowest point in hull1 and the highest in hull2
    for (int k = 1; k < n; ++k)
        if (hull1(k, 1) < hull1(i, 1)) i = k;
    for (int k = 1; k < m; ++k)
        if (hull2(k, 1) > hull2(j, 1)) j = k;

    double min_dist = numeric_limits<double>::infinity();
    int start_i = i, start_j = j;

    do {
        Vector2d a1 = hull1.row(i);
        Vector2d a2 = hull1.row((i + 1) % n);
        Vector2d b1 = hull2.row(j);
        Vector2d b2 = hull2.row((j + 1) % m);

        Vector2d edge1 = a2 - a1;
        Vector2d edge2 = b2 - b1;

        // Use cross product to determine rotation
        double cross = edge1.x() * edge2.y() - edge1.y() * edge2.x();

        // Advance the caliper with the smaller angle
        if (cross <= 0)
            i = (i + 1) % n;
        else
            j = (j + 1) % m;

        // Compute distances
        min_dist = min(min_dist, point_to_segment_distance(a1, b1, b2));
        min_dist = min(min_dist, point_to_segment_distance(b1, a1, a2));

    } while (i != start_i || j != start_j);

    return min_dist;
}


// MatrixXd glob_path_points_2_local_frame_vel(MatrixXd path_points, double ts) {
//     // This function returns a path in local frame, first number is u,
//     // second number is v (zero bassicaly), third number is orientation
    
//     // Check if path_points is empty
//     if (path_points.size() == 0) {
//         return path_points;
//     }
//     int n = path_points.cols();
//     // Initialize the matrix to store the local frame velocities
//     MatrixXd path_points_local_frame_vel = MatrixXd::Zero(3, n);

//     double curr_theta;
//     Vector2d curr_dir;
//     Vector2d point1;
//     Vector2d point2;

//     for (int i = 0; i < n; i++) {
//         if (i == 0) {
//             point1 << path_points(0, i), path_points(1, i);
//             point2 << path_points(0, i + 1), path_points(1, i + 1);
//             // u
//             path_points_local_frame_vel(0, i) = (point2 - point1).norm() / ts;
//         } else if (i == n - 1) {
//             point1 << path_points(0, i - 2), path_points(1, i - 2);
//             point2 << path_points(0, i - 1), path_points(1, i - 1);
//             // u
//             path_points_local_frame_vel(0, i) = (point2 - point1).norm() / ts;
//         }
//         else {
//             point1 << path_points(0, i - 1), path_points(1, i - 1);
//             point2 << path_points(0, i + 1), path_points(1, i + 1);
//             // u
//             path_points_local_frame_vel(0, i) = (point2 - point1).norm() / (2 * ts);
//         }
//         // if u < epsilon in absolute value, set it to 0
//         if (abs(path_points_local_frame_vel(0, i)) < 0.0001) {
//             path_points_local_frame_vel(0, i) = 0.0;
//         }

//         // Print u
//         // cout << "u: " << path_points_local_frame_vel(0, i) << endl;
//         // cout.flush();

//         // v
//         path_points_local_frame_vel(1, i) = 0.0;

//         // orientation
//         curr_dir = point2 - point1;
//         path_points_local_frame_vel(2, i) = wrap_theta(atan2(curr_dir.y(), curr_dir.x()));
//         // Correct the orientation to boat frame
//         path_points_local_frame_vel(2, i) = PI / 2 - path_points_local_frame_vel(2, i);
//     }
//     return path_points_local_frame_vel;
// }