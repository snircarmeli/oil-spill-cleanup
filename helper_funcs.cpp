#include "helper_funcs.h"

// Wrap theta between -pi and pi
float wrap_theta(float theta) {
    theta = fmod(theta, 2 * PI); // Normalize theta within [-2PI, 2PI]
    if (theta > PI) {
        theta -= 2 * PI; // Adjust if theta is in (PI, 2PI]
    } else if (theta < -PI) {
        theta += 2 * PI; // Adjust if theta is in [-2PI, -PI)
    }
    // cout << theta << endl;
    return theta;
}

// Sign function
int sign(float x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

// Rotation matrix
Matrix2f rot_mat(float theta) {
    Matrix2f R;
    R << cos(theta), -sin(theta),
         sin(theta), cos(theta);
    return R;
}

// Check if point k is on segment ij
bool on_segment(float xi, float yi, float xj, float yj, float xk, float yk) {
    return min(xi, xj) <= xk && xk <= max(xi, xj) && min(yi, yj) <= yk && yk <= max(yi, yj);
}

// Check if two line segments intersect
bool check_intersection(float x_11, float y_11, float x_12, float y_12,
 float x_21, float y_21, float x_22, float y_22, float L) {
    // normalized direction vectors
    Vector2f m1 = Vector2f(x_12 - x_11, y_12 - y_11).normalized();
    Vector2f m2 = Vector2f(x_22 - x_21, y_22 - y_21).normalized();

    // Check if the two segments are parallel
    if (m1 == m2 || m1 == -m2) {
        // check if either point on the second segment is on the first segment
        // check if (x11, y11) is on the second segment
        float s1 = (x_11 - x_21) / m2.x();
        float s2 = (y_11 - y_21) / m2.y();
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
    Matrix2f A;
    A << m1.x(), -m2.x(), m1.y(), -m2.y();
    Vector2f b = Vector2f(x_21 - x_11, y_21 - y_11);
    // s = inv(A) * b
    Vector2f s = A.inverse() * b;
    // Check if the intersection point is on both segments
    if (s.x() >= 0 && s.x() <= L && s.y() >= 0 && s.y() <= L) {
        return true;
    }
    return false;
}

bool canConvertToFloat(const string &s) {
    std::istringstream iss(s);
    float f;
    // noskipws ensures that whitespace is not ignored.
    iss >> std::noskipws >> f;
    // Check that we have reached the end of the stream and no error occurred.
    return iss.eof() && !iss.fail();
}