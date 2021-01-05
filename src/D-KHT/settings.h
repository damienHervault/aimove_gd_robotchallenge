#ifndef SETTINGS_H
#define SETTINGS_H


#include <limits>
#include <sat.h>
#include <libdlib/geometry/vector.h>
//#include <QDir>
#define NONZERO 0.00001

const double PI = acos(-1);
const double PI2 = 2.0 * PI;
static const double root22pi32 = 2.0 * sqrt(2.0) * pow(PI, 1.5);

typedef dlib::vector<double, 3> dvec3;
typedef dlib::vector<double, 3> dvec2;
typedef dlib::vector<int, 2> ivec2;

class hough_settings {
    public:
        hough_settings() {}

        int n_phi;
        int n_rho;
        double s_t;
        int s_ms;

        double max_point_distance;
        double max_distance2plane;

        double inv_camera_fx;
        double inv_camera_fy;
        double camera_cx;
        double camera_cy;
};

#endif
