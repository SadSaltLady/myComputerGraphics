
#include "../geometry/spline.h"
#include "debug.h"
#include <iostream>

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

    // TODO (Animation): Task 1a
    // Given time in [0,1] compute the cubic spline coefficients and use them to compute
    // the interpolated value at time 'time' based on the positions & tangents

    // Note that Spline is parameterized on type T, which allows us to create splines over
    // any type that supports the * and + operators.

    T p0 = position0;
    T p1 = position1;
    T m0 = tangent0;
    T m1 = tangent1;

    float t2 = time * time;
    float t3 = time * time * time; 

    float h00 = 2.f * t3 - 3.f * t2 + 1.f;
    float h10 = t3 - 2.f * t2 + time;
    float h01 = - 2.f * t3 + 3 * t2;
    float h11 = t3 - t2;

    T ret = h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1;

    return ret;
}

template<typename T> T Spline<T>::at(float time) const {

    // TODO (Animation): Task 1b

    // Given a time, find the nearest positions & tangent values
    // defined by the control point map.

    // Transform them for use with cubic_unit_spline

    // Be wary of edge cases! What if time is before the first knot,
    // before the second knot, etc...
    
    size_t count = control_points.size();
    //case 0: no points
    if (!any()) { 
        return T();
    }
    //case 1: only one point?
    if (count == 1) {
        return control_points.begin()->second;
    }

    assert(control_points.size() >= 2);
    //have more points...
    auto pfirst = control_points.begin(); //assumed first point
    auto plast = std::prev(control_points.end()); //assumed last

    float what = plast->first;
    T w = plast->second;
    std::cout << "plast t: " << what << "\n";
    
    //if inquiry point before/after first/last
    if (time <= pfirst->first) {
        return pfirst->second;
    }
    if (time >= plast->first) {
        return plast->second;
    }

    //if inquiry point is between first and last
    std::cout << "size of spline : " << count << "\n";
    for (auto it = control_points.begin(); it != control_points.end(); it++) {
        what = it->first;
        w = it->second;
        std::cout << "time:" << what << " loc: " << w << "\n";
    }
    
    auto K2_iter = control_points.upper_bound(time);
    assert(K2_iter != control_points.end());
    auto K1_iter = std::prev(K2_iter);

    float t1 = K1_iter->first;
    float t2 = K2_iter->first;
    T k1 = K1_iter->second;
    T k2 = K2_iter->second;
    float t0, t3;
    T k0;
    T k3;

    std::cout << "curr time : " << time << "\n";

    std::cout << "t1 :" << t1 << " t2: " << t2 << "\n";
    //assert (t2 > t1);
    assert(t2 > time);
    assert(time >= t1);
    //attempt to find K0 and k3
    if (K1_iter == control_points.begin()) { //K0 doesn't exist
        t0 = t1 - (t2 - t1);
        k0 = k1 - (k2 - k1);
    } else {
        auto K0_iter = --K1_iter;
        t0 = K0_iter->first;
        k0 = K0_iter->second;
    }
    assert(K2_iter != control_points.end());
    if (K2_iter == plast) {
        t3 = t2 + (t2 - t1);
        k3 = k2 + (k2 - k1);
    } else {
        auto K3_iter = ++K2_iter;
        t3 = K3_iter->first;
        k3 = K3_iter->second;
    }

    assert (t1 > t0 && t2 > t1 && t3 > t2);

    //some values used in scaling
    float interval = t2 - t1;
    float s_time = (time - t1) / (interval);

    //calculate for tangent
    T m1 = interval * (k2 - k0) / (t2 - t0);
    T m2 = interval * (k3 - k1) / (t3 - t1);


    //return pfirst->second;
    return cubic_unit_spline(s_time, k1, k2, m1, m2);
}
