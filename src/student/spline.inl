
#include "../geometry/spline.h"
#include "debug.h"

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

    T t2 = powf(time, 2.f);
    T t3 = powf(time, 3.f); 

    T h00 = 2.f * t3 - 3.f * t2 + 1.f;
    T h10 = t3 - 2.f * t2 + time;
    T h01 = - 2.f * t3 + 3 * t2;
    T h11 = t3 - t2;

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

    return cubic_unit_spline(0.0f, T(), T(), T(), T());
}
