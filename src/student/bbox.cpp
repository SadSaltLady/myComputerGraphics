
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    /** code referenced:
     * https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
     */

    Vec3 invdir = 1 / ray.dir;
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    tmin = (min.x - ray.point.x) * invdir.x;
    tmax = (max.x - ray.point.x) * invdir.x;
    tymin = (min.y - ray.point.y) * invdir.y;
    tymax = (max.y - ray.point.y) * invdir.y;

    if((tmin > tymax) || (tymin > tmax)) {
        return false;
    }
    if(tymin > tmin) {
        tmin = tymin;
    }
    if(tymax < tmax) {
        tmax = tymax;
    }

    tzmin = (min.z - ray.point.z) * invdir.z; 
    tzmax = (max.z - ray.point.z) * invdir.z; 

    if ((tmin > tzmax) || (tzmin > tmax)) {
        return false; 
    }
    if (tzmin > tmin) {
        tmin = tzmin; 
    }
    if (tzmax < tmax) {
        tmax = tzmax; 
    }

    if (tmin < times.x || tmax > times.y) { //ray intersected outside of bound
        return false;
    }
    //shrink max if max is smaller
    if (tmax < times.y) {
        times.y = tmax;
    }

    return false;
}
