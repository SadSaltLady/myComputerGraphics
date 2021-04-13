
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

    //does the ray pass through the box
    Vec3 invdir = 1 / ray.dir;
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    tmin = (min.x - ray.point.x) * invdir.x;
    tmax = (max.x - ray.point.x) * invdir.x;
    tymin = (min.y - ray.point.y) * invdir.y;
    tymax = (max.y - ray.point.y) * invdir.y;

    if((tmin > tymax) || (tymin > tmax)) {
        return false;
    }    
    //printf("what???\n");

    if(tymin > tmin) {
        tmin = tymin;
    }
    if(tymax < tmax) {
        tmax = tymax;
    }

    tzmin = (min.z - ray.point.z) * invdir.z; 
    tzmax = (max.z - ray.point.z) * invdir.z; 
    if (tzmin > tzmax) std::swap(tzmin, tzmax);

    if ((tmin > tzmax) || (tzmin > tmax)) {
        return false; 
    }
    if (tzmin > tmin) {
        tmin = tzmin; 
    }
    if (tzmax < tmax) {
        tmax = tzmax; 
    }
    /**
    if (tmin < times.x || tmax > times.y) { //ray intersected outside of bound
        return false;
    }
    */

    //it intersects the box, but is the intersection in range of the ray?
    /**
    Vec3 min_point = ray.point + tmin*ray.dir;
    Vec3 max_point = ray.point + tmax*ray.dir;
    float min_dist = (min_point - ray.point).norm();
    float max_dist = (max_point - ray.point).norm();
    if (max_dist > ray.dist_bounds.y || min_dist < ray.dist_bounds.x)    {
        return false;
    }
    */
    //feedback on the times it hit
    times.x = tmin;
    times.y = tmax;

    //times.x = std::min(tmin, std::min(tymin, tzmin));
    //times.y = std::max(tmax, std::max(tzmax, tymax));

    //printf("hit\n");
    return true;
}
