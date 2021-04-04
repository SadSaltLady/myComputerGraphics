
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    //try to reach the quadratic equation:
    /** slide referenced: 
     * https://cmu-graphics.github.io/Scotty3D/pathtracer/ray_sphere_intersection
     */
    float a = dot(ray.dir, ray.dir); //if dir is normalized, this is just 1?
    float b = 2.0f * dot(ray.point, ray.dir);
    float c = dot(ray.point, ray.point) - radius*radius;

    //try to solve it 
    bool pos, neg, single; //mark if we had found a solution
    float pos_t, neg_t, dist_pos, dist_neg;
    Vec3 pos_sol, neg_sol, first, next;
    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?

    pos, neg, single = false; 
    //pos_sol, neg_sol = 15462;
    //situation 1: no intersection -> no real solution
    float inside = b*b - 4*a*c; //inside the sqrt
    if (inside >= 0.0f) {
        pos_t = b/2 + sqrt(inside);
        neg_t = b/2 - sqrt(inside);
        pos, neg = true;
        if (inside == 0) {
            single = true;
        }
    } else {
        return ret;
    }
    
    pos_sol = ray.point + pos_t*ray.dir;
    neg_sol = ray.point + neg_t*ray.dir;

    dist_pos = pos_sol.norm();
    dist_neg = neg_sol.norm();

    //determine which intersection came first
    if (dist_pos < dist_neg) {
        first = pos_sol;
        next = neg_sol;
    } else {
        first = neg_sol;
        next = pos_sol;
    }

    //if (first)

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    return ret;
}

} // namespace PT
