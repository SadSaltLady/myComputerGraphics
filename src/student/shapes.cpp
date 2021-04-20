
#include "../rays/shapes.h"
#include "debug.h"
#include <iostream>
#include "../util/rand.h"

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
    float a = ray.dir.norm_squared(); //if dir is normalized, it's just 1
    assert(a > 0);
    float b = dot(ray.point, ray.dir);
    float c = ray.point.norm_squared() - radius*radius;
    float inv_a = 1.0f / a;

    //try to solve it 
    float pos_t, neg_t, dist_pos, dist_neg;
    Vec3 pos_sol, neg_sol;
    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection??

    //SITUATION 1: no intersection -> no real solution
    float inside = b*b - c; //inside the sqrt
    if (inside <= 0) {
        return ret;
    }

    pos_t = (-b + sqrt(inside))*inv_a;
    neg_t = (-b - sqrt(inside))*inv_a;

    //calculate for point of intersection
    pos_sol = ray.at(pos_t);
    neg_sol = ray.at(neg_t);
    //calculate the distance of intersection
    //**need to preserve sign in distance
    size_t sign_pos = (0 <= pos_t) ? 1 : -1;
    size_t sign_neg = (0 <= neg_t) ? 1 : -1;
    dist_pos = (pos_sol - ray.point).norm() * sign_pos;
    dist_neg = (neg_sol - ray.point).norm() * sign_neg;

    //check which intersection is in bound of ray.dist_disbound


    if (neg_t < ray.dist_bounds.y && neg_t > ray.dist_bounds.x) {
        ret.hit = true;
        ret.distance = dist_neg;   
        ret.position = neg_sol; 
        ret.normal = neg_sol.unit();  
        //update the ray_dist bounds
        ray.dist_bounds.y = neg_t;
    } else if (pos_t > ray.dist_bounds.x &&  pos_t < ray.dist_bounds.y) {
        ret.hit = true;
        ret.distance = dist_pos;   
        ret.position = pos_sol; 
        ret.normal = pos_sol.unit();
        //update the ray_dist bounds
        ray.dist_bounds.y = pos_t;
    }

    return ret;
}

} // namespace PT
