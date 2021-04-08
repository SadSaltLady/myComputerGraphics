
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
    //float a = 1; //if dir is normalized, it's just 1
    float b = 2.0f * dot(ray.point, ray.dir);
    float c = dot(ray.point, ray.point) - radius*radius;

    //try to solve it 
    bool pos, neg; //mark if we had found a solution
    float pos_t, neg_t, dist_pos, dist_neg;
    Vec3 pos_sol, neg_sol;
    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?

    pos = false; neg = false;
    //SITUATION 1: no intersection -> no real solution
    float inside = b*b - 4*c; //inside the sqrt
    if (inside >= 0.0f) {
        pos_t = (-b + sqrt(inside))/2.0f;
        neg_t = (-b - sqrt(inside))/2.0f;
    } else {
        return ret; //if no real solution can be found, no intersection
    }
    //calculate for point of intersection
    pos_sol = ray.point + pos_t*ray.dir;
    neg_sol = ray.point + neg_t*ray.dir;
    //calculate the distance of intersection
    dist_pos = (pos_sol - ray.point).norm();
    dist_neg = (neg_sol - ray.point).norm();

    //check which intersection is in bound of ray.dist_disbound
    if (dist_pos > ray.dist_bounds.x && dist_pos < ray.dist_bounds.y) {
        pos = true;
    }
    if (dist_neg > ray.dist_bounds.x && dist_neg < ray.dist_bounds.y) {
        neg = true;
    }

    //SITUATION 2: has solutions, but none are inbound, no intersection is found
    if (!pos && !neg) {
        return ret;
    }

    ret.hit = true; //all situation onewards ensures a hit               
    
    //SITUATION 3: if one of them is in bound, then return the inbound solution
    if (pos && !neg) { //positive intersects
        ret.distance = dist_pos;   
        ret.position = pos_sol; 
        ret.normal = pos_sol.unit();  
        //update the ray_dist bounds
        ray.dist_bounds.y = dist_pos;
        return ret;
    } else if (neg && !pos) { //negative intersects
        ret.distance = dist_neg;   
        ret.position = neg_sol; 
        ret.normal = neg_sol.unit();  
        //update the ray_dist bounds
        ray.dist_bounds.y = dist_neg;
        return ret;
    }

    //situation 3: two intersections, record the closer one
    //determine which intersection came first
    if (dist_pos < dist_neg) {
        ret.distance = dist_pos;   
        ret.position = pos_sol; 
        ret.normal = pos_sol.unit();  
        //update the ray_dist bounds
        ray.dist_bounds.y = dist_pos;
    } else {
        ret.distance = dist_neg;   
        ret.position = neg_sol; 
        ret.normal = neg_sol.unit();  
        //update the ray_dist bounds
        ray.dist_bounds.y = dist_neg;
    }
    //if(RNG::coin_flip(0.0001f)) std::cout << ret.position;

    return ret;
}

} // namespace PT
