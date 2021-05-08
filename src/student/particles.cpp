
#include "../scene/particles.h"
#include "../rays/pathtracer.h"

bool Particle::update(const PT::BVH<PT::Object>& scene, float dt, float radius) {
    
    //if no collision
    Vec3 velocity_update = velocity + acceleration*dt;

    //collisions
    Ray detect = Ray(pos, velocity.unit());
    PT::Trace hit = scene.hit(detect);
    float step_dist = (velocity*dt).norm();
    if (hit.hit && (hit.distance - radius -EPS_F) <= step_dist) {
        Vec3 surface = hit.normal.unit();
        velocity_update = velocity - 2*dot(velocity, surface)*surface + acceleration*dt;
    }
    

    //updates
    velocity = velocity_update;
    pos = pos + velocity*dt;    
    age -= dt;
    return (age > 0);
}
