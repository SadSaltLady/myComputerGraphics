
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.

    double pi = 3.14159265359;
    //screen height/width
    float sh = (float)(tan(vert_fov/2.0 * (pi/180.0))*2.0); 
    float sw = aspect_ratio * sh;

    //view_space coordinate of sampled point
    float vs_x = screen_coord.x * sw; 
    float vs_y = screen_coord.y * sh; 
    float focal = -1.0f * focal_dist;
    Vec3 vs_sample = Vec3(vs_x, vs_y, focal);

    //adjust point of origin (in view space)
    Vec3 vs_origin = Vec3();
    if (aperture > 0) {
        Samplers::Rect::Uniform aperture_sample;
        float pdf;
        aperture_sample.size = Vec2(aperture, aperture);
        Vec2 apt_offset = aperture_sample.sample(pdf);
        vs_origin.x = apt_offset.x - aperture/2.0f;
        vs_origin.y = apt_offset.y - aperture/2.0f;
    }
    
    //find origin in the worldspace
    Vec3 ws_origin = iview*vs_origin;

    //find sample point in world space
    Vec3 ws_sample = iview*vs_sample;

    //find ray direction in world space
    Vec3 ray_world = ws_sample - ws_origin;

    return Ray(ws_origin, ray_world.unit());
}
