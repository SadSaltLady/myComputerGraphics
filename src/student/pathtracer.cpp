
#include "../rays/pathtracer.h"
#include "../rays/samplers.h"
#include "../util/rand.h"
#include "debug.h"

namespace PT {

Spectrum Pathtracer::trace_pixel(size_t x, size_t y) {

    Vec2 xy((float)x, (float)y);
    Vec2 wh((float)out_w, (float)out_h);

    // (PathTracer): Task 1

    // Generate a sample within the pixel with coordinates xy and return the
    // incoming light using trace_ray.

    // Tip: Samplers::Rect::Uniform
    // Tip: you may want to use log_ray for debugging

    // This currently generates a ray at the bottom left of the pixel every time.

    // NAIVE IMPLEMENTATION:
    /** generate one ray at the center of the pixel, add a randomly generated offset,
     * normalize to screenspace coordinates([0-1] ~[0-1]), and then pass it to generate ray
     */

    Samplers::Rect::Uniform get_sample;
    get_sample.size = Vec2(1.0f, 1.0f);
    float pdf;
    Vec2 offset = get_sample.sample(pdf);

    // Vec2 offset = Vec2(0.5f, 0.5f);
    // normalized, then shift to [-0.5 - 0.5] coordinates
    Vec2 cameraspace = (xy + offset) / wh - Vec2(0.5f, 0.5f);
    assert(cameraspace.x >= -0.5f && cameraspace.x <= 0.5f);
    assert(cameraspace.y >= -0.5f && cameraspace.y <= 0.5f);

    Ray out = camera.generate_ray(cameraspace);

    if(RNG::coin_flip(0.00005f)) log_ray(out, 5.0f); //DEBUG
    return trace_ray(out);
}

Spectrum Pathtracer::trace_ray(const Ray& ray) {

    // Trace ray into scene. If nothing is hit, sample the environment
    Trace hit = scene.hit(ray);
    if(!hit.hit) {
        if(env_light.has_value()) {
            return env_light.value().sample_direction(ray.dir);
        }
        return {};
    }

    // If we're using a two-sided material, treat back-faces the same as front-faces
    const BSDF& bsdf = materials[hit.material];
    if(!bsdf.is_sided() && dot(hit.normal, ray.dir) > 0.0f) {
        hit.normal = -hit.normal;
    }

    // Set up a coordinate frame at the hit point, where the surface normal becomes {0, 1, 0}
    // This gives us out_dir and later in_dir in object space, where computations involving the
    // normal become much easier. For example, cos(theta) = dot(N,dir) = dir.y!
    Mat4 object_to_world = Mat4::rotate_to(hit.normal);
    Mat4 world_to_object = object_to_world.T();
    Vec3 out_dir = world_to_object.rotate(ray.point - hit.position).unit();

    // Debugging: if the normal colors flag is set, return the normal color
    if(debug_data.normal_colors) return Spectrum::direction(hit.normal);

    // Now we can compute the rendering equation at this point.
    // We split it into two stages: sampling lighting (i.e. directly connecting
    // the current path to each light in the scene), then sampling the BSDF
    // to create a new path segment.

    // TODO (PathTracer): Task 5
    // The starter code sets radiance_out to (0.5,0.5,0.5) so that you can test your geometry
    // queries before you implement path tracing. You should change this to (0,0,0) and accumulate
    // the direct and indirect lighting computed below.

    Spectrum radiance_out = Spectrum(0.0f); 
    {
        auto sample_light = [&](const auto& light) {
            // If the light is discrete (e.g. a point light), then we only need
            // one sample, as all samples will be equivalent
            int samples = light.is_discrete() ? 1 : (int)n_area_samples;
            for(int i = 0; i < samples; i++) {

                Light_Sample sample = light.sample(hit.position);
                Vec3 in_dir = world_to_object.rotate(sample.direction);

                // If the light is below the horizon, ignore it
                float cos_theta = in_dir.y;
                if(cos_theta <= 0.0f) continue;

                // If the BSDF has 0 throughput in this direction, ignore it.
                // This is another oppritunity to do Russian roulette on low-throughput rays,
                // which would allow us to skip the shadow ray cast, increasing efficiency.
                Spectrum attenuation = bsdf.evaluate(out_dir, in_dir);
                if(attenuation.luma() == 0.0f) continue;

                // TODO (PathTracer): Task 4s
                // Construct a shadow ray and compute whether the intersected surface is
                // in shadow. Only accumulate light if not in shadow.
                
                
                Ray shadowray = Ray(hit.position, sample.direction.unit());
                shadowray.dist_bounds.x = EPS_F;
                shadowray.dist_bounds.y = sample.distance - EPS_F;
                Trace shadowhit = scene.hit(shadowray);
                

                // Tip: since you're creating the shadow ray at the intersection point, it may
                // intersect the surface at time=0. Similarly, if the ray is allowed to have
                // arbitrary length, it will hit the light it was cast at. Therefore, you should
                // modify the time_bounds of your shadow ray to account for this. Using EPS_F is
                // recommended.

                // Note: that along with the typical cos_theta, pdf factors, we divide by samples.
                // This is because we're  doing another monte-carlo estimate of the lighting from
                // area lights.

                //NOTE: THIS really should be !shadowhit.hit
                if (!shadowhit.hit) {
                    radiance_out +=
                    (cos_theta / (samples * sample.pdf)) * sample.radiance * attenuation;
                }
                
            }
        };

        // If the BSDF is discrete (i.e. uses dirac deltas/if statements), then we are never
        // going to hit the exact right direction by sampling lights, so ignore them.
        if(!bsdf.is_discrete()) {
            for(const auto& light : lights) sample_light(light);
            if(env_light.has_value()) sample_light(env_light.value());
        }
    }

    // TODO (PathTracer): Task 5
    // Compute an indirect lighting estimate using pathtracing with Monte Carlo.


    // (2) Randomly select a new ray direction (it may be reflection or transmittance
    // ray depending on surface type) using bsdf.sample()
    auto sample_brdf = bsdf.sample(out_dir);


    // (1) Ray objects have a depth field; if it reaches max_depth, you should
    // terminate the path.
    if (ray.depth >= max_depth) {
        return radiance_out + sample_brdf.emissive; //maybe add emissive
    }
    Vec3 in_dir = sample_brdf.direction;
    //convert direction to world space (normalized)
    Vec3 new_dir_world = (object_to_world.rotate(sample_brdf.direction)).unit();
    //generate new ray from current position to the world space

    // (3) Compute the throughput of the recursive ray. This should be the current ray's
    // throughput scaled by the BSDF attenuation, cos(theta), and inverse BSDF sample PDF.
    // Potentially terminate the path using Russian roulette as a function of the new throughput.
    // Note that allowing the termination probability to approach 1 may cause extra speckling.
    //@toask what is this cos theta
    auto throughput = ray.throughput * sample_brdf.attenuation * std::abs(in_dir.y) * (1.0f / sample_brdf.pdf);
    float terminateProbability = 1.f - clamp(throughput.luma(), 0.0f, 1.0f); 
    //smaller/dimmer ray has a higher chance of being terminated
    if (RNG::unit() < terminateProbability) {
        return radiance_out + sample_brdf.emissive;
    }

    // (4) Create new scene-space ray and cast it to get incoming light. As wisth shadow rays, you
    // should modify time_bounds so that the ray does not intersect at time = 0. Remember to
    // set the new throughput and depth values.

    Ray new_ray = Ray(hit.position + new_dir_world * EPS_F, new_dir_world);
    new_ray.throughput = throughput; //??
    new_ray.dist_bounds.x = EPS_F;
    new_ray.depth = ray.depth + 1;

    // (5) Add contribution due to incoming light with proper weighting. Remember to add in
    // the BSDF sample emissive term.

    return radiance_out + sample_brdf.emissive + trace_ray(new_ray) * sample_brdf.attenuation * std::abs(in_dir.y) 
            / (sample_brdf.pdf * (1 - terminateProbability));

}

} // namespace PT
