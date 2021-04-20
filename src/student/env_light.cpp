
#include "../rays/env_light.h"
#include "debug.h"

#include <limits>

namespace PT {

Light_Sample Env_Map::sample() const {

    Light_Sample ret;
    ret.distance = std::numeric_limits<float>::infinity();

    // TODO (PathTracer): Task 7
    // Uniformly sample the sphere. Tip: implement Samplers::Sphere::Uniform
    Samplers::Sphere::Uniform uniform;
    ret.direction = uniform.sample(ret.pdf);

    // Once you've implemented Samplers::Sphere::Image, remove the above and
    // uncomment this line to use importance sampling instead.
    // ret.direction = sampler.sample(ret.pdf);

    ret.radiance = sample_direction(ret.direction);
    return ret;
}

Spectrum Env_Map::sample_direction(Vec3 dir) const {

    // TODO (PathTracer): Task 7
    // Find the incoming light along a given direction by finding the corresponding
    // place in the enviornment image. You should bi-linearly interpolate the value
    // between the 4 image pixels nearest to the exact direction.
    /**
    size_t w = std::get<0>(image.dimension());
    size_t h = std::get<1>(image.dimension());

    //convert direction to theta and 
    float theta = acosf(dir.y);
    float phi = atan2(dir.z, dir.x); //not sure how to calculate

    //DEBUG
    assert(theta >= 0.0f && theta <= PI_F);
    assert(phi >= 0.0f && phi <= PI_F);
    //lookup in texture
    float XX = phi / (2.0f * PI_F);
    float YY = theta / PI_F;

    //find base index
    size_t x = (size_t)floor(XX);
    size_t y = (size_t)floor(YY);

    //find surrounding values
    
    Spectrum& s1 = image.at(x, y);
    Spectrum& s2 = image.at((x + 1)%w, y);
    Spectrum& s3 = image.at(x, (y + 1) % h);
    Spectrum& s4 = image.at((x + 1)%w , (y + 1) % h);

    //bilinearly interpolate
    float x_w = floor(XX) + 1.0f - XX;
    float y_w = floor(YY) + 1.0f - YY;

    Spectrum ret = (s1 * (1.f - x_w) + s2 * x_w) * (1.f - y_w) + 
                    (s3 * (1.f - x_w) + s4 * x_w) * y_w;
    */


    return Spectrum();
}

Light_Sample Env_Hemisphere::sample() const {
    Light_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.radiance = radiance;
    ret.distance = std::numeric_limits<float>::infinity();
    return ret;
}

Spectrum Env_Hemisphere::sample_direction(Vec3 dir) const {
    if(dir.y > 0.0f) return radiance;
    return {};
}

Light_Sample Env_Sphere::sample() const {
    Light_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.radiance = radiance;
    ret.distance = std::numeric_limits<float>::infinity();
    return ret;
}

Spectrum Env_Sphere::sample_direction(Vec3) const {
    return radiance;
}

} // namespace PT
