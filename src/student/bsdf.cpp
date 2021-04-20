
#include "../rays/bsdf.h"
#include "../util/rand.h"
#include <iostream>
#include "debug.h"

namespace PT {

Vec3 reflect(Vec3 dir) {

    // (PathTracer): Task 6
    // Return reflection of dir about the surface normal (0,1,0).
    
    return Vec3(-dir.x, dir.y, -dir.z);
}

Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    // TODO (PathTracer): Task 6
    // Use Snell's Law to refract out_dir through the surface
    // Return the refracted direction. Set was_internal to false if
    // refraction does not occur due to total internal reflection,
    // and true otherwise.

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.
    Vec3 ret;
    //determine ior (if ray is going in/coming out of material)
    float ior_inv = (out_dir.y >= 0) ? (1.0f/index_of_refraction) : (index_of_refraction);
    //total internal refraction?
    float d = 1.0f - (ior_inv * ior_inv) * (1.0f - (out_dir.y * out_dir.y)); 

    if (d < 0.0f) { //maybe I can just reflect here 
        was_internal = true;
        return reflect(out_dir);
    }

    ret.x = -out_dir.x * ior_inv;
    ret.z = -out_dir.z * ior_inv;
    ret.y = (out_dir.y >= 0 ? -1 : 1) * sqrtf(d);

    return ret.unit();
}

BSDF_Sample BSDF_Lambertian::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5
    // Implement lambertian BSDF. Use of BSDF_Lambertian::sampler may be useful
    float pdf;
    Vec3 in_dir = sampler.sample(pdf); //random hemisphere sample
    Spectrum eval = evaluate(out_dir, in_dir);
    //convert to object space
    BSDF_Sample ret;
    ret.attenuation = eval; // What is the ratio of reflected/incoming light?
    ret.direction = in_dir;       // What direction should we sample incoming light from?
    ret.pdf = pdf;               // Was was the PDF of the sampled direction?
    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    return albedo * (1.0f / PI_F);
}

BSDF_Sample BSDF_Mirror::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement mirror BSDF

    BSDF_Sample ret;
    Vec3 in_dir = reflect(out_dir);
    ret.attenuation = evaluate(out_dir, in_dir); // What is the ratio of reflected/incoming light?
    ret.direction = in_dir;       // What direction should we sample incoming light from?
    ret.pdf = 1.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    return ret;
}

Spectrum BSDF_Mirror::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // Technically, we would return the proper reflectance
    // if in_dir was the perfectly reflected out_dir, but given
    // that we assume these are single exact directions in a
    // continuous space, just assume that we never hit them
    // _exactly_ and always return 0.
    return reflectance * (1.0f  / std::abs(in_dir.y));
}

BSDF_Sample BSDF_Glass::sample(Vec3 out_dir) const {

    //(PathTracer): Task 6
    // Implement glass BSDF.
    BSDF_Sample ret;
    bool valid = false;
    //try refract
    Vec3 in_dir = refract(out_dir, index_of_refraction, valid);
    if (valid) { //total internal reflection
        in_dir = reflect(out_dir);
        ret.attenuation = reflectance * (1.0f  / std::abs(in_dir.y));
        ret.direction = in_dir;
        ret.pdf = 1.0f;
        return ret;
    }
    
    // (1) Compute Fresnel coefficient. Tip: use Schlick's approximation.
    // using Schlick's approximation

    //find the outgoing ray from the surface

    float etai = 1.0f;
    float etat = index_of_refraction;

    if (out_dir.y >= 0.0f)
    {
        std::swap(etai, etat);
    }

    float costheta_i = std::abs(out_dir.y);
    float costheta_t = std::abs(in_dir.y);

    float r_parl = ((etat * costheta_i) - (etai * costheta_t)) /
                ((etat * costheta_i) + (etai * costheta_t));
    float r_perp = ((etai * costheta_i) - (etat * costheta_t)) /
                ((etai * costheta_i) + (etat * costheta_t));

    float fresnel = (r_parl * r_parl + r_perp * r_perp) * 0.5f;
    /**
    float outgoing = (out_dir.y >= 0) ? out_dir.y : in_dir.y;
    float R0 = pow((index_of_refraction - 1.f) / (index_of_refraction + 1.f), 2.f);

    float fresnel = R0 + pow((1.f - R0) * (1.f - outgoing), 5.f);
    */

    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    if (RNG::coin_flip(fresnel)) { //reflect 
        in_dir = reflect(out_dir);
        ret.attenuation = reflectance * (fresnel / std::abs(in_dir.y));
        ret.direction = in_dir;
        ret.pdf = clamp(fresnel, 0.0f, 1.0f);
    } else { //refract
        ret.attenuation = transmittance * ((1.0f - fresnel) / std::abs(in_dir.y));
        ret.direction = in_dir;
        ret.pdf = clamp(1.0f - fresnel, 0.0f, 1.0f);;
    }
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    return ret;
}

Spectrum BSDF_Glass::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Diffuse::sample(Vec3 out_dir) const {
    BSDF_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.emissive = radiance;
    ret.attenuation = {};
    return ret;
}

Spectrum BSDF_Diffuse::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // No incoming light is reflected; only emitted
    return {};
}

BSDF_Sample BSDF_Refract::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement pure refraction BSDF.

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?

    BSDF_Sample ret;
    bool was_internal = false;
    Vec3 in_dir = refract(out_dir, index_of_refraction, was_internal);

    if (was_internal) {
        in_dir = reflect(out_dir);
    }
    ret.attenuation = evaluate(out_dir, in_dir); // What is the ratio of reflected/incoming light?
    ret.direction = in_dir;       // What direction should we sample incoming light from?
    ret.pdf = 1.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    return ret;
}

Spectrum BSDF_Refract::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return transmittance * (1.0f  / std::abs(in_dir.y));
}

} // namespace PT
