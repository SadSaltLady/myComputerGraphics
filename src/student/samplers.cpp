
#include "../rays/samplers.h"
#include "../util/rand.h"
#include "debug.h"

namespace Samplers {

Vec2 Rect::Uniform::sample(float& pdf) const {

    // (PathTracer): Task 1
    // Generate a uniformly random point on a rectangle of size size.x * size.y
    // Tip: RNG::unit()

    pdf = 1.0f / (size.x * size.y); // the PDF should integrate to 1 over the whole rectangle
    return Vec2(RNG::unit() * size.x, RNG::unit() * size.y); //*scale
}

Vec3 Hemisphere::Cosine::sample(float& pdf) const {

    // TODO (PathTracer): Task 6
    // You may implement this, but don't have to.
    return Vec3();
}

Vec3 Sphere::Uniform::sample(float& pdf) const {

    // TODO (PathTracer): Task 7
    // Generate a uniformly random point on the unit sphere (or equivalently, direction)
    // Tip: start with Hemisphere::Uniform
    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    //chance to flip everything so it's a sphere
    //:P
    Vec3 ret = Vec3(xs, ys, zs).unit();
    if (RNG::unit() <= 0.5f) {
        ret = -1.f * ret;
    }
    pdf = 1.0f / (4.0f * PI_F);
    return ret;
}

Sphere::Image::Image(const HDR_Image& image) {

    // TODO (PathTracer): Task 7
    // Set up importance sampling for a spherical environment map image.

    // You may make use of the pdf, cdf, and total members, or create your own
    // representation.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;

    float pdfsum = 0.f;
    float cdfsum = 0.f;
    float p_pdf;

    //row major(first row then second row...)
    for (size_t col = 0; col < h; col++) {
        for (size_t row = 0; row < w; row++ ) {

            //size_t idx = col * w + row;

            const Spectrum& pixel = image.at(row, col);
            float theta = std::acos((float)(col / h));

            //calculate the pdf corresonding to each pixel
            p_pdf = (pixel.luma() * sinf(theta) * 2 * PI_F * PI_F) /(w * h);
            assert(p_pdf >= 0.f);
            pdfsum += p_pdf;
            pdf.push_back(p_pdf);
        }
    }
    assert(pdf.size() ==  w * h);
    //normalize all the samples taken
    size_t i = 0;
    for (auto &start :pdf) {
        start = start/pdfsum;
        cdfsum += start;
        cdf.push_back(cdfsum);
        i++;
    }

}

Vec3 Sphere::Image::sample(float& out_pdf) const {

    // TODO (PathTracer): Task 7
    // Use your importance sampling data structure to generate a sample direction.
    // Tip: std::upper_bound can easily binary search your CDF

    //randomly sample
    float Xi1 = RNG::unit();

    //find the sample & retrive w and height index
    auto theta_idx = std::upper_bound(cdf.begin(), cdf.end(), Xi1) - cdf.begin();
    float w_idx = (float)(theta_idx % w);
    float h_idx = (float)(theta_idx / w);

    float theta = ((float)h - h_idx) / (float)h * PI_F;
    float phi = 2.0f * PI_F * (w_idx / (float) w);

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);
    
    //need to find theta and phi

    out_pdf = pdf[theta_idx]; // what was the PDF (again, PMF here) of your chosen sample?
    return Vec3(xs, ys, zs).unit();
}

Vec3 Point::sample(float& pmf) const {

    pmf = 1.0f;
    return point;
}

Vec3 Two_Points::sample(float& pmf) const {
    if(RNG::unit() < prob) {
        pmf = prob;
        return p1;
    }
    pmf = 1.0f - prob;
    return p2;
}

Vec3 Hemisphere::Uniform::sample(float& pdf) const {

    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    pdf = 1.0f / (2.0f * PI_F);
    return Vec3(xs, ys, zs);
}

} // namespace Samplers
