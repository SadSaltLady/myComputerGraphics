
#include "../rays/tri_mesh.h"
#include "debug.h"

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2
    // compute the bounding box of the triangle
    Vec3 v_0 = vertex_list[v0].position;
    Vec3 v_1 = vertex_list[v1].position;
    Vec3 v_2 = vertex_list[v2].position;

    Vec3 min = Vec3(std::min(std::min(v_0.x, v_1.x), v_2.x),
                          std::min(std::min(v_0.y, v_1.y), v_2.y),
                          std::min(std::min(v_0.z, v_1.z), v_2.z));

    Vec3 max = Vec3(std::max(std::max(v_0.x, v_1.x), v_2.x),
                          std::max(std::max(v_0.y, v_1.y), v_2.y),
                          std::max(std::max(v_0.z, v_1.z), v_2.z));


    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::intersect

    float epsilon = 0.0625f; //nice floating point :)
    //in case of flat/zero-volume box, add a small epsilon to the axis
    Vec3 diff = max - min;
    if (diff.x == 0.0f) max.x += epsilon;
    if (diff.y == 0.0f) max.y += epsilon;
    if (diff.z == 0.0f) max.z += epsilon;

    BBox box = BBox(min, max);
    return box;
}

Trace Triangle::hit(const Ray& ray) const {

    // Vertices of triangle - has postion and surface normal
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    (void)v_0;
    (void)v_1;
    (void)v_2;

    // TODO (PathTracer): Task 2
    // Intersect this ray with a triangle defined by the three above points.
    // method: the faster one (Moller-Trumbore algorithm)
    Vec3 e1 = v_1.position - v_0.position;
    Vec3 e2 = v_2.position - v_0.position;
    Vec3 S = ray.point - v_0.position;

    Trace ret; //initalize ret to be in no intersection state
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?
                           // (this should be interpolated between the three vertex normals)

    //calculate for (e1 x d) dot e2
    float denominator = dot(cross(e1, ray.dir), e2);
    //if denominator == 0, then ray parallel to face, thus no intersection
    //if denominator < 0, then the triangle is back facing 
    /** if you're looking back at this, here's a helpful link:
     * https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering
     * -a-triangle/moller-trumbore-ray-triangle-intersection
     */

    if (denominator == 0.0f) {
        return ret;
    }

    //construct the matrix for cramer's 
    float m_x = -(dot(cross(S, e2), ray.dir));
    float m_y = dot(cross(e1, ray.dir), S);
    float m_z = -(dot(cross(S, e2), e1));
    Vec3 m = Vec3(m_x, m_y, m_z);
    //calcualte for {u, v, t}
    Vec3 uvt = m / denominator;

    Vec3 intersect_pos = ray.point + (uvt.z * ray.dir); //location from param ray
    float intersect_dist = (intersect_pos - ray.point).norm();

    //if collision occured outside of ray dist_bounds, it is invalid
    if (intersect_dist < ray.dist_bounds.x || intersect_dist > ray.dist_bounds.y) {
        return ret;
    }

    //if collision occured outside of the triangle (1-u-v > 0),  it is invalid
    //also x and y has to be strictly between 0 & 1
    //x and y has to be between in 0-1
    if (uvt.x < 0.0f || uvt.x > 1.0f || uvt.y < 0.0f || uvt.y > 1.0f) {
        return ret;
    }
    if (1 - uvt.x - uvt.y < 0.0f || 1 - uvt.x - uvt.y > 1.0f) {
        return ret;
    }

    //affirm that we have a collision, now calculate for all the values needed to ret
    ret.origin = ray.point;
    ret.hit = true;       // yes intersection
    ret.position = intersect_pos; 
    ret.distance = intersect_dist;
    //interpolated between the three vertex normals
    ret.normal = (1 - uvt.x - uvt.y)*v_0.normal + uvt.x * v_1.normal + uvt.y * v_2.normal;

    //@TOASK: How should I update the ray bounds?
    //THOUGHT: set max to the current point we have calculated
    //update ray bounds, setting max to current pt of intersection
    ray.dist_bounds.y = intersect_dist;
    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

void Tri_Mesh::build(const GL::Mesh& mesh) {

    verts.clear();
    triangles.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    triangles.build(std::move(tris), 4);
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh) {
    build(mesh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangles = triangles.copy();
    return ret;
}

BBox Tri_Mesh::bbox() const {
    return triangles.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    Trace t = triangles.hit(ray);
    return t;
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    return triangles.visualize(lines, active, level, trans);
}

} // namespace PT
