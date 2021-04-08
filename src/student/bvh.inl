
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>

namespace PT {

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes don't
    // contain pointers to children, but rather indicies. This is because instead
    // of allocating each node individually, the BVH class contains a vector that
    // holds all of the nodes. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code builds a BVH with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    printf("%zd", max_leaf_size);
    size_t root = construct(primitives, 0, primitives.size(), max_leaf_size);
    root_idx = root;
}

template<typename Primitive>
size_t BVH<Primitive>::construct(std::vector<Primitive>& prims, size_t start, size_t size, size_t max_leaf_size) {
    //BASE CASE: construct leaf node
    if (size <= max_leaf_size) {
        printf("leaf\n");
        BBox leaf_box;
        for(size_t prim_idx = start; prim_idx < start + size; prim_idx++) {
            Primitive& prim = prims[prim_idx];
            leaf_box.enclose(prim.bbox());
        }
        size_t idx = new_node(leaf_box, start, size, 0, 0);
        return idx;
    }
    //RECURSIVE CASE
    printf("recursed, start: %zd, size: %zd\n", start, size);
    BBox box; //construct a bbox that encloses all the primatives
    for(size_t prim_idx = start; prim_idx < start + size; prim_idx++) {
        const Primitive& prim = prims[prim_idx];
        box.enclose(prim.bbox());
    } 
    //gather information about its min and max
    Vec3 bbmin = box.min;
    Vec3 bbmax = box.max;

    /** we want to keep track of the best partitions,
     * each partition is defined by:
     * - the partitioning axis
     * - the partitioning idx (where did we slice)
     * - the SAH cost
     */
    Vec3 best_axis = Vec3();
    size_t best_idx = 0;
    float best_SAH = INFINITY;
    size_t best1 = 0;
    size_t best2 = 0;
    //initalize the evalution values
    BBox eval_box1;
    BBox eval_box2;
    size_t prim_count1 = 0;
    size_t prim_count2 = 0;
    float amount;
    float SAH;

    float bucket_size = 8.0f;
    float slice_size;
    std::vector<Vec3> axes;
    Vec3 axis; //contais x, y, z axis
    axes.push_back(Vec3(1.0f, 0.0f, 0.0f));
    axes.push_back(Vec3(0.0f, 1.0f, 0.0f));
    axes.push_back(Vec3(0.0f, 0.0f, 1.0f));

    //evaluate base on the 3 axis
    for (size_t a = 0; a < 3; a++) {
        axis = axes[a];
        slice_size = (dot(bbmax, axis) - dot(bbmin, axis))/bucket_size;
        assert (slice_size > 0); //DEBUG

        //for each axis../
        for (size_t i = 1; i < bucket_size; i++) {
            amount = dot(bbmin, axis) + i*slice_size;
            //form a partition
            auto partition_pt = std::partition(
                prims.begin() + start, prims.begin() + start + size, 
                            [&](const Primitive& prim) -> bool {
                                return(dot(prim.bbox().center(), axis) < amount);}
            );
            //construct bbox from partition 
            for(auto prim = prims.begin()+start; prim != partition_pt; prim++) {
                //Primitive& prim = prims[prim_idx];
                eval_box1.enclose((*prim).bbox());
                ++prim_count1;
            }
            for(auto prim = partition_pt; prim != prims.begin() + start + size; prim++) {
                eval_box2.enclose((*prim).bbox());
                ++prim_count2;
            }
            //evaluate SAH
            SAH = (prim_count1*eval_box1.surface_area()) + (prim_count2*eval_box2.surface_area());

            //if better partition than current partition, record the current partition
            if (SAH < best_SAH) {
                best_axis = axis;
                best_idx = i;
                best_SAH = SAH;
                best1 = prim_count1;
                best2 = prim_count2;
            }

            //clear boxes & values
            eval_box1.reset();
            eval_box2.reset();
            prim_count1 = 0;
            prim_count2 = 0;
        }
    }

    //partition along the best partition 
    assert(best_idx != 0);
    slice_size = (dot(bbmax, best_axis) - dot(bbmin, best_axis))/bucket_size;
    amount = dot(bbmin, best_axis) + best_idx*slice_size;
    auto partition_pt = std::partition(prims.begin() + start, prims.begin() + start + size, 
                            [&](const Primitive& prim) -> bool { 
            return (dot(prim.bbox().center(), axis) < amount); }
            );
    size_t partitionIdx = size_t(std::distance(prims.begin(), partition_pt));
    //size_t partitions_size = partitionIdx - start;

    printf("partiton index: %zd, best: %zd, %zd, idx: %zd, size: %f",partitionIdx, best1, best2,best_idx, slice_size);
    //make recursive calls
    size_t left = construct(prims, start, best1, max_leaf_size);
    size_t right = construct(prims, partitionIdx, best2, max_leaf_size);
 
    //cosntruct node
    size_t idx = new_node(box, start, size, left, right);
    return idx;
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    Trace ret;
    for(const Primitive& prim : primitives) {
        Trace hit = prim.hit(ray);
        ret = Trace::min(ret, hit);
    }
    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(node.l && node.r) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
