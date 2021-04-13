
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
    //s
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

    size_t root = construct(primitives, 0, primitives.size(), max_leaf_size);
    root_idx = root;
}

template<typename Primitive>
size_t BVH<Primitive>::construct(std::vector<Primitive>& prims, size_t start, size_t size, size_t max_leaf_size) {
    //BASE CASE: construct leaf node
    if (size <= max_leaf_size) {
        BBox leaf_box;
        for(size_t prim_idx = start; prim_idx < start + size; prim_idx++) {
            Primitive& prim = prims[prim_idx];
            leaf_box.enclose(prim.bbox());
        }
        size_t idx = new_node(leaf_box, start, size, 0, 0);
        return idx;
    }
    //RECURSIVE CASE
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
    Vec3 axis; //contais x, y, z axis

    //evaluate base on the 3 axis
    for (size_t a = 0; a < 3; ++a) {
        if (a == 0) {
            axis = Vec3(0.0f, 0.0f, 1.0f);
        } else if (a == 1){
            axis = Vec3(1.0f, 0.0f, 0.0f);
        } else {
            axis = Vec3(0.0f, 1.0f, 0.0f);
        }
        slice_size = (dot(bbmax, axis) - dot(bbmin, axis))/bucket_size;
        assert (slice_size > 0); //DEBUG
        float temp = dot(bbmin, axis);
        //for each axis../
        for (size_t i = 1; i < bucket_size; i++) {
            amount =  temp + i*slice_size;
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

            //unicentered objects
            if (prim_count2 == 0 || prim_count1 == 0) {
                //just divide them by half
                size_t half = (prim_count2 + prim_count1)/2;
                partition_pt = prims.begin() + start + half;
                eval_box1.reset();
                eval_box2.reset();
                prim_count1 = 0;
                prim_count2 = 0;

                for(auto prim = prims.begin()+start; prim != partition_pt; prim++) {
                eval_box1.enclose((*prim).bbox());
                ++prim_count1;
                }

                for(auto prim = partition_pt; prim != prims.begin() + start + size; prim++) {
                eval_box2.enclose((*prim).bbox());
                ++prim_count2;
                }
            }
            //evaluate SAH
            SAH = (prim_count1*eval_box1.surface_area()/box.surface_area()) + 
                (prim_count2*eval_box2.surface_area()/box.surface_area());

            //if better partition than current partition, record the current partition
            if (prim_count1 > 0 && prim_count2 > 0 && SAH < best_SAH) {
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
    std::partition(prims.begin() + start, prims.begin() + start + size, 
                            [&](const Primitive& prim) -> bool { 
            return (dot(prim.bbox().center(), best_axis) < amount); }
            );
    size_t partitionIdx = start + best1;
    //debug
    //printf("current box has size: %zd, best index is %zd, best axis is: %zd \n", size, best_idx, axis_idx);
    //printf("partiton index: %zd, best1: %zd, best2: %zd, start: %zd, size: %f",partitionIdx, best1, best2,start, slice_size);
    
    //make recursive calls
    assert(best2 + best1 == size);
    assert(start + best1 == partitionIdx);
    assert(partitionIdx + best2 <= prims.size());
    size_t left = construct(prims, start, best1, max_leaf_size);
    size_t right = construct(prims, partitionIdx, best2, max_leaf_size);
 
    //cosntruct node
    size_t idx = new_node(box, start, size, left, right);
    return idx;
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {

    //  (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.
    Vec2 hitinfo = Vec2(0.0f, ray.dist_bounds.y);
    Trace ret = find_closest_hit(ray, root_idx, hitinfo);

    return ret;
}


template<typename Primitive> 
Trace BVH<Primitive>::find_closest_hit(const Ray& ray, size_t node_idx, Vec2& hitinfo) const {
    //note to self: the hitinfo keep tracks of distance from the hit point to the ray origin
    //why is it vec2? I don't know since I'm only using the max bound
    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       
    ret.distance = INFINITY;   
    //base case where it's a leaf
    if (nodes[node_idx].is_leaf()){
        const Node& thisnode = nodes[node_idx];
        //loop through all the primatives, find the closest hit
        for(size_t i = thisnode.start; i < thisnode.start + thisnode.size; i++) {
            const Primitive& prim = primitives[i];
            Trace hit = prim.hit(ray);
            ret = Trace::min(ret, hit);
        }
        /**
        //check if the found intersection is the closest
        if (hitinfo.y < ret.distance) {
            //we already found a closer hit, dont' return this hit
            ret.hit = false;
            ret.distance = hitinfo.y;
        } else {
            //found a closer hit (or not hit)
            //hit info records the max location of the intersection
            hitinfo.y = ret.distance;
        }
        */
        return ret;
    }
    //for the left and right child, check if bbox intersects
    const Node& node = nodes[node_idx];
    const Node& node_l = nodes[node.l];
    const Node& node_r = nodes[node.r];
    const BBox& box_l = node_l.bbox;
    const BBox& box_r = node_r.bbox;
    Vec2 times_l;
    Vec2 times_r;

    //see if child boxes intersects
    bool intersect_l = box_l.hit(ray, times_l);
    bool intersect_r = box_r.hit(ray, times_r);

    //times now should have been filled with intersect information if there's intersection.
    //case on situations:
    //CASE0: no intersection was found
    if (!intersect_l && !intersect_r) {
        ret.hit = false;
        return ret;
    }

    //CASE1: Only left or right intersected
    if (!intersect_l && intersect_r) { //only right intersects
        Trace right = find_closest_hit(ray, node.r, hitinfo);
        return right;
    } else if (!intersect_r && intersect_l) { //only left
        Trace left = find_closest_hit(ray, node.l, hitinfo);
        return left;
    }

    //CASE2: Both intersects
    //find which one intersects first
    size_t first = (times_l.x <= times_r.x) ? node.l : node.r;
    size_t second = (times_l.x <= times_r.x) ? node.r : node.l;
    Vec2& hitsecond = (times_l.x <= times_r.x) ? times_r : times_l;

    //updates hitinfo with the distance of the closest hit found
    Trace firsthit = find_closest_hit(ray, first, hitinfo);
    
    
    float secondhit_dist = (ray.dir*hitsecond.x).norm();

    if (secondhit_dist < hitinfo.y) {
        Trace secondhit = find_closest_hit(ray, second, hitinfo);
        ret = Trace::min(firsthit, secondhit);
        return ret;
    }

    return firsthit;
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

    // A node is a leaf if l == r, since all interior nodes must have distinct children
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

        if(!node.is_leaf()) {
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
