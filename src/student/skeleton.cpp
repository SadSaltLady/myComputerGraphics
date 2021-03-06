
#include "../scene/skeleton.h"

Vec3 closest_on_line_segment(Vec3 start, Vec3 end, Vec3 point) {

    // TODO(Animation): Task 3
    //assumptio: everything is in the object space

    Vec3 line = (end - start).unit();
    float lambda = dot((point - start), line);
    Vec3 ret = start + lambda * line;
    if (lambda <= 0.f) ret = start;
    if (lambda >= (end - start).norm()) ret = end;

    // Return the closest point to 'point' on the line segment from start to end
    return ret;
}

Mat4 Joint::joint_to_bind() const {

    // (Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space in bind position.

    // Bind position implies that all joints have pose = Vec3{0.0f}

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos
    

    //in short: just acculate all the translation information
    if (is_root()){
        return Mat4::I;
    }

    Mat4 L = Mat4::translate(parent->extent);

    return L * parent->joint_to_bind();
}

Mat4 Joint::joint_to_posed() const {

    // (Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space, taking into account joint poses.

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos
    
    //recursive?
    if (is_root()){
        return Mat4::euler(pose);
    }

    Mat4 LR = Mat4::translate(parent->extent) * Mat4::euler(pose);

    return parent->joint_to_posed() * LR;
}

Vec3 Skeleton::end_of(Joint* j) {

    // (Animation): Task 2

    // Return the bind position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.

    Mat4 bind_transform = j->joint_to_bind();
    
    return bind_transform * j->extent + base_pos;
}

Vec3 Skeleton::posed_end_of(Joint* j) {

    // (Animation): Task 2

    // Return the posed position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.
    Mat4 bind_transform = j->joint_to_posed();
    return  bind_transform * j->extent + base_pos;
}

Mat4 Skeleton::joint_to_bind(const Joint* j) const {

    // (Animation): Task 2   

    // Return a matrix transforming points in joint j's space to object space in
    // bind position. This should take into account Skeleton::base_pos.

    Mat4 trans = j->joint_to_bind();
    Mat4 base = Mat4::translate(base_pos);

    return base * trans;
}

Mat4 Skeleton::joint_to_posed(const Joint* j) const {

    // (Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space with
    // poses. This should take into account Skeleton::base_pos.

    
    Mat4 trans = j->joint_to_posed();
    Mat4 base = Mat4::translate(base_pos);

    return base * trans ;
}

void Skeleton::find_joints(const GL::Mesh& mesh,
                           std::unordered_map<unsigned int, std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Construct a mapping from vertex indices to lists of joints in this skeleton
    // that should effect the vertex at that index. A joint should effect a vertex
    // if it is within Joint::radius distance of the bone's line segment in bind position.

    const std::vector<GL::Mesh::Vert>& verts = mesh.verts();
    (void)verts;

    // For each i in [0, verts.size()), map[i] should contain the list of joints that
    // effect vertex i. Note that i is NOT Vert::id! i is the index in verts.

    //loop through all the vertices 
    for (size_t i = 0; i < verts.size(); ++i) {
        //loop through each joint
        std::vector<Joint*> influence;
        for_joints([&](Joint* j) {
            //aquire the transformation matrices, calcualte everything in the objectspace
            Mat4 to_skele = joint_to_bind(j);
            Vec3 start = to_skele*Vec3();
            Vec3 end = to_skele * j->extent;

            //check if point within range to be influenced
            Vec3 closest = closest_on_line_segment(start, end, verts[i].pos);
            float dist = (verts[i].pos - closest).norm();
            //add to map if within range of influence
            if (dist <= j->radius) {
                influence.push_back(j);
            }
        });

        //add the info collected to the map
        std::pair<unsigned int, std::vector<Joint*>> per_vert ((unsigned int)i, influence);
        map.insert(per_vert);
        //clear vector for next use
        influence.clear();
    }
}

void Skeleton::skin(const GL::Mesh& input, GL::Mesh& output,
                    const std::unordered_map<unsigned int, std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Apply bone poses & weights to the vertices of the input (bind position) mesh
    // and store the result in the output mesh. See the task description for details.
    // map was computed by find_joints, hence gives a mapping from vertex index to
    // the list of bones the vertex should be effected by.

    // Currently, this just copies the input to the output without modification.

    std::vector<GL::Mesh::Vert> verts = input.verts();
    for(size_t i = 0; i < verts.size(); i++) {
        //fetch the influenced joints
        std::vector<Joint*> influences = map.at((unsigned int)i);
        //somethings I use to store the weights
        float dist_sum = 0.f;
        Vec3 pos_final = Vec3();
        Vec3 normal_final = Vec3();
        //for each joint, find the transformations and influences, and weigth them
        for (size_t j = 0; j < influences.size(); ++j){
            Mat4 to_bind = joint_to_bind(influences[j]);
            Mat4 to_joint = to_bind.inverse();
            Mat4 to_pose = joint_to_posed(influences[j]);

            Vec3 start = to_bind*Vec3();
            Vec3 end = to_bind * influences[j]->extent;
            //find closest point and weight
            Vec3 closest = closest_on_line_segment(start, end, verts[i].pos);
            float dist = (verts[i].pos - closest).norm();
            dist_sum += 1/ dist;
            //calculate the posed position with weight
            
            Vec3 pos = (1.f / dist) * (to_pose*(to_joint*verts[i].pos));
            Vec3 nor = (1.f / dist) * (to_pose*(to_joint*verts[i].norm));
            pos_final += pos;
            normal_final += nor;
        }
        //update the final position of vert
        verts[i].pos = pos_final / dist_sum;
        verts[i].norm = normal_final / dist_sum;
    }

    std::vector<GL::Mesh::Index> idxs = input.indices();
    output.recreate(std::move(verts), std::move(idxs));
}

void Joint::compute_gradient(Vec3 target, Vec3 current) {

    // (Animation): Task 2

    // Computes the gradient of IK energy for this joint and, should be called
    // recursively upward in the heirarchy. Each call should storing the result
    // in the angle_gradient for this joint.

    // Target is the position of the IK handle in skeleton space.
    // Current is the end position of the IK'd joint in skeleton space.
    //BASE CASE
    if (is_root()) {
        return;
    }

    Vec3 diff = target - current;
    Mat4 to_world = parent->joint_to_posed();
    Vec3 p = current - to_world*parent->extent;
 
    //calculate the jacobian for each axis
    Vec3 j_x = cross(to_world*Vec3(1,0,0), p);
    Vec3 j_y = cross(to_world*Vec3(0,1,0), p);
    Vec3 j_z = cross(to_world*Vec3(0,0,1), p);

    //calculate the gradient for each axis
    Vec3 grad = Vec3();
    grad.x = dot(j_x, diff);
    grad.y = dot(j_y, diff);
    grad.z = dot(j_z, diff);

    angle_gradient += grad;

    parent->compute_gradient(target, current);
}

void Skeleton::step_ik(std::vector<IK_Handle*> active_handles) {

    // (Animation): Task 2

    // Do several iterations of Jacobian Transpose gradient descent for IK
    size_t step_count = 100;
    float step = 0.01;
    //for every time stamp
    for (size_t i = 0; i < step_count; i++){
        //for everY ik handle
        for (auto iter = active_handles.begin(); iter != active_handles.end(); iter++) {
            IK_Handle* IK = *iter;
            //for every effected joint in IK handle, update the gradient
            Joint* endjoint = IK->joint;
            Vec3 endpoint = endjoint->joint_to_posed() * endjoint->extent;
            IK->joint->compute_gradient(IK->target, endpoint);
        }

        //update the position of all the joints, and zero out the gradient
        for (auto iter = active_handles.begin(); iter != active_handles.end(); iter++) {
            IK_Handle* IK = *iter;
            //for every effected joint in IK handle, update the gradient
            Joint* endjoint = IK->joint;
            update_and_zero(endjoint, step);
        }
    }

}

void Skeleton::update_and_zero(Joint* j, float step) {
    if (j->is_root()) {
        j->angle_gradient = Vec3();
        return;
    }
    //update
    Vec3 increm = j->angle_gradient*step;
    j->pose += increm;
    //zerp
    j->angle_gradient = Vec3();
    //call on parent
    update_and_zero(j->parent, step);
}
