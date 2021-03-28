
#include <cassert>
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    // DOESN'T HANDLE BOUNDARIES FOR NOW
    if(e->on_boundary()) {
        return std::nullopt;
    }

    // approach: collapse everything onto the f0 plane

    // STEP 1: collect elements
    // HALFEDGES
    HalfedgeRef h0, h1, h2, h4, h3, h5;
    h0 = e->halfedge();
    h1 = h0->twin();

    h2 = h0->next();
    h4 = h0;
    while(h4->next() != h0) {
        h4 = h4->next();
    }

    h3 = h1->next();
    h5 = h1;
    while(h5->next() != h1) {
        h5 = h5->next();
    }

    std::vector<HalfedgeRef> f1_edges;
    HalfedgeRef temp = h1;
    while(temp->next() != h1) {
        temp = temp->next();
        f1_edges.push_back(temp);
    }
    // Edges
    EdgeRef e0 = h0->edge();
    // VERTEX
    VertexRef v0, v1;
    v0 = h0->vertex();
    v1 = h1->vertex();

    if (v0->degree() == 1 || v1->degree() == 1) {
        //edge case
        return std::nullopt;
    }
    // FACE
    FaceRef f0 = h0->face();
    FaceRef f1 = h1->face();

    // STEP 2: Reassign elements
    // HALFEDGES
    // for every halfedge in f1 needs to be on f0
    for(long unsigned int i = 0; i < f1_edges.size(); i++) {
        f1_edges.at(i)->face() = f0;
    }

    h2->face() = f0;
    h4->face() = f0;
    h5->next() = h2;
    h4->next() = h3;
    // uncessary but it makes me happy
    h2->vertex() = v1;
    h3->vertex() = v0;

    // VERTEX
    v0->halfedge() = h3;
    v1->halfedge() = h2;

    // FACES
    f0->halfedge() = h2;
    // DEBUG
    for(long unsigned int i = 0; i < f1_edges.size(); i++) {
        assert(f1_edges.at(i)->face() == f0);
    }

    // STEP 3: Delete elements we don't want
    erase(f1);
    erase(h0);
    erase(h1);
    erase(e0);

    return f0;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    // note: in this implementation, we are collapsing onto v0

    // DOESN'T HANDLE BOUNDARIES FOR NOW
    if(e->on_boundary()) {
        return std::nullopt;
    }

    // Halfedges corresponding to edge
    HalfedgeRef h0 = e->halfedge();
    HalfedgeRef h1 = h0->twin();

    // endpoints/vertices on the edge
    VertexRef v0 = h0->vertex();
    VertexRef v1 = h1->vertex();

    // VALIDITY CHECK
    /** in case of overlapping triangles("multiple edges across same vertices"),
     * do not collapse edge */
    // doesn't seem to work??
    HalfedgeRef checking2, checking3;
    checking2 = h0->next();
    checking3 = checking2->twin();
    while(checking3 != h0) {
        if((checking2->vertex() == v1) && (checking3->vertex() == v0)) {
            return std::nullopt;
        }
        checking2 = checking3->next();
        checking3 = checking2->twin();
    }

    // STEP 1: Collect all elements
    // h0,h1.v0.v1 gathered above
    // halfedges used in face transform
    HalfedgeRef h2 = h0->next(); // target next value for h4
    HalfedgeRef h4 = h0;         // half edge previous to h0
    while(h4->next() != h0) {
        h4 = h4->next();
    }

    HalfedgeRef h3 = h1->next(); // next for h7
    HalfedgeRef h7 = h1;
    while(h7->next() != h1) { // half edge previous to h1
        h7 = h7->next();
    }

    // collect all halfegdes going out from the endpoints
    std::vector<HalfedgeRef> v0_edges;
    std::vector<HalfedgeRef> v1_edges;
    // collect faces corresponding to the edge into a vector
    FaceRef f0 = h0->face();
    FaceRef f1 = h1->face();

    // iterate over half edges with twin()->next()
    HalfedgeRef temp = h0;
    while(temp->twin()->next() != h0) {
        v0_edges.push_back(temp);
        temp = temp->twin()->next();
    }
    temp = h1;
    while(temp->twin()->next() != h1) {
        v1_edges.push_back(temp);
        temp = temp->twin()->next();
    }

    // STEP 2: reassign elements
    // for all halfegdes on v1, change their vertex to v0
    // HALFEDGES
    /** for every halfedge, modify:
     * next, twin, vertex, edge, face
     */
    for(long unsigned int i = 0; i < v1_edges.size(); i++) {
        v1_edges.at(i)->vertex() = v0;
    }

    // now collapse the faces attached to the edge

    // case on condition if the sides is boundary, and if the side is triangle

    VertexRef v4 = h4->vertex();
    VertexRef v3 = h7->vertex();

    FaceRef f2 = h2->twin()->face();
    FaceRef f3 = h7->twin()->face();

    // for F0*****F0----------------------------------------------------------
    if(f0->degree() <= 3) {
        // if triangle, collapse into edge
        // STEP 1: collect information
        // EdgeRef e4 = h4->edge(); //ultimately collapse onto this edge
        EdgeRef e2 = h2->edge(); // edge that should get deleted

        HalfedgeRef h23 = h2->twin()->next();
        HalfedgeRef h21 = h2->twin();
        while(h21->next() != h2->twin()) {
            h21 = h21->next();
        }

        // STEP2 reassignment
        /** for every halfedge, modify:
         * next, twin, vertex, edge, face
         */
        h4->next() = h23;
        h4->face() = f2;
        h4->vertex() = v4;
        f2->halfedge() = h4;

        h21->next() = h4;
        v4->halfedge() = h4;

        // STEP 3: erase points I don't want
        erase(h2->twin());
        erase(h2);
        erase(e2);
        erase(f0);

    } else {
        // not triangle, then simply redirect the points
        h4->next() = h2;
        h2->vertex() = v0;
        v4->halfedge() = h4;
        f0->halfedge() = h0->next();
    }

    // FOR F1------------------------------------------------------
    if(f1->degree() <= 3) {
        // if triangle, collapse into edge
        // STEP 1: collect information
        // EdgeRef e3 = h3->edge(); //ultimately collapse onto this edge
        EdgeRef e7 = h7->edge(); // edge that should get deleted

        HalfedgeRef h13 = h7->twin()->next();
        HalfedgeRef h11 = h7->twin();
        while(h11->next() != h7->twin()) {
            h11 = h11->next();
        }

        // STEP2 reassignment
        /** for every halfedge, modify:
         * next, twin, vertex, edge, face
         */
        h3->next() = h13;
        h3->face() = f3;
        // h3->vertex() = v0;
        f3->halfedge() = h3;
        v3->halfedge() = h13;

        h11->next() = h3;

        // STEP 3: erase points I don't want
        erase(h7->twin());
        erase(h7);
        erase(e7);
        erase(f1);

    } else {
        // not triangle, then simply redirect the points
        h7->next() = h3;
        h7->twin()->vertex() = v0;
        h3->vertex() = v0;
        v3->halfedge() = h7;
        f1->halfedge() = h7;
    }

    // VERTEX
    // reassign v0->halfedge to ensure != h0
    v0->halfedge() = h4->twin();

    // change location of new vertex
    Vec3 v0_pos = v0->center();
    Vec3 v1_pos = v1->center();
    Vec3 new_pos = (v0_pos + v1_pos) / 2;

    v0->pos = new_pos;

    // STEP 3: delete elements
    erase(h0);
    erase(h1);
    erase(v1);
    erase(e);

    return v0;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    // if border, do not flip
    if(e->on_boundary()) {
        return std::nullopt;
    }

    // NOTES: a vertex's halfedge must point back to the vertext

    // STEP 1: Collecting the elements
    // Halfedges:
    HalfedgeRef h0 = e->halfedge(); // starting edge
    HalfedgeRef h1 = h0->twin();    // and its twin

    std::vector<HalfedgeRef> face0;
    std::vector<HalfedgeRef> face1;

    HalfedgeRef h2 = h0->next(); // h0->next
    HalfedgeRef h4 = h2->next(); // used as the next value for flipped edge
    HalfedgeRef h6 = h0;         // the edge previous to h0, need to modify next

    while(h6->next() != h0) {
        h6 = h6->next();
    }

    HalfedgeRef h3 = h1->next(); // h1->next
    HalfedgeRef h5 = h3->next(); // used as the next value for flipped edge
    HalfedgeRef h7 = h1;         // the edge previous to h1, need to modify next

    while(h7->next() != h1) {
        h7 = h7->next();
    }

    // Vetices:
    // need to modify edge assocated with them
    VertexRef v0 = h0->vertex(); // vertex corresponding to h0
    VertexRef v1 = h1->vertex(); // vertex corresponding to h1

    VertexRef v4 = h4->vertex(); // new startpoint for h1
    VertexRef v3 = h5->vertex(); // new startpoint for h0

    // Faces:
    // need to assocaite them with the modified edges
    FaceRef f0 = h0->face();
    FaceRef f1 = h1->face();

    // STEP 2: modify the pointers on the halfedges
    /** for every halfedge, modify:
     * next, twin, vertex, edge, face
     */
    h0->next() = h4;
    h0->twin() = h1;
    h0->vertex() = v3;
    h0->edge() = e;
    h0->face() = f0;

    h1->next() = h5;
    h1->twin() = h0;
    h1->vertex() = v4;
    h1->edge() = e;
    h1->face() = f1;

    // edge and face unchanged

    // STEP 3: reassign pointers at other points
    // HALFEDGES:
    // adjust the next positions of halfedges
    h6->next() = h3;
    h6->face() = f0;

    h7->next() = h2;
    h7->face() = f1;
    // adjust halfedgees now belonging in a new face
    // and next goes to the flipped edges
    h2->next() = h1;
    h2->face() = f1;
    h2->vertex() = v1;

    h3->next() = h0;
    h3->face() = f0;
    h3->vertex() = v0;

    // VERTICES:
    // associate vertices with new halfedges
    v0->halfedge() = h3;
    v1->halfedge() = h2;
    v4->halfedge() = h1;
    v3->halfedge() = h0;

    // assocaite face with the flipped edges
    f0->halfedge() = h0;
    f1->halfedge() = h1;

    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {
    // Appraoch: min allocation of new edges, maintain most information on v0
    // FOR NOW: doesn't handle border cases
    if(e->on_boundary()) {
        // determine the halfedge that's within the face
        HalfedgeRef h0, h1;
        if(!e->halfedge()->is_boundary()) {
            h0 = e->halfedge();
        } else {
            assert(!e->halfedge()->twin()->is_boundary());
            h0 = e->halfedge()->twin();
        }

        h1 = h0->twin();
        // only handles triangles
        if(h0->face()->degree() != 3) {
            return std::nullopt;
        }
        // STEP 1: Collect all elements (with respect to h0)
        // HALFEDGES
        HalfedgeRef h2, h4, h11, h13;
        h2 = h0->next();
        h4 = h2->next();
        // on the boundary plane
        h13 = h1->next();
        h11 = h1;
        while(h11->next() != h1) {
            h11 = h11->next();
        }

        // EDGES
        EdgeRef e0 = h0->edge();

        // VERTEX
        VertexRef v0, v1, v2;
        v0 = h0->vertex();
        v1 = h1->vertex();
        v2 = h4->vertex();

        // FACES
        FaceRef f0 = h0->face(); // the actual face
        FaceRef f = h1->face();  // the boundary plane

        // STEP 2: Create new elements
        HalfedgeRef n0, n1, n2, n3;
        n0 = new_halfedge();
        n1 = new_halfedge();
        n2 = new_halfedge();
        n3 = new_halfedge();

        EdgeRef e1, e2;
        e1 = new_edge();
        e2 = new_edge();

        VertexRef v3 = new_vertex();

        FaceRef f1 = new_face();

        // STEP 3: reassignment
        // HALFEDGES:
        /** for every halfedge, modify:
         * next, twin, vertex, edge, face
         */
        n0->next() = h4;
        n0->twin() = n1;
        n0->vertex() = v3;
        n0->edge() = e1;
        n0->face() = f0;

        n1->next() = n2;
        n1->twin() = n0;
        n1->vertex() = v2;
        n1->edge() = e1;
        n1->face() = f1;

        n2->next() = h2;
        n2->twin() = n3;
        n2->vertex() = v3;
        n2->edge() = e2;
        n2->face() = f1;

        n3->next() = h1;
        n3->twin() = n2;
        n3->vertex() = v1;
        n3->edge() = e2;
        n3->face() = f;

        // OLD edges
        h0->next() = n0;
        h0->edge() = e0;
        h0->face() = f0;

        h1->next() = h13;
        h1->vertex() = v3;
        h1->edge() = e0;
        h1->face() = f;

        h2->next() = n1;
        h2->face() = f1;

        h4->next() = h0;
        h4->face() = f0;

        h11->next() = n3;
        h13->vertex() = v0;

        // EDGES
        e1->halfedge() = n0;
        e2->halfedge() = n2;
        e0->halfedge() = h0;

        // VERTEX
        v2->halfedge() = n1;
        v0->halfedge() = h0;
        v1->halfedge() = n3;

        v3->halfedge() = h1;
        Vec3 pos_v3;
        pos_v3 = (v0->center() + v1->center()) / 2;
        v3->pos = pos_v3;

        // FACE
        f0->halfedge() = h4;
        f1->halfedge() = h2;

        return v3;
    }

    HalfedgeRef h0 = e->halfedge();
    HalfedgeRef h1 = h0->twin();

    // only handles triangles
    if((h0->face()->degree() != 3) && (h1->face()->degree() != 3)) {
        return std::nullopt;
    }

    // STEP 1: collect elements
    // HALFEDGES
    HalfedgeRef h2 = h0->next();
    HalfedgeRef h4 = h2->next();

    HalfedgeRef h3 = h1->next();
    HalfedgeRef h5 = h3->next();

    // EDGES
    EdgeRef e0 = h0->edge();

    // Vertex
    VertexRef v0 = h0->vertex();
    VertexRef v1 = h1->vertex();
    VertexRef v4 = h4->vertex();
    VertexRef v5 = h5->vertex();

    // FACES
    FaceRef f0 = h0->face();
    FaceRef f1 = h1->face();

    assert(h4->face() == f0 && h3->face() == f1);

    // STEP 2: Allocate new elem
    // probably could use a vector...//
    HalfedgeRef n0, n1, n2, n3, n4, n5;
    n0 = new_halfedge();
    n1 = new_halfedge();
    n2 = new_halfedge();
    n3 = new_halfedge();
    n4 = new_halfedge();
    n5 = new_halfedge();

    EdgeRef e3, e4, e5;
    e3 = new_edge();
    e4 = new_edge();
    e5 = new_edge();

    VertexRef v6 = new_vertex();

    FaceRef f2 = new_face();
    FaceRef f3 = new_face();

    // STEP 3: Reassign Variables
    // Initialize new varaibles, and modify old varliables
    // HALFEDGES
    /** for every halfedge, modify:
     * next, twin, vertex, edge, face
     */
    n0->next() = h4;
    n0->twin() = n1;
    n0->vertex() = v6;
    n0->edge() = e4;
    n0->face() = f0;

    n1->next() = n2;
    n1->twin() = n0;
    n1->vertex() = v4;
    n1->edge() = e4;
    n1->face() = f2;

    n2->next() = h2;
    n2->twin() = n3;
    n2->vertex() = v6;
    n2->edge() = e3;
    n2->face() = f2;

    n3->next() = n4;
    n3->twin() = n2;
    n3->vertex() = v1;
    n3->edge() = e3;
    n3->face() = f3;

    n4->next() = h5;
    n4->twin() = n5;
    n4->vertex() = v6;
    n4->edge() = e5;
    n4->face() = f3;

    n5->next() = h1;
    n5->twin() = n4;
    n5->vertex() = v5;
    n5->edge() = e5;
    n5->face() = f1;

    // old
    h0->next() = n0;
    h0->edge() = e0;
    h0->face() = f0;
    h0->vertex() = v0;

    h1->next() = h3;
    h1->edge() = e0;
    h1->face() = f1;
    h1->vertex() = v6;

    h2->next() = n1;
    h2->face() = f2;

    h5->next() = n3;
    h5->face() = f3;

    h3->next() = n5;
    h3->face() = f1;

    // EDGES
    e0->halfedge() = h0;
    e3->halfedge() = n3;
    e4->halfedge() = n1;
    e5->halfedge() = n5;

    // VERTEX
    v6->halfedge() = h1;
    // set position
    Vec3 pos_v6;
    pos_v6 = (v0->center() + v1->center() + v4->center() + v5->center()) / 4;
    v6->pos = pos_v6;
    // old
    v0->halfedge() = h0;
    v4->halfedge() = n1;
    v1->halfedge() = n3;
    v5->halfedge() = n5;

    // FACES
    f2->halfedge() = h2;
    f3->halfedge() = h5;
    // old
    f0->halfedge() = h4;
    f1->halfedge() = h3;

    return v6;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    // let's try
    // Step 1: collect elements
    // FACE
    FaceRef f0 = f;

    // HALF EDGES:
    // collect all the inner & outer halfedges on the face
    // all vertices, and all edges
    std::vector<HalfedgeRef> inner_HE;
    std::vector<HalfedgeRef> outer_HE;
    std::vector<EdgeRef> outer_edge;
    std::vector<VertexRef> outer_vertex;
    unsigned int N = f->degree();

    HalfedgeRef h0 = f->halfedge();
    // HalfedgeRef h1 = h0->twin();

    HalfedgeRef temp = h0;
    for(unsigned int i = 0; i < N; i++) {
        inner_HE.push_back(temp);
        outer_HE.push_back(temp->twin());

        EdgeRef e = temp->edge();
        outer_edge.push_back(e);

        VertexRef v = temp->vertex();
        outer_vertex.push_back(v);

        temp = temp->next();
    }

    // STEP 2: allocate new elements
    /** for each edge, there needs to be:
     * 1 new face
     * 1 new (inner vertex)
     * 2 edges
     * and 4 half edges created
     */

    std::vector<FaceRef> new_surroundface;
    std::vector<VertexRef> new_vertices;
    std::vector<EdgeRef> new_twin_edge;
    std::vector<EdgeRef> new_diagonal_edge;
    std::vector<std::vector<HalfedgeRef>> new_HE;
    assert(outer_vertex.size() == f->degree());
    for(long unsigned int i = 0; i < outer_vertex.size(); i++) {
        FaceRef newface = new_face();
        new_surroundface.push_back(newface);
        VertexRef newver = new_vertex();
        // set vertex position
        newver->pos = outer_vertex[i]->center();
        new_vertices.push_back(newver);
        EdgeRef newtwin = new_edge();
        new_twin_edge.push_back(newtwin);
        EdgeRef newdiagonal = new_edge();
        new_diagonal_edge.push_back(newdiagonal);
        std::vector<HalfedgeRef> HE_per_face;
        // each new extruded face is 4 sided
        for(long unsigned int j = 0; j < 4; j++) {
            HalfedgeRef h = new_halfedge();
            HE_per_face.push_back(h);
        }
        new_HE.push_back(HE_per_face);
    }

    // STEP 3: Reassign elements
    // start by initializing all new elements per new allocated face
    assert(new_HE.size() == N);
    for(long unsigned int i = 0; i < N; i++) {
        // iterate over the new faces
        // collecte all elements in the face and then reassign
        // halfedge array
        std::vector<HalfedgeRef> per_face = new_HE[i];
        // on the ouside/twin
        std::vector<HalfedgeRef> pf_twin;
        HalfedgeRef h4, h5, h6, h7;
        h4 = inner_HE[i];
        h5 = new_HE[(i + 1) % N][1];
        h6 = outer_HE[i];
        h7 = new_HE[(i + N - 1) % N][3];
        pf_twin = {h4, h7, h6, h5};
        // vertex
        std::vector<VertexRef> face_vertex;
        VertexRef v0, v1, v2, v3;
        v0 = new_vertices[(i + 1) % N];
        v1 = new_vertices[i];
        v2 = outer_vertex[i];
        v3 = outer_vertex[(i + 1) % N];
        face_vertex = {v0, v1, v2, v3};
        // edges
        std::vector<EdgeRef> face_edge;
        EdgeRef e0, e1, e2, e3;
        e0 = new_twin_edge[i];
        e1 = new_diagonal_edge[i];
        e2 = outer_edge[i];
        e3 = new_diagonal_edge[(i + 1) % N];
        face_edge = {e0, e1, e2, e3};
        // face
        FaceRef ff = new_surroundface[i];

        // INITIALIZE ALL NEW HALFEDGE
        // HALFEDGES:
        /** for every halfedge, modify:
         * next, twin, vertex, edge, face
         */
        // INITIALIZE ALL NEW HALFEDGE
        for(long unsigned int j = 0; j < 4; j++) {
            per_face.at(j)->next() = per_face[(j + 1) % 4];
            per_face.at(j)->twin() = pf_twin[j];
            per_face.at(j)->vertex() = face_vertex[j];
            per_face.at(j)->edge() = face_edge[j];
            per_face.at(j)->face() = ff;
        }
        // redirect old halfedges
        h4->twin() = per_face[0];
        h4->vertex() = v1;
        h4->edge() = e0;
        h4->face() = f0;

        h6->twin() = per_face[2];
        h6->vertex() = v3;
        h6->edge() = e2;

        // VERTEX
        v0->halfedge() = h5;
        v1->halfedge() = h4;
        v2->halfedge() = h7;
        v3->halfedge() = h6;

        // EDGES
        e0->halfedge() = h4;
        e1->halfedge() = h7;
        e2->halfedge() = h6;
        e3->halfedge() = h5;

        // face
        ff->halfedge() = per_face[0];
    }

    return f;
    ;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    unsigned int N = (unsigned int)new_halfedges.size();

    // Updating tangent & normal vectors
    /*
    std::vector<Vec3> HE_tangents;
    std::vector<Vec3> HE_normals;
    std::vector<VertexRef> verts;
    */
    /** store all the tangent vectors:
     * tangent calculated by a projection of v's location onto the midpoint of segment v2-v0
     * and projected onto the normal plane, such that its not modifying the normal
     * I think this is how Maya does it?
     */

    /** store all the normal vectors :
     * normal vectors are done by calculating the cross product of v1 and v0, such that
     * in the case of n-gon faces it will still generate a reasonable result
     */

    assert(new_halfedges.size() == face->degree());

    for(size_t i = 0; i < new_halfedges.size(); i++) {
        // store the vertex
        VertexRef v = new_halfedges[i]->vertex();

        // store all the tangent vectors
        Vec3 v0_pos = start_positions[(i + N - 1) % N];
        Vec3 v1_pos = start_positions[i];
        Vec3 v2_pos = start_positions[(i + 1) % N];
        Vec3 tangent = (v0_pos + v2_pos) / 2 - v1_pos;
        tangent.normalize();

        // store all normals
        Vec3 dir0 = v0_pos - v1_pos;
        Vec3 dir1 = v2_pos - v1_pos;
        Vec3 normal = cross(dir0, dir1);
        normal.normalize(); 
        //Vec3 normal = (face->normal()).normalize();

        // coplaner tests
        /*
        Vec3 v3_pos = start_positions[(i+N-2) % N];
        Vec3 v4_pos = start_positions[(i+2) % N];
        Vec3 dir00 = v4_pos - v1_pos;
        Vec3 dir00 = v3_pos - v1_pos;
        */

        // projection of tangent onto normal
        Vec3 projection_v = dot(tangent, normal) * normal;
        tangent = tangent - projection_v;
        // tangent.normalize();

        // update the vertex positions

        v->pos = v1_pos + tangent * tangent_offset + normal * normal_offset;
    }
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
    printf("i ran\n");
    //size_t face_count = n_faces();
    for(FaceRef f = faces_begin(); f != faces_end(); f++) {
        //printf("traversing...\n");
        if((!f->is_boundary()) && f->degree() > 3) {
            unsigned int count = f->degree() - 3;
            //printf("degree is %ud, id is %ud, count is %ud\n", f->degree(), f->id(), count);
            FaceRef f_old = f;
            HalfedgeRef A = f->halfedge();
            HalfedgeRef B = A->next()->next();
            HalfedgeRef A_prev, B_prev, temp;
            std::vector<HalfedgeRef> AtoB; //halfedged between A to B by calling next
            std::vector<HalfedgeRef> BtoA; //BtoA inclusive of B, exclusive of A
            VertexRef vA, vB;
            FaceRef f_AB, f_BA;
            for(unsigned int i = 0; i < count; ++i) {
                // collect elements 
                //printf("iterated at %ud\n", i);
                temp = A;
                while(temp != B){
                    AtoB.push_back(temp);
                    temp = temp->next();
                }
                assert(temp == B);
                while(temp != A) {
                    BtoA.push_back(temp);
                    temp = temp->next();
                }

                B_prev = AtoB.back();
                A_prev = BtoA.back();

                vA = A->vertex();
                vB = B->vertex();
                // allocate new elements
                FaceRef f_new = new_face();
                EdgeRef e = new_edge();
                HalfedgeRef AB = new_halfedge();
                HalfedgeRef BA = new_halfedge();
                // reassign elements
                // HALFEDGES:
                /** for every halfedge, modify:
                 * next, twin, vertex, edge, face */
                AB->next() = B;
                AB->twin() = BA;
                AB->vertex() = vA;
                AB->edge() = e;

                BA->next() = A;
                BA->twin() = AB;
                BA->vertex() = vB;
                BA->edge() = e;

                A_prev->next() = AB;
                B_prev->next() = BA;
                //EDGES
                e->halfedge() = AB;
                //VERTICES
                vA->halfedge() = AB;
                vB->halfedge() = BA;
                //faces
                if (i%2 == 0) { // for all even i
                    f_AB = f_old;
                    f_BA = f_new;
                } else {
                    f_AB = f_new;
                    f_BA = f_old;
                }

                for (std::vector<HalfedgeRef>::iterator it = AtoB.begin() ; it != AtoB.end(); ++it) {
                    (*it)->face() = f_BA;
                }
                BA->face() = f_BA;
                f_BA->halfedge() = BA;
                for (std::vector<HalfedgeRef>::iterator it = BtoA.begin() ; it != BtoA.end(); ++it) {
                    (*it)->face() = f_AB;
                }
                AB->face() = f_AB;
                f_AB->halfedge() = AB;
                if (i%2 == 0) { // for all even i
                    assert(f_BA->degree() == 3);
                    f_old = f_AB;
                    A = B->next();
                    B = AB;
                } else {
                    assert(f_AB->degree() == 3);
                    f_old = f_BA;
                    B = A->next();
                    A = BA;
                }
                AtoB.clear();
                BtoA.clear();
            }
        }
    }
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->new_pos = v->pos;
    }

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    for(EdgeRef e = edges_begin(); e != edges_end(); e++){
        e->new_pos = e->center();
    }

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        f->new_pos = f->center();
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        f->new_pos = f->center();
    }

    // Edges
    for(EdgeRef e = edges_begin(); e != edges_end(); e++){
        //collect the neighboring faces & halfedges
        HalfedgeRef A = e->halfedge();
        HalfedgeRef B = A->twin();
        FaceRef faceA = A->face();
        FaceRef faceB = B->face();
        //face average + edge center/2
        e->new_pos = (faceA->new_pos + faceB->new_pos + e->center()*2.0f)/4.0f;
    }

    // Vertices
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        float n = (float)v->degree();
        //iterate over all faces & edges connected to a vertex
        HalfedgeRef h = v->halfedge();
        HalfedgeRef h_start = v->halfedge();
        Vec3 Q = Vec3();
        Vec3 R = Vec3();
        Vec3 S = v->pos;
        do {
            FaceRef f = h->face();
            EdgeRef e = h->edge();
            Q += f->new_pos;
            R += e->center();
            h = h->twin()->next();
        } while (h != h_start);

        Q = Q/n;
        R = R/n;
        
        v->new_pos = (Q + 2.0f*R + (n-3.0f)*S)/n;
    }
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        // Compute new positions for all the vertices in the input mesh, using
        // the Loop subdivision rule, and store them in Vertex::new_pos.
        float n = (float)v->degree();
        float u;
        if (n == 3.0f) {
            u = 3/16;
        } else {
            u = 3.0f/(8.0f*n);
        }

        Vec3 old = v->pos;
        Vec3 neighbors = Vec3();
        HalfedgeRef h0 = v->halfedge();
        for (unsigned int i = 0; i < v->degree(); i++){
            VertexRef n_v = h0->twin()->vertex();
            neighbors += n_v->pos;
            h0 = h0->twin()->next();
        }
        v->new_pos = (1.0f - n*u) * old + u*neighbors;
        // -> At this point, we also want to mark each vertex as being a vertex of the
        //    original mesh. Use Vertex::is_new for this.
        v->is_new = false;
    }

    //set all original edges as old edges
    for(EdgeRef e = edges_begin(); e != edges_end(); e++){
        e->is_new = false;
        // -> Next, compute the updated vertex positions associated with edges, and
        //    store it in Edge::new_pos.
        //3/8 * (A + B) + 1/8 * (C + D)
        VertexRef A = e->halfedge()->vertex();
        VertexRef B = e->halfedge()->twin()->vertex();
        VertexRef C = e->halfedge()->next()->next()->vertex();
        VertexRef D = e->halfedge()->twin()->next()->next()->vertex();

        e->new_pos = 3.0f/8.0f * (A->pos + B->pos) + 1.0f/8.0f * (C->pos + D->pos);
    }

    size_t n = n_edges();
    EdgeRef e = edges_begin();
    size_t checking = 0;
    for (size_t i = 0; i < n; i++) {
        // get the next edge NOW!
        EdgeRef nextEdge = e;
        nextEdge++;
        // -> Next, we're going to split every edge in the mesh, in any order.  For
        //    future reference, we're also going to store some information about which
        //    subdivided edges come from splitting an edge in the original mesh, and
        //    which edges are new, by setting the flat Edge::is_new. Note that in this
        //    loop, we only want to iterate over edges of the original mesh.
        //    Otherwise, we'll end up splitting edges that we just split (and the
        //    loop will never end!)
        //if (!e->is_new) {
        assert(!e->is_new);
            auto ref = split_edge(e);
            if (ref.has_value()){
                VertexRef v_new = ref.value();
                HalfedgeRef h = v_new->halfedge();
                EdgeRef edge_old = h->edge();
                //set vertex as new
                v_new->is_new = true;
                v_new->new_pos = edge_old->new_pos;
                //set every newly created edge in the split as new
                for (size_t j = 0; j < 4; j++) { //splitting one edge generates 4 edges
                    if (j%2 == 1){ 
                        EdgeRef edge_new = h->edge();
                        edge_new->is_new = true;
                        checking++;
                    }
                    h = h->twin()->next();
                }
            } else {
                printf("loop_subdivide: impossible case, program aborted\n");
                assert(2 == 1);
                return;
            }
        //}
        e = nextEdge;
    }

    printf("checking: %zu",checking);
    
    for (EdgeRef ee = edges_begin(); ee != edges_end(); ee++){
        // Finally, flip any new edge that connects an old and new vertex.
        if (ee->is_new) {
            VertexRef v0 = ee->halfedge()->vertex();
            VertexRef v1 = ee->halfedge()->twin()->vertex();
            

            if (v0->is_new != true || v1->is_new == false) {
                flip_edge(ee);
            }
        }
    }
    

    // -> Finally, copy the new vertex positions into final Vertex::pos.
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->pos = v->new_pos;
    }

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {
            //edge = e;
        // Compute the combined quadric from the edge endpoints.
        Halfedge_Mesh::VertexRef v0 = e->halfedge()->vertex();
        Halfedge_Mesh::VertexRef v1 = e->halfedge()->twin()->vertex();
        Mat4 K_edge = vertex_quadrics[v0] + vertex_quadrics[v1];
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        Mat4 A = K_edge;
        A.cols[3].x = 0.0f; A.cols[3].y = 0.0f; A.cols[3].z = 0.0f; A.cols[3].w = 1.0f;
        A.cols[0].w = 0.0f; A.cols[1].w = 0.0f; A.cols[2].w = 0.0f;
        Vec3 b = Vec3(-A.cols[3].x, -A.cols[3].y, -A.cols[3].z);

        //if inverse exists: 
        if (A.det() != 0) {
            // -> Use this system to solve for the optimal position, and store it in
            //    Edge_Record::optimal.
            Vec3 x = A.inverse() * b;
            optimal = x;
            // -> Also store the cost associated with collapsing this edge in
            //    Edge_Record::cost.
            Vec4 x_calc = Vec4(x, 1.0f);
            Vec4 why = (K_edge * x_calc);
            cost = dot(x_calc, why);
        } else {
            //pick one of the end points with the lower cost. 
            Vec4 v0_pos = Vec4(v0->pos, 1.0f);
            Vec4 v1_pos = Vec4(v1->pos, 1.0f); 

            float cost_v0 = dot(v0_pos, (K_edge*v0_pos));
            float cost_v1 = dot(v1_pos, (K_edge*v1_pos));

            if (cost_v0 < cost_v1) {
                cost = cost_v0;
                optimal = v0->pos;
            } else {
                cost = cost_v1;
                optimal = v1->pos;
            }
        }
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    //remember current face count of mesh and calculate for target
    size_t original = n_faces();
    size_t target = original/4;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics

    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        //pick a vertex
        VertexRef p = f->halfedge()->vertex();
        //compute normalized normal and offset
        Vec3 n = f->normal().normalize();
        float d = -dot(p->pos, n);
        //compose v 
        Vec4 v = Vec4(n.x, n.y, n.z, d);
        //calculate matrix K
        Mat4 K = outer(v, v);
        face_quadrics[f] = K;
    }
    printf("loaded face quadrics \n");
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        //halfedge to iterate over faces
        HalfedgeRef h = v->halfedge();
        unsigned int count = v->degree();
        Mat4 K_sum = Mat4(Vec4(),Vec4(),Vec4(),Vec4());
        FaceRef f;
        //iterate over faces and collect sum
        for (unsigned int i = 0; i < count; i++) {
            f = h->face();
            assert(face_quadrics.count(f) == 1);
            Mat4 K = face_quadrics[f];
            K_sum += K;
            h = h->twin()->next();
        }
        //add to dictionary
        vertex_quadrics[v] = K_sum;
    }
    printf("loaded vertex quadrics \n");

    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        Edge_Record R = Edge_Record(vertex_quadrics, e);
        edge_records[e] = R;
        edge_queue.insert(R);
    }
    printf("loaded edge and PQ  \n");
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    for (size_t i = 0; i < original-target; i++) {
        Edge_Record cheapest = edge_queue.top();
        edge_queue.pop();
        printf("checkpoint4\n");
        //get the removed edge and its endpoints
        EdgeRef removed = cheapest.edge;
        VertexRef v0 = removed->halfedge()->vertex();
        VertexRef v1 = removed->halfedge()->twin()->vertex();
        Mat4 m_v0 = vertex_quadrics[v0];
        Mat4 m_v1 = vertex_quadrics[v1];
        printf("checkpoint5\n");
        //Remove any edge touching either of its endpoints from the queue.
        unsigned int v0_d = v0->degree();
        unsigned int v1_d = v1->degree();
        HalfedgeRef ref = v0->halfedge();
        printf("checkpoint6\n");
        for (unsigned int j = 0; j < v0_d; j++) {
            EdgeRef eref = ref->edge();
            auto search = edge_records.find(eref);
            if (search != edge_records.end()) {
                printf("checkpoint11\n");
                Edge_Record e_rec = edge_records[eref];
                //found an edge assocaited with endpoint v0;
                edge_queue.remove(e_rec);
            }
            ref = ref->twin()->next();
        }
        ref = v1->halfedge();
        printf("checkpoint7\n");
        for (unsigned int j = 0; j < v1_d; j++) {
            printf("checkpoint8\n");
            EdgeRef eref = ref->edge();
            printf("waht");
            auto search = edge_records.find(eref);
            printf("checkpoint9\n");
            if (search != edge_records.end()) {
                printf("checkpoint11\n");
                Edge_Record e_rec = edge_records[eref];
                //found an edge assocaited with endpoint v1;
                edge_queue.remove(e_rec);
                printf("checkpoint10\n");
            }
            ref = ref->twin()->next();
        }
        
        printf("removed all elem from PQ \n");


        //collapse the edge
        auto v = collapse_edge_erase(removed);
        if (v.has_value()){
            VertexRef v_new = v.value();
            printf("checkpoint1\n");
            //set location of new vertex
            //Set the quadric of the new vertex to the quadric computed in Step 3.
            vertex_quadrics[v_new] = m_v0 + m_v1;
            printf("checkpoint2\n");
            unsigned int n = v_new->degree();
            HalfedgeRef he_new = v_new->halfedge();
            EdgeRef e_new;
            printf("checkpoint3\n");
            for (unsigned int jj = 0; jj < n; jj++) {
                e_new = he_new->edge();
                Edge_Record R = Edge_Record(vertex_quadrics, e_new);
                //insert into dictionary
                edge_records[e_new] = R;
                //insert into Pqueue
                edge_queue.insert(R);
                he_new = he_new->twin()->next();
            }
        }
        printf("added new elems\n");

    }


    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.
    return true;
}
