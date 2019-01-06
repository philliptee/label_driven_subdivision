#ifndef LABEL_SUBDIVISION_H
#define LABEL_SUBDIVISION_H

#include <OpenMesh/Core/Utils/Property.hh>
#include <set>

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

namespace LabelSubdivision {
    typedef OpenMesh::PolyMesh_ArrayKernelT<> MyMesh;

    typedef MyMesh::FaceHandle FaceHandle;
    typedef MyMesh::VertexHandle VertexHandle;
    typedef MyMesh::EdgeHandle EdgeHandle;
    typedef MyMesh::HalfedgeHandle HalfedgeHandle;

    typedef MyMesh::Point Point;
    typedef MyMesh::Normal Normal;
    typedef MyMesh::FaceIter FaceIter;
    typedef MyMesh::EdgeIter EdgeIter;
    typedef MyMesh::VertexIter VertexIter;

    typedef MyMesh::VertexEdgeIter VertexEdgeIter;
    typedef MyMesh::VertexFaceIter VertexFaceIter;

    typedef MyMesh::VOHIter VOHIter;

    extern OpenMesh::MPropHandleT <std::map<FaceHandle,int>> face_depths;
    extern OpenMesh::VPropHandleT <int> vertex_label;
    extern OpenMesh::MPropHandleT<std::set<VertexHandle>> positive_label_set;
    extern OpenMesh::MPropHandleT<bool> quad_mesh;

    // Initialize properties
    void prepare(MyMesh &_m);

    // Remove properties
    void cleanup(MyMesh &_m);

    void all_quadrilaterals_test(MyMesh &_m);

    void eliminate_illegal_faces(MyMesh &_m);

    // Label driven, Catmull-Clark subdivision
    void catmull_clark_subdivide(MyMesh &_m, bool single_iteration);


//===========================================================================

    void split_edge(MyMesh &_m, const EdgeHandle &_eh, const bool _update_points);

    void split_face(MyMesh &_m, const FaceHandle &_fh, const bool _update_points);

    void compute_midpoint(MyMesh &_m, const EdgeHandle &_eh, const bool _update_points);

    void update_vertex(MyMesh &_m, const VertexHandle &_vh);

    void populate_illegal_vertex_set(std::set<VertexHandle> &illegals, MyMesh &_m);

    void populate_illegal_vertex_attributes(std::set<VertexHandle> &illegals, MyMesh &_m);

    void set_vertex_updates(MyMesh &_m);

}

#endif
