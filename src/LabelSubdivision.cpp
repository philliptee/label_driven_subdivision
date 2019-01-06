#include <bits/stdc++.h>


#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Utils/MeshCheckerT.hh>
#include "LabelSubdivision.h"

namespace LabelSubdivision {

    OpenMesh::MPropHandleT<std::map<FaceHandle, int>> face_depths;
    OpenMesh::MPropHandleT<std::set<VertexHandle>> positive_label_set;
    OpenMesh::MPropHandleT<bool> quad_mesh;

    OpenMesh::VPropHandleT<Point> vp_pos_;
    OpenMesh::VPropHandleT<bool> vp_update_;
    OpenMesh::VPropHandleT<int> potential_new_illegal_count_;
    OpenMesh::VPropHandleT<int> illegal_valence_;
    OpenMesh::VPropHandleT<int> vertex_label;

    OpenMesh::EPropHandleT<Point> ep_pos_;
    std::vector<VertexHandle> zero_v;
    OpenMesh::FPropHandleT<Point> fp_pos_;
    OpenMesh::FPropHandleT<VertexHandle> pos_vert_id_;
    OpenMesh::FPropHandleT<bool> face_update_;

//-----------------------------------------------------------------------------

    void prepare(MyMesh &_m, const bool _update_points) {
        _m.add_property(vp_pos_);
        _m.add_property(ep_pos_);
        _m.add_property(fp_pos_);

        if (_update_points) {
            _m.add_property(pos_vert_id_);
            _m.add_property(face_update_);
            _m.add_property(vp_update_);
        }
    }

    void set_vertex_updates(MyMesh &_m) {
        _m.property(positive_label_set).clear();

        for (auto v_itr = _m.vertices_begin(); v_itr != _m.vertices_end(); ++v_itr)
            _m.property(vp_update_, *v_itr) = false;

        for (auto f_itr = _m.faces_begin(); f_itr != _m.faces_end(); ++f_itr) {
            _m.property(face_update_, *f_itr) = false;
            int pos_label_count = 0;
            std::vector<HalfedgeHandle> half_edge_to_pos_label;
            for (auto fh_it = _m.fh_ccwiter(*f_itr); fh_it.is_valid(); ++fh_it) {
                if (_m.property(vertex_label, _m.to_vertex_handle(*fh_it)) > 0) {
                    _m.property(positive_label_set).insert(_m.to_vertex_handle(*fh_it));
                    _m.property(pos_vert_id_, *f_itr) = _m.to_vertex_handle(*fh_it);
                    pos_label_count += 1;
                    half_edge_to_pos_label.push_back(*fh_it);
                    _m.property(face_update_, *f_itr) = true;
                }
            }
            if (half_edge_to_pos_label.size() >= 2) {
                for (auto fh_it = _m.fh_ccwiter(*f_itr); fh_it.is_valid(); ++fh_it) {
                    _m.property(vp_update_, _m.from_vertex_handle(*fh_it)) = true;
                }
            } else if (half_edge_to_pos_label.size() == 1) {
                auto heh = half_edge_to_pos_label[0];
                for (int i = 0; i < 4; ++i, heh = _m.next_halfedge_handle(heh)) {
                    if (i == 2) {
                        _m.property(vp_update_, _m.to_vertex_handle(heh)) =
                                false | _m.property(vp_update_, _m.to_vertex_handle(heh));
                    } else {
                        _m.property(vp_update_, _m.to_vertex_handle(heh)) = true;
                    }
                }
            }
        }
    }

//-----------------------------------------------------------------------------

    void cleanup(MyMesh &_m, const bool _update_points) {
        _m.remove_property(vp_pos_);
        _m.remove_property(ep_pos_);
        _m.remove_property(fp_pos_);

        if (_update_points) {
            _m.remove_property(vp_update_);
            _m.remove_property(pos_vert_id_);
            _m.remove_property(face_update_);
            _m.property(face_depths).clear();
        }

    }

//-----------------------------------------------------------------------------

    void all_quadrilaterals_test(MyMesh &_m) {
        if (_m.property(LabelSubdivision::quad_mesh)) {
            return;
        }
        _m.property(LabelSubdivision::quad_mesh) = true;
        FaceIter f_end = _m.faces_end();
        // Compute face centroid and check if _m is a quad mesh
        for (auto f_itr = _m.faces_begin(); f_itr != f_end; ++f_itr) {
            if (_m.valence(*f_itr) != 4)
                _m.property(LabelSubdivision::quad_mesh) = false;
        }
    }

    void catmull_clark_subdivide(MyMesh &_m, bool single_iteration) {
        // ASSUME mesh has face_depths, and v_label properties
        prepare(_m, true);
        set_vertex_updates(_m);

        // Do  subdivisions while there are positive vertex labels
        while (!_m.property(positive_label_set).empty()) {

            set_vertex_updates(_m);

            FaceIter f_end = _m.faces_end();
            // Compute face centroid
            for (auto f_itr = _m.faces_begin(); f_itr != f_end; ++f_itr) {
                Point centroid;
                _m.calc_face_centroid(*f_itr, centroid);
                _m.property(fp_pos_, *f_itr) = centroid;
            }

            EdgeIter e_end = _m.edges_end();
            // Compute position for new (edge-) vertices and store them in the edge property
            for (auto e_itr = _m.edges_begin(); e_itr != e_end; ++e_itr)
                compute_midpoint(_m, *e_itr, true);


            // compute new positions for old vertices
            for (auto v_itr = _m.vertices_begin(); v_itr != _m.vertices_end(); ++v_itr)
                update_vertex(_m, *v_itr);


            // Commit changes in geometry while updating vertex labels
            for (auto v_itr = _m.vertices_begin(); v_itr != _m.vertices_end(); ++v_itr) {
                _m.property(vertex_label, *v_itr) = (_m.property(vertex_label, *v_itr) == 0) ? 0 :
                                                    _m.property(vertex_label, *v_itr) - 1;
                if (_m.property(vertex_label, *v_itr) == 0) {
                    _m.property(positive_label_set).erase(*v_itr);
                }
                if (_m.property(vp_update_, *v_itr) == true) {
                    _m.set_point(*v_itr, _m.property(vp_pos_, *v_itr));
                }
            }

            // Split each edge at midpoint stored in edge property ep_pos_;
            // Attention! Creating new edges, hence make sure the loop ends correctly.
            for (auto e_itr = _m.edges_begin(); e_itr != e_end; ++e_itr) {
                split_edge(_m, *e_itr, true);
            }


            // Commit changes in topology and reconsitute consistency
            // Attention! Creating new picked_face_ids, hence make sure the loop ends correctly.
            for (auto f_itr = _m.faces_begin(); f_itr != f_end; ++f_itr) {
                if (_m.property(face_update_, *f_itr) == true) {
                    split_face(_m, *f_itr, true);
                }
            }
            if (single_iteration) break;
        }

#if defined(_DEBUG) || defined(DEBUG)
        // Now we have an consistent mesh!
        assert( OpenMesh::Utils::MeshCheckerT<MeshType>(_m).check() );
#endif

        _m.update_normals();

        cleanup(_m, true);
    }

//-----------------------------------------------------------------------------

    void split_face(MyMesh &_m, const FaceHandle &_fh, const bool _update_points) {
        /*
           Split an n-gon into n quads by connecting
           each vertex of fh to vh.

           - _fh will remain valid (it will become one of the quads)
           - the halfedge handles of the new quads will
           point to the old halfedges
         */

        // edge-vertex labels around face ( or some non positive labels around U_sub face)
        std::forward_list<int> labels;

        int valence = _m.valence(_fh) / 2;

        // new mesh vertex from face centroid
        VertexHandle vh = _m.add_vertex(_m.property(fp_pos_, _fh));


        HalfedgeHandle hend = _m.halfedge_handle(_fh);
        if (_update_points)
            labels.push_front(_m.property(vertex_label, _m.to_vertex_handle(hend)));

        // make sure hend is right for u_sub

        HalfedgeHandle hh = _m.next_halfedge_handle(hend);

        HalfedgeHandle hold = _m.new_edge(_m.to_vertex_handle(hend), vh);

        _m.set_next_halfedge_handle(hend, hold);
        _m.set_face_handle(hold, _fh);

        hold = _m.opposite_halfedge_handle(hold);

        for (size_t i = 1; i < valence; i++) {
            HalfedgeHandle hnext = _m.next_halfedge_handle(hh);

            FaceHandle fnew = _m.new_face();
            if (!_update_points) {
                if (_m.property(face_depths).find(_fh) != _m.property(face_depths).end())
                    _m.property(face_depths)[fnew] = _m.property(face_depths)[_fh];
            }

            _m.set_halfedge_handle(fnew, hh);

            HalfedgeHandle hnew = _m.new_edge(_m.to_vertex_handle(hnext), vh);

            if (_update_points) {
                labels.push_front(_m.property(vertex_label, _m.to_vertex_handle(hnext)));
            }

            _m.set_face_handle(hnew, fnew);
            _m.set_face_handle(hold, fnew);
            _m.set_face_handle(hh, fnew);
            _m.set_face_handle(hnext, fnew);

            _m.set_next_halfedge_handle(hnew, hold);
            _m.set_next_halfedge_handle(hold, hh);
            _m.set_next_halfedge_handle(hh, hnext);
            hh = _m.next_halfedge_handle(hnext);
            _m.set_next_halfedge_handle(hnext, hnew);

            hold = _m.opposite_halfedge_handle(hnew);
        }

        _m.set_next_halfedge_handle(hold, hh);
        _m.set_next_halfedge_handle(hh, hend);
        hh = _m.next_halfedge_handle(hend);
        _m.set_next_halfedge_handle(hend, hh);
        _m.set_next_halfedge_handle(hh, hold);

        _m.set_face_handle(hold, _fh);

        _m.set_halfedge_handle(vh, hold);

        // see paper for label generation rules
        if (_update_points) {
            if (std::all_of(labels.begin(), labels.end(), [](int i) { return i == 0; })) {

                _m.property(vertex_label, vh) = 0;
            } else {
                auto new_end = std::remove_if(labels.begin(), labels.end(), [](int i) { return i == 0; });
                _m.property(vertex_label, vh) = *std::min_element(labels.begin(), new_end);
                _m.property(positive_label_set).insert(vh);

            }
        }
    }


//-----------------------------------------------------------------------------

    void split_edge(MyMesh &_m, const EdgeHandle &_eh, const bool _update_points) {
        HalfedgeHandle heh = _m.halfedge_handle(_eh, 0);
        HalfedgeHandle opp_heh = _m.halfedge_handle(_eh, 1);


        HalfedgeHandle new_heh, opp_new_heh, t_heh;
        VertexHandle vh;
        VertexHandle vh1(_m.to_vertex_handle(heh));
        VertexHandle vh2(_m.to_vertex_handle(opp_heh));


        Point zero(0, 0, 0);

        int minimum;

        // don't split if labels are zero
        if (_update_points) {
            if ((_m.property(vp_update_, vh1) == false) || (_m.property(vp_update_, vh2) == false)) {
                return;
            } else {
                minimum = std::min(_m.property(vertex_label, vh1), _m.property(vertex_label, vh2));
            }
        }

        // new vertex
        vh = _m.new_vertex(zero);
        _m.set_point(vh, _m.property(ep_pos_, _eh));

        if (_update_points) {
            _m.property(vertex_label, vh) = minimum;
            if (minimum > 0)
                _m.property(positive_label_set).insert(vh);
        }

        // Re-link mesh entities
        if (_m.is_boundary(_eh)) {
            for (t_heh = heh;
                 _m.next_halfedge_handle(t_heh) != opp_heh;
                 t_heh = _m.opposite_halfedge_handle(_m.next_halfedge_handle(t_heh))) {}
        } else {
            for (t_heh = _m.next_halfedge_handle(opp_heh);
                 _m.next_halfedge_handle(t_heh) != opp_heh;
                 t_heh = _m.next_halfedge_handle(t_heh)) {}
        }

        new_heh = _m.new_edge(vh, vh1);
        opp_new_heh = _m.opposite_halfedge_handle(new_heh);
        _m.set_vertex_handle(heh, vh);

        _m.set_next_halfedge_handle(t_heh, opp_new_heh);
        _m.set_next_halfedge_handle(new_heh, _m.next_halfedge_handle(heh));
        _m.set_next_halfedge_handle(heh, new_heh);
        _m.set_next_halfedge_handle(opp_new_heh, opp_heh);

        if (_m.face_handle(opp_heh).is_valid()) {
            _m.set_face_handle(opp_new_heh, _m.face_handle(opp_heh));
            if (!_update_points || (_update_points && (_m.to_vertex_handle(opp_heh) ==
                                                       _m.property(pos_vert_id_, _m.face_handle(opp_heh)))))
                _m.set_halfedge_handle(_m.face_handle(opp_new_heh), opp_new_heh);
        }

        if (_m.face_handle(heh).is_valid()) {
            _m.set_face_handle(new_heh, _m.face_handle(heh));
            if (!_update_points ||
                (_update_points && (_m.to_vertex_handle(new_heh) == _m.property(pos_vert_id_, _m.face_handle(heh)))))
                _m.set_halfedge_handle(_m.face_handle(heh), heh);
        }

        _m.set_halfedge_handle(vh, new_heh);
        _m.set_halfedge_handle(vh1, opp_new_heh);

        // Never forget this, when playing with the topology
        _m.adjust_outgoing_halfedge(vh);
        _m.adjust_outgoing_halfedge(vh1);
    }

//-----------------------------------------------------------------------------

    void
    compute_midpoint(MyMesh &_m, const EdgeHandle &_eh,
                     const bool _update_points) {
        HalfedgeHandle heh, opp_heh;

        heh = _m.halfedge_handle(_eh, 0);
        opp_heh = _m.halfedge_handle(_eh, 1);

        Point pos(_m.point(_m.to_vertex_handle(heh)));

        pos += _m.point(_m.to_vertex_handle(opp_heh));

        // boundary edge: just average vertex positions
        // this yields the [1/2 1/2] mask
        if (_m.is_boundary(_eh) || !_update_points) {
            pos *= 0.5;
        }

            // inner edge: add neighbouring Vertices to sum
            // this yields the [1/16 1/16; 3/8 3/8; 1/16 1/16] mask
        else {
            pos += _m.property(fp_pos_, _m.face_handle(heh));
            pos += _m.property(fp_pos_, _m.face_handle(opp_heh));
            pos *= 0.25;
        }

        _m.property(ep_pos_, _eh) = pos;
    }

//-----------------------------------------------------------------------------

    void
    update_vertex(MyMesh &_m, const VertexHandle &_vh) {
        Point pos(0.0, 0.0, 0.0);

        if (_m.is_boundary(_vh)) {
            Normal Vec;
            pos = _m.point(_vh);
            VertexEdgeIter ve_itr;
            for (ve_itr = _m.ve_iter(_vh); ve_itr.is_valid(); ++ve_itr)
                if (_m.is_boundary(*ve_itr))
                    pos += _m.property(ep_pos_, *ve_itr);
            pos /= 3.0;
        } else // inner vertex
        {
            double valence(0.0);
            VOHIter voh_it = _m.voh_iter(_vh);
            for (; voh_it.is_valid(); ++voh_it) {
                pos += _m.point(_m.to_vertex_handle(*voh_it));
                valence += 1.0;
            }
            pos /= valence * valence;

            VertexFaceIter vf_itr;
            Point Q(0, 0, 0);

            for (vf_itr = _m.vf_iter(_vh); vf_itr.is_valid(); ++vf_itr) //, neigboring_faces += 1.0 )
            {
                Q += _m.property(fp_pos_, *vf_itr);
            }

            Q /= valence * valence;//neigboring_faces;

            pos += _m.point(_vh) * (valence - 2.0) / valence + Q;
        }

        _m.property(vp_pos_, _vh) = pos;
    }

//-----------------------------------------------------------------------------

    void eliminate_illegal_faces(MyMesh &_m) {
        _m.add_property(illegal_valence_);
        _m.add_property(potential_new_illegal_count_);


        // two comparison lambda functions for standard library functions
        auto cmp_potential_new = [&](VertexHandle a, VertexHandle b) {
            return (_m.property(potential_new_illegal_count_, a) < _m.property(potential_new_illegal_count_, b));
        };
        auto cmp_illegal_valence = [&](MyMesh::VertexHandle a, MyMesh::VertexHandle b) {
            return _m.property(illegal_valence_, a) < _m.property(illegal_valence_, b);
        };

        // fill illegal vertices set
        std::set<MyMesh::VertexHandle> illegals;
        populate_illegal_vertex_set(illegals, _m);

        // there might not be any illegals!
        if (illegals.empty()) return;

        // populate illegal vertex attributes
        populate_illegal_vertex_attributes(illegals, _m);

        while (!illegals.empty()) {
            // sort based on newly computed vertex attributes
            auto illegal_ordered = std::multiset<MyMesh::VertexHandle, decltype(cmp_illegal_valence)>(
                    cmp_illegal_valence);
            illegal_ordered.insert(illegals.begin(), illegals.end());

            // create set of illegal vertices that have one edge in illegal graph
            std::set<VertexHandle> _1v_illegals;
            std::copy_if(illegals.begin(), illegals.end(), std::inserter(_1v_illegals, _1v_illegals.begin()),
                         [&](VertexHandle a) { return (_m.property(illegal_valence_, a) == 1); });

            VertexHandle choice;
            std::vector<VertexHandle> choices;
            // If the number of illegal vertices with valence==1 is not zero,
            if (!_1v_illegals.empty()) {
                std::set<VertexHandle> _1v_adjacent;
                for (auto v : _1v_illegals)
                    _1v_adjacent.insert(_m.cvv_ccwbegin(v), _m.cvv_ccwend(v));

                std::set<VertexHandle> pool;
                std::set_intersection(_1v_adjacent.begin(), _1v_adjacent.end(),
                                      illegals.begin(), illegals.end(),
                                      std::inserter(pool, pool.begin()));


                auto ordered_pool = std::multiset<VertexHandle, decltype(cmp_illegal_valence)>(cmp_illegal_valence);
                ordered_pool.insert(pool.begin(), pool.end());

                auto first = ordered_pool.find(*ordered_pool.rbegin());
                choice = *std::min_element(first, ordered_pool.end(), cmp_potential_new);
            }
                // Otherwise
            else {
                auto pair = illegal_ordered.equal_range(*illegal_ordered.rbegin());
                choice = *std::min_element(pair.first, pair.second, cmp_potential_new);
            }


            // change chosen vertex label to 1
            _m.property(LabelSubdivision::vertex_label, choice) = 1;
            _m.property(LabelSubdivision::positive_label_set).insert(choice);

            // reset illegal vert attributes
            for (auto v_it = _m.vertices_begin(); v_it != _m.vertices_end(); ++v_it) {
                _m.property(illegal_valence_, *v_it) = 0;
                _m.property(potential_new_illegal_count_, *v_it) = 0;
            }

            // fill illegal vertices set
            illegals.clear();
            populate_illegal_vertex_set(illegals, _m);

            // there might not be any illegals!
            if (illegals.empty()) return;

            populate_illegal_vertex_attributes(illegals, _m);

        }
        _m.remove_property(illegal_valence_);
        _m.remove_property(potential_new_illegal_count_);
    }


    void populate_illegal_vertex_set(std::set<VertexHandle> &illegals, MyMesh &_m) {

        auto pfd = _m.property(face_depths);
        std::set<FaceHandle> visited;
        std::vector<MyMesh::VertexHandle> zero_v;
        zero_v.resize(4);
        for (auto pos_depth : pfd) {
            for (auto ff_it = _m.cff_begin(pos_depth.first); ff_it.is_valid(); ++ff_it) {
                auto result_pair = visited.insert(*ff_it);
                if (result_pair.second == false) break;
                zero_v.clear();
                for (auto fv_it = _m.cfv_iter(*ff_it); fv_it.is_valid(); ++fv_it) {
                    if (_m.property(LabelSubdivision::vertex_label, *fv_it) == 0)
                        zero_v.push_back(*fv_it);
                }
                if (zero_v.size() == 2) {
                    illegals.insert(zero_v[0]);
                    illegals.insert(zero_v[1]);
                }

            }
        }
    }


    void populate_illegal_vertex_attributes(std::set<VertexHandle> &illegals, MyMesh &_m) {
        for (auto v : illegals) {
            // compute degree among illegal verticies
            for (auto vv_it = _m.vv_iter(v); vv_it.is_valid(); ++vv_it) {
                if (illegals.find(*vv_it) != illegals.end())
                    _m.property(illegal_valence_, v) += 1;
            }

            // compute the number of possible new illegal vertices
            for (auto vf_it = _m.cvf_ccwiter(v); vf_it.is_valid(); ++vf_it) {
                int zero_count = 0;
                for (auto fvIt = _m.cfv_ccwiter(*vf_it); fvIt.is_valid(); ++fvIt) {
                    if (_m.property(LabelSubdivision::vertex_label, *fvIt) == 0)
                        zero_count += 1;
                }
                if (zero_count == 3)
                    _m.property(potential_new_illegal_count_, v) += 1;
            }
        }

    }

}
