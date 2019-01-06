
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include "LabelSubdivision.h"
#include "Object.h"

#include "GL/gl3w.h"
#include <iostream>


namespace sb7 {

    Object::Object()
            : position_buffer(0),
              normals_buffer(0),
              label_pos_buffer(0),
              ssbo(0),
              vao(0) {
    }

    Object::~Object() {

    }

    void Object::read(const char *filename) {
        OpenMesh::IO::Options opt;
        if (!OpenMesh::IO::read_mesh(original_mesh, filename, opt)) {
            std::cout << "Error loading mesh from file " << filename << std::endl;
        }
        if (!opt.check(OpenMesh::IO::Options::VertexNormal)) {
            original_mesh.request_face_normals();
            original_mesh.request_vertex_normals();
            original_mesh.update_normals();
        }
        original_mesh.add_property(LabelSubdivision::face_depths);
        original_mesh.add_property(LabelSubdivision::vertex_label);
        original_mesh.add_property(LabelSubdivision::quad_mesh);
        original_mesh.add_property(LabelSubdivision::positive_label_set);

        last_mesh = original_mesh;

        mesh = original_mesh;

        update_gpu_data();
    }

    void Object::undo() {
        mesh = last_mesh;
        zero_labels_and_depths();
        update_gpu_data();
    }

    void Object::revert_to_original() {
        mesh = original_mesh;
        zero_labels_and_depths();
        update_gpu_data();
    }

    void Object::solve_mesh_labels() {
        LabelSubdivision::all_quadrilaterals_test(mesh);
        if (mesh.property(LabelSubdivision::quad_mesh)==false){
            increase_all_face_depths();
            update_mesh_properties();
        }
        LabelSubdivision::eliminate_illegal_faces(mesh);
        update_gpu_data();
    }

    void Object::label_driven_catmull_clark_subdivision(bool single_itr) {
        last_mesh = mesh;
        solve_mesh_labels();
        LabelSubdivision::catmull_clark_subdivide(mesh, single_itr);
        update_gpu_data();
    }

    void Object::orient_camera(GLfloat *center, GLfloat &size) {
        auto vertex_positions = (GLfloat *) mesh.points();

        float min[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
        float max[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
        for (size_t i = 0; i < mesh.n_vertices(); ++i) {
            for (int j = 0; j < 3; ++j) {
                float v = vertex_positions[i * 3 + j];
                min[j] = std::min(min[j], v);
                max[j] = std::max(max[j], v);
            }
        }
        for (int j = 0; j < 3; ++j) {
            center[j] = (min[j] + max[j]) * 0.5f;
            size += (max[j] - min[j]) * (max[j] - min[j]);
        }
        size = sqrtf(size);
    }


    void Object::increase_face_depth(int id) {
        mesh.property(LabelSubdivision::face_depths)[mesh.face_handle(id)] += 1;
    }

    void Object::decrease_face_depth(int id) {
        if (mesh.property(LabelSubdivision::face_depths).find(mesh.face_handle(id)) !=
            mesh.property(LabelSubdivision::face_depths).end()) {
            mesh.property(LabelSubdivision::face_depths)[mesh.face_handle(id)] -= 1;
            if (mesh.property(LabelSubdivision::face_depths)[mesh.face_handle(id)] <= 0) {
                mesh.property(LabelSubdivision::face_depths).erase(mesh.face_handle(id));
                set_fv_labels_to_zero(mesh.face_handle(id));
            }
        }
    }

    void Object::decrease_all_face_depths() {
        for (auto f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
            decrease_face_depth(f_it->idx());
        }
    }

    void Object::increase_all_face_depths() {
        for (auto f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
            increase_face_depth(f_it->idx());
        }
    }

    void Object::increase_curved_face_depth() {
        float angle_limit_radians = vmath::radians(angle_limit_degrees);
        for (auto f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
            float max_angle = 0;
            for (auto fe_it = mesh.fe_iter(*f_it); fe_it.is_valid(); ++fe_it) {
                auto a = mesh.calc_dihedral_angle(*fe_it);
                max_angle = std::max(std::abs(mesh.calc_dihedral_angle(*fe_it)), max_angle);
            }
            if (max_angle > angle_limit_radians)
                increase_face_depth(f_it->idx());
        }
    }

    void Object::set_fv_labels_to_zero(MyMesh::FaceHandle fh) {
        for (auto fv_it = mesh.fv_iter(fh); fv_it.is_valid(); ++fv_it) {
            mesh.property(LabelSubdivision::vertex_label, *fv_it) = 0;
            mesh.property(LabelSubdivision::positive_label_set).erase(*fv_it);
        }
    }

    void Object::vertex_label_data_to_gpu() {
        fdepth_to_vlabels();
        non_zero_labels_pos.clear();
        for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
            if (mesh.property(LabelSubdivision::vertex_label, *v_it) != 0) {
                non_zero_labels_pos.push_back(mesh.point(*v_it)[0]);
                non_zero_labels_pos.push_back(mesh.point(*v_it)[1]);
                non_zero_labels_pos.push_back(mesh.point(*v_it)[2]);
            }
        }

        glBindVertexArray(vao);
        glGenBuffers(1, &label_pos_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, label_pos_buffer);
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(GLfloat) * non_zero_labels_pos.size(),
                     non_zero_labels_pos.data(),
                     GL_STATIC_DRAW);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(2);

    }

    void Object::update_mesh_properties() {
        fdepth_to_vlabels();
        face_depth_data_to_gpu();
        vertex_label_data_to_gpu();
    }

    void Object::face_depth_data_to_gpu() {
        if (!wireframe) {
            vertex_label_data.resize(mesh.n_faces());
            std::fill(vertex_label_data.begin(), vertex_label_data.end(), 0);
            for (auto elem : mesh.property(LabelSubdivision::face_depths)) {
                vertex_label_data[elem.first.idx()] = static_cast<unsigned int>(elem.second);
            }
        } else {
            vertex_label_data.resize(mesh.n_faces());
            std::fill(vertex_label_data.begin(), vertex_label_data.end(), 10);
        }

        glGenBuffers(1, &ssbo);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
        glBufferData(GL_SHADER_STORAGE_BUFFER, mesh.n_faces() * sizeof(GLuint), vertex_label_data.data(),
                     GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, ssbo);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); // unbind
    }


    void Object::update_gpu_data() {
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        face_elem_count.resize(mesh.n_faces());
        face_index.resize(mesh.n_faces());
        render_vert_count = 0;
        for (auto f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
            render_vert_count += ((mesh.valence(*f_it) - 2) * 3);
        }
        vertex_pos.resize(render_vert_count * 3);
        normals.resize(render_vert_count * 3);
        int i = 0;
        int fc = 0;
        for (auto f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
            face_index[fc] = i / 3;
            auto fv_it = mesh.cfv_ccwiter(*f_it);
            MyMesh::ConstFaceVertexCCWIter fvStart;
            fvStart = fv_it;

            for (int j = 0; j < 3; ++j) {
                vertex_pos[i] = mesh.point(*fv_it)[0];
                vertex_pos[i + 1] = mesh.point(*fv_it)[1];
                vertex_pos[i + 2] = mesh.point(*fv_it)[2];
                if (smooth_shade) {
                    normals[i] = mesh.normal(*fv_it)[0];
                    normals[i + 1] = mesh.normal(*fv_it)[1];
                    normals[i + 2] = mesh.normal(*fv_it)[2];
                } else {
                    normals[i] = mesh.normal(*f_it)[0];
                    normals[i + 1] = mesh.normal(*f_it)[1];
                    normals[i + 2] = mesh.normal(*f_it)[2];
                }
                i += 3;
                fv_it++;
            }
            if (!fv_it.is_valid()) {
                face_elem_count[fc] = (i / 3) - face_index[fc];
                ++fc;
                continue;
            }

            while (fv_it.is_valid()) {
                --fv_it;
                for (int j = 0; j < 2; ++j) {
                    vertex_pos[i] = mesh.point(*fv_it)[0];
                    vertex_pos[i + 1] = mesh.point(*fv_it)[1];
                    vertex_pos[i + 2] = mesh.point(*fv_it)[2];
                    if (smooth_shade) {
                        normals[i] = mesh.normal(*fv_it)[0];
                        normals[i + 1] = mesh.normal(*fv_it)[1];
                        normals[i + 2] = mesh.normal(*fv_it)[2];
                    } else {
                        normals[i] = mesh.normal(*f_it)[0];
                        normals[i + 1] = mesh.normal(*f_it)[1];
                        normals[i + 2] = mesh.normal(*f_it)[2];
                    }
                    i += 3;
                    ++fv_it;
                }
                vertex_pos[i] = mesh.point(*fvStart)[0];
                vertex_pos[i + 1] = mesh.point(*fvStart)[1];
                vertex_pos[i + 2] = mesh.point(*fvStart)[2];
                if (smooth_shade) {
                    normals[i] = mesh.normal(*fv_it)[0];
                    normals[i + 1] = mesh.normal(*fv_it)[1];
                    normals[i + 2] = mesh.normal(*fv_it)[2];
                } else {
                    normals[i] = mesh.normal(*f_it)[0];
                    normals[i + 1] = mesh.normal(*f_it)[1];
                    normals[i + 2] = mesh.normal(*f_it)[2];
                }
                i += 3;
            }
            face_elem_count[fc] = (i / 3) - face_index[fc];
            ++fc;
        }


        glGenBuffers(1, &position_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, position_buffer);
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(GLfloat) * render_vert_count * 3,
                     vertex_pos.data(),
                     GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);

        glGenBuffers(1, &normals_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, normals_buffer);
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(GLfloat) * render_vert_count * 3,
                     normals.data(),
                     GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);


        face_depth_data_to_gpu();
        vertex_label_data_to_gpu();
    }


    void Object::free() {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &position_buffer);
        glDeleteBuffers(1, &normals_buffer);

        vao = 0;
        position_buffer = 0;
        normals_buffer = 0;
        label_pos_buffer = 0;
    }

    void Object::render_object(GLuint prog) {
        if (wireframe && (prog != vertex_program)) {
            glMultiDrawArrays(GL_LINE_LOOP, face_index.data(), face_elem_count.data(), mesh.n_faces());
        }
        if (wireframe && (prog == vertex_program)) {
            glDrawArrays(GL_POINTS, 0, static_cast<uint>(non_zero_labels_pos.size() / 3));
        }
        if (!wireframe && (prog != vertex_program)) {
            glMultiDrawArrays(GL_TRIANGLES, face_index.data(), face_elem_count.data(), mesh.n_faces());
        }
    }

    void Object::fdepth_to_vlabels() {
        for (auto elem : mesh.property(LabelSubdivision::face_depths)) {
            for (auto fv_it = mesh.fv_iter(elem.first); fv_it.is_valid(); ++fv_it) {
                int new_label = std::max(mesh.property(LabelSubdivision::vertex_label, *fv_it), elem.second);
                mesh.property(LabelSubdivision::vertex_label, *fv_it) = new_label;
                if (new_label > 0)
                    mesh.property(LabelSubdivision::positive_label_set).insert(*fv_it);
            }
        }
    }

    void Object::zero_labels_and_depths() {
        mesh.property(LabelSubdivision::face_depths).clear();
        mesh.property(LabelSubdivision::positive_label_set).clear();
        for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
            mesh.property(LabelSubdivision::vertex_label, *v_it) = 0;
        }
    }

}
