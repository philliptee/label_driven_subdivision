#ifndef __OBJECT_H__
#define __OBJECT_H__

#include <vector>
#include <vmath.h>


#include <GL/glcorearb.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include "LabelSubdivision.h"


namespace sb7 {
    typedef OpenMesh::PolyMesh_ArrayKernelT<> MyMesh;

    class Object {
    public:
        Object();

        ~Object();

        void render_object(GLuint program);

        void increase_face_depth(int);

        void decrease_face_depth(int);

        void decrease_all_face_depths();

        void increase_all_face_depths();

        void increase_curved_face_depth();

        void update_mesh_properties();

        void face_depth_data_to_gpu();

        void vertex_label_data_to_gpu();

        void set_fv_labels_to_zero(MyMesh::FaceHandle);

        void zero_labels_and_depths();

        void fdepth_to_vlabels();

        void solve_mesh_labels();

        void read(const char *filename);

        void update_gpu_data();

        void undo();

        void revert_to_original();

        void label_driven_catmull_clark_subdivision(bool);

        void orient_camera(float *center, float &size);

        void free();

        bool wireframe = false;
        bool smooth_shade = false;
        float angle_limit_degrees = 30.0f;
        GLuint vertex_program;


    private:
        GLuint vao;
        GLuint position_buffer;
        GLuint normals_buffer;
        GLuint label_pos_buffer;
        GLuint ssbo;

        MyMesh mesh;
        MyMesh last_mesh;
        MyMesh original_mesh;

        std::vector<GLfloat> vertex_pos;
        std::vector<GLfloat> normals;

        std::vector<GLsizei> face_elem_count;
        std::vector<GLint> face_index;

        std::vector<GLuint> vertex_label_data;
        std::vector<GLfloat> non_zero_labels_pos;
        int render_vert_count;


    };

}

#endif

