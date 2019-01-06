#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include <sb7.h>
#include <vmath.h>
#include "Object.h"
#include <shader.h>

#include <iostream>
#include <fstream>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

class subdivision_app : public sb7::application {

protected:
    void init() {
        static const char title[] = "CPSC 689 : Label-Driven Subdivision Painting";

        sb7::application::init();

        memcpy(info.title, title, sizeof(title));
    }

    virtual void onMouseButton(GLFWwindow *w, int button, int action) {
        // record mouse button state
        if (!ImGui::GetIO().WantCaptureMouse) {
            if (button < 3) {
                g_mbutton[button] = (action == GLFW_PRESS);
            }

            // increase face depth property for faces under mouse click
            if (!glfwGetKey(w, GLFW_KEY_LEFT_ALT)) {
                if (g_mbutton[0] && !g_mbutton[1] && !g_mbutton[2] && !object.wireframe) {
                    brush_pick(w, glfwGetKey(w, GLFW_KEY_LEFT_SHIFT));
                }
            }

            // reset history of faces under mouse and faces depths changed while LMB has been held
            if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_RELEASE) {
                ids_this_brush_stroke.clear();
                prev_ids_under_mouse.clear();
            }
        }
    }


    virtual void onMouseMove(GLFWwindow *w, int x, int y) {

        // camera movement
        if (glfwGetKey(w, GLFW_KEY_LEFT_ALT)) {
            if (g_mbutton[0] && !g_mbutton[1] && !g_mbutton[2]) {
                // orbit
                g_rotate[0] += x - g_prev_x;
                g_rotate[1] += y - g_prev_y;
            } else if (!g_mbutton[0] && !g_mbutton[1] && g_mbutton[2]) {
                // pan
                g_pan[0] -= g_dolly * (x - g_prev_x) / info.windowWidth;
                g_pan[1] += g_dolly * (y - g_prev_y) / info.windowHeight;
            } else if ((g_mbutton[0] && !g_mbutton[1] && g_mbutton[2]) ||
                       (!g_mbutton[0] && g_mbutton[1] && !g_mbutton[2])) {
                // dolly
                g_dolly -= g_dolly * 0.01f * (x - g_prev_x);
                if (g_dolly <= 0.01) g_dolly = 0.01f;
            }
        }
            // increase face depth property for faces under mouse click
        else {
            if (g_mbutton[0] && !g_mbutton[1] && !g_mbutton[2] && !object.wireframe) {
                brush_pick(w, glfwGetKey(w, GLFW_KEY_LEFT_SHIFT));
            }
        }

        // record mouse drag
        g_prev_x = x;
        g_prev_y = y;
    }

    void gui() {
        ImGui::Begin("Controls");

        // file selection
        if (ImGui::BeginCombo("Object Dir", current_filename.c_str()))
        {
            for (auto item : object_filenames)
            {
                bool is_selected = (current_filename == item);
                if (ImGui::Selectable(item.c_str(), is_selected)) {
                    current_filename = item.c_str();
                    object.read(current_filename.c_str());
                    object.orient_camera(g_center, g_dolly);
                }
                if (is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        ImGui::Text("Rotate, pan, and dolly with alt + mouse buttons.");

        if (ImGui::Checkbox("Wireframe ", &object.wireframe))
            object.update_gpu_data();

        ImGui::SameLine();
        if (ImGui::Checkbox("Smooth Shade", &object.smooth_shade))
            object.update_gpu_data();

        if (ImGui::SliderInt("Brush Size", &g_brush_size, 1, 64))
            pick_brush_data.resize(g_brush_size * g_brush_size);

        if (ImGui::Button("+1 All")){
            object.increase_all_face_depths();
            object.update_mesh_properties();
        }

        ImGui::SameLine();
        if (ImGui::Button("-1 All")){
            object.decrease_all_face_depths();
            object.update_mesh_properties();
        }

        ImGui::SameLine();
        if (ImGui::Button("Zero All")) {
            object.zero_labels_and_depths();
            object.update_mesh_properties();
        }

        if (ImGui::Button("+1 Visible")){
            pick_visible();
            object.update_mesh_properties();
        }

        ImGui::SameLine();
        if (ImGui::Button("+1 High Curvature")){
            object.increase_curved_face_depth();
            object.update_mesh_properties();
        }

        ImGui::SliderFloat("Maximum Angle", &object.angle_limit_degrees, 0.0f, 90.0f);

        if (ImGui::Button("Remove Illegal Vertices"))
            object.solve_mesh_labels();

        if (ImGui::Button("Subdivide"))
            object.label_driven_catmull_clark_subdivision(false);

        ImGui::SameLine();
        if (ImGui::Button("Subdivide (One Iteration)"))
            object.label_driven_catmull_clark_subdivision(true);

        if (ImGui::Button("Undo"))
            object.undo();

        ImGui::SameLine();
        if (ImGui::Button("Revert To Original Mesh"))
            object.revert_to_original();

        ImGui::End();

    }

    virtual void startup() {
        std::string path("objects");
        std::string ext(".obj");
        for(auto& p: fs::recursive_directory_iterator(path))
        {
            if(p.path().extension() == ext)
                object_filenames.push_back(p.path());
        }
        current_filename = object_filenames[0];
        object.read(current_filename.c_str());
        object.orient_camera(g_center, g_dolly);

        // allocate pixel picking brush data space
        pick_brush_data.resize(g_brush_size * g_brush_size);

        // imgui commands
        bool err = gl3wInit() != 0;
        if (err) { fprintf(stderr, "Failed to initialize OpenGL loader!\n"); }
        const char* glsl_version = "#version 430";
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;
        // Setup Platform/Renderer bindings
        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init(glsl_version);

//         Setup Style
        ImGui::StyleColorsDark();
        ImGui::GetStyle().ScaleAllSizes(1.0);
        ImGui::GetIO().FontGlobalScale = 1.0;

        load_shaders();

        glDepthFunc(GL_LEQUAL);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glPointSize(10);
    }

    virtual void render(double currentTime) {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        gui();
//        ImGui::ShowDemoWindow();

        // set background and depth buffer
        static const GLfloat gray[] = {0.2f, 0.2f, 0.2f, 1.0f};
        static const GLfloat ones[] = {1.0f};
        glClearBufferfv(GL_COLOR, 0, gray);
        glClearBufferfv(GL_DEPTH, 0, ones);

        // set viewport size
        glViewport(0, 0, info.windowWidth, info.windowHeight);

        // compute MV & P matrices
        vmath::mat4 proj_matrix = vmath::perspective(45.0f, (float) info.windowWidth / (float) info.windowHeight, 0.1f,
                                                     500.0f);
        vmath::mat4 mv_matrix = vmath::translate(0.0f, 0.0f, 0.0f) *
                                vmath::translate(-g_pan[0], -g_pan[1], -g_dolly) *
                                vmath::rotate(g_rotate[1], 1.0f, 0.0f, 0.0f) *
                                vmath::rotate(g_rotate[0], 0.0f, 1.0f, 0.0f) *
                                vmath::translate(-g_center[0], -g_center[1], -g_center[2]);

        // mesh rendering
        glUseProgram(face_program);
        glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, mv_matrix);
        glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, proj_matrix);
        glShaderStorageBlockBinding(face_program, uniforms.picked_face_ids, 3);
        object.render_object(face_program);

        // positive vertex label rendering
        glUseProgram(vertex_program);
        glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, mv_matrix);
        glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, proj_matrix);
        object.render_object(vertex_program);

        // imgui commands
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

    void brush_pick(GLFWwindow *w, bool left_shift_pressed) {
        // set background as pure white
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        vmath::mat4 proj_matrix = vmath::perspective(45.0f, (float) info.windowWidth / (float) info.windowHeight, 0.1f,
                                                     500.0f);
        vmath::mat4 mv_matrix = vmath::translate(0.0f, 0.0f, 0.0f) *
                                vmath::translate(-g_pan[0], -g_pan[1], -g_dolly) *
                                vmath::rotate(g_rotate[1], 1.0f, 0.0f, 0.0f) *
                                vmath::rotate(g_rotate[0], 0.0f, 1.0f, 0.0f) *
                                vmath::translate(-g_center[0], -g_center[1], -g_center[2]);


        // render faces as unique colors to select them
        glUseProgram(faceID_program);
        glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, mv_matrix);
        glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, proj_matrix);
        object.render_object(face_program);

        glFlush();
        glFinish();

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        std::set<int> ids_under_mouse;

        // real nxn block of pixes around mouse
        glReadnPixels(g_prev_x - (g_brush_size / 2),
                      info.windowHeight - (g_prev_y + (g_brush_size / 2)),
                      g_brush_size, g_brush_size,
                      GL_RGBA, GL_UNSIGNED_BYTE, pick_brush_data.size() * 4, pick_brush_data.data());

        // build set of picked face ids, approximate circular brush
        for (int i = 0; i < g_brush_size; i++) {
            for (int j = 0; j < g_brush_size; j++) {
                float x = (i - (g_brush_size / 2));
                float y = (j - (g_brush_size / 2));
                if (std::sqrt(x * x + y * y) < g_brush_size / 2.0f) {
                    int pickedID =
                            pick_brush_data[i * g_brush_size + j][0] +
                            pick_brush_data[i * g_brush_size + j][1] * 256 +
                            pick_brush_data[i * g_brush_size + j][2] * 256 * 256;
                    ids_under_mouse.insert(pickedID);
                }
            }
        }

        // update mesh properties if mouse is over new data
        if (prev_ids_under_mouse != ids_under_mouse) {
            prev_ids_under_mouse = ids_under_mouse;

            // find set of picked face ids that haven't yet had their face depth changed
            std::set<int> new_ids;
            std::set_difference(ids_under_mouse.begin(), ids_under_mouse.end(),
                                ids_this_brush_stroke.begin(), ids_this_brush_stroke.end(),
                                std::inserter(new_ids, new_ids.begin()));

            ids_this_brush_stroke.insert(new_ids.begin(), new_ids.end());

            // apply face depth modifications
            for (auto id : new_ids) {
                if (id == 0x00ffffff) {
                } else {
                    if (!left_shift_pressed) {
                        object.increase_face_depth(id);
                    } else {
                        object.decrease_face_depth(id);
                    }
                }
            }
            object.update_mesh_properties();
        }
    }

    void pick_visible() {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        vmath::mat4 proj_matrix = vmath::perspective(45.0f, (float) info.windowWidth / (float) info.windowHeight, 0.1f,
                                                     500.0f);
        vmath::mat4 mv_matrix = vmath::translate(0.0f, 0.0f, 0.0f) *
                                vmath::translate(-g_pan[0], -g_pan[1], -g_dolly) *
                                vmath::rotate(g_rotate[1], 1.0f, 0.0f, 0.0f) *
                                vmath::rotate(g_rotate[0], 0.0f, 1.0f, 0.0f) *
                                vmath::translate(-g_center[0], -g_center[1], -g_center[2]);


        // render faces as unique colors to select them
        glUseProgram(faceID_program);
        glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, mv_matrix);
        glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, proj_matrix);
        object.render_object(face_program);

        glFlush();
        glFinish();

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        std::set<int> ids_in_view;

        // real nxn block of pixes around mouse
        pick_brush_data.resize(info.windowWidth * info.windowHeight);
        glReadnPixels(0, 0, info.windowWidth, info.windowHeight,
                      GL_RGBA, GL_UNSIGNED_BYTE, pick_brush_data.size() * 4, pick_brush_data.data());

        // build set of picked face ids
        for (auto pixel : pick_brush_data) {
            int pickedID =
                    pixel[0] +
                    pixel[1] * 256 +
                    pixel[2] * 256 * 256;
            ids_in_view.insert(pickedID);
        }
        pick_brush_data.resize(g_brush_size * g_brush_size);

        // update mesh properties if mouse is over new data
        // apply face depth modifications
        for (auto id : ids_in_view) {
            if (id == 0x00ffffff) {
            } else {
                object.increase_face_depth(id);
            }
        }
        object.update_mesh_properties();
    }

    virtual void shutdown() {
        glDeleteProgram(face_program);
        glDeleteProgram(faceID_program);
        glDeleteProgram(vertex_program);
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

    void load_shaders() {
        // mesh_face program
        if (face_program)
            glDeleteProgram(face_program);

        GLuint vs = sb7::shader::load("shaders/mesh_faces.vs.glsl", GL_VERTEX_SHADER);
        GLuint fs = sb7::shader::load("shaders/mesh_faces.fs.glsl", GL_FRAGMENT_SHADER);

        face_program = glCreateProgram();
        glAttachShader(face_program, vs);
        glAttachShader(face_program, fs);
        glLinkProgram(face_program);

        glDeleteShader(vs);
        glDeleteShader(fs);

        uniforms.mv_matrix = glGetUniformLocation(face_program, "mv_matrix");
        uniforms.proj_matrix = glGetUniformLocation(face_program, "proj_matrix");
        uniforms.picked_face_ids == glGetProgramResourceIndex(face_program, GL_SHADER_STORAGE_BLOCK, "shader_data");

        // face_ID program
        if (faceID_program)
            glDeleteProgram(faceID_program);

        GLuint fid_vs = sb7::shader::load("shaders/face_IDs.vs.glsl", GL_VERTEX_SHADER);
        GLuint fid_fs = sb7::shader::load("shaders/face_IDs.fs.glsl", GL_FRAGMENT_SHADER);

        faceID_program = glCreateProgram();
        glAttachShader(faceID_program, fid_vs);
        glAttachShader(faceID_program, fid_fs);
        glLinkProgram(faceID_program);

        glDeleteShader(fid_vs);
        glDeleteShader(fid_fs);

        // vertex label program
        if (vertex_program)
            glDeleteProgram(vertex_program);

        GLuint vl_vs = sb7::shader::load("shaders/vertex_labels.vs.glsl", GL_VERTEX_SHADER);
        GLuint vl_fs = sb7::shader::load("shaders/vertex_labels.fs.glsl", GL_FRAGMENT_SHADER);

        vertex_program = glCreateProgram();
        glAttachShader(vertex_program, vl_vs);
        glAttachShader(vertex_program, vl_fs);
        glLinkProgram(vertex_program);


        object.vertex_program = vertex_program;

        glDeleteShader(vl_vs);
        glDeleteShader(vl_fs);

    }


    virtual void onKey(int key, int action) {
        if (action) {
            switch (key) {
                case 'C':
                    object.label_driven_catmull_clark_subdivision(false);
                    break;
                case 'X':
                    object.label_driven_catmull_clark_subdivision(true);
                    break;
                case 'Z':
                    object.zero_labels_and_depths();
                    object.update_mesh_properties();
                    break;
                case 'B':
                    object.undo();
                    break;
                case 'A':
                    object.increase_all_face_depths();
                    object.update_mesh_properties();
                    break;
                case 'J':
                    object.increase_curved_face_depth();
                    object.update_mesh_properties();
                    break;
                case 'K':
                    pick_visible();
                    object.update_mesh_properties();
                    break;
                case 'O':
                    object.revert_to_original();
                    break;
                case 'S':
                    object.solve_mesh_labels();
                    break;
                case 'T':
                    object.smooth_shade = !object.smooth_shade;
                    object.update_gpu_data();
                    break;
                case 'W':
                    object.wireframe = !object.wireframe;
                    object.update_gpu_data();
                    break;
                case GLFW_KEY_SPACE:
                    object.decrease_all_face_depths();
                    object.update_mesh_properties();
                    break;
            }
        }
    }

protected:
    GLuint face_program;
    GLuint faceID_program;
    GLuint vertex_program;

    struct {
        GLint mv_matrix;
        GLint proj_matrix;
        GLint picked_face_ids;
    } uniforms;

    float g_rotate[2] = {0, 0},
            g_dolly = 5,
            g_pan[2] = {0, 0},
            g_center[3] = {0, 0, 0};

    int g_prev_x = 0,
            g_prev_y = 0,
            g_mbutton[3] = {0, 0, 0},
            g_brush_size = 32;

    std::set<int> prev_ids_under_mouse;
    std::vector<std::array<unsigned char, 4>> pick_brush_data;
    std::set<int> ids_this_brush_stroke;

    std::vector<std::string> object_filenames;
    std::vector<std::string> items;
    std::string current_filename;

    sb7::Object object;
};

DECLARE_MAIN(subdivision_app)
