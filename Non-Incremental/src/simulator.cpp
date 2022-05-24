#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <filesystem>
#include <stack>
#include <queue>
#include <chrono>
#include <thread>
#include <future>
#include <mutex>
#include <unordered_map>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/ImGuiIntegration/Context.hpp>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/StringView.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Magnum/DebugTools/FrameProfiler.h>
#include <Magnum/DebugTools/Screenshot.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Math/Color.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Shaders/PhongGL.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>
#include <cmath>
#include <ctime>
#include "ArcBall.h"
#include "include/VoxelGrid.h"
#include "include/Voxel.h"
#include "include/Simdata.h"
#include "Corrade/PluginManager/Manager.h"
#include "Magnum/GL/TextureFormat.h"
#include "include/astar.h"
#include "include/phistar.h"
#include "include/lpastar.h"
#include "include/dlite.h"



using namespace Magnum;
using namespace Magnum::Examples;
using namespace Math::Literals;

int runSimulator(VoxelGrid &voxels);


class Simulator: public Platform::Application {
public:
    explicit Simulator(const Arguments& arguments, VoxelGrid &voxels, SparseVoxelOctree &octree);
    std::vector<InstanceData> _instanceData;

private:
    /*******************************************************************************************************************
    * All the override functions below are overload SDL2 application function that handle input for the application.
    ******************************************************************************************************************/
    void drawEvent() override;
    void viewportEvent(ViewportEvent& event) override;
    void keyPressEvent(KeyEvent& event) override;
    void mousePressEvent(MouseEvent& event) override;
    void mouseReleaseEvent(MouseEvent& event) override;
    void mouseMoveEvent(MouseMoveEvent& event) override;
    void mouseScrollEvent(MouseScrollEvent& event) override;

    /*******************************************************************************************************************
    * Functions below are class functions that need to be accessible from all the threads interating with the OpenGL/
     * Render loop (drawing code).
    ******************************************************************************************************************/
    void fireAlgoGrid(VoxelGrid &voxels, Voxel start, int fTime, int limit, std::mutex &mutex,
                      std::atomic_bool &isRunning);
    void fireAlgoSVO(SparseVoxelOctree &octree, Voxel start, int fTime, int limit, std::mutex &mutex,
                     std::atomic_bool &isRunning);
    void cleanVoxels();
    void cleanGrid(VoxelGrid &voxels);
    void showWalkableGrid(VoxelGrid &voxels);
    void showWalkableOctree(SparseVoxelOctree &octree);
    void drawCircle(Voxel start, int radius, int id);

    void createInstance(Voxel voxel, Color3 color, int id);
    void createInstanceOctree(OctreeNode *voxel, Color3 color, int id);

    DynamicGrid generateDGrid(VoxelGrid &voxels, DynamicGrid &dynamicGrid);
    /**
     * Spatial data structure
     */
    VoxelGrid voxelGrid;
    SparseVoxelOctree voxelOctree;

    /** OpenGL/Magnum variables **/
    GL::Mesh _mesh{NoCreate};
    GL::Buffer _instanceBuffer{NoCreate};
    Containers::Array<Containers::Optional<GL::Mesh>> _meshes;
    Color3 _colorClear = Color3::fromSrgb({0.9f, 0.9f,0.9f});
    Color3 _colorModel = Color3::fromSrgb({0.9f, 0.9f,0.9f});
    Shaders::PhongGL _shader;
    Shaders::PhongGL _voxelShader;
    GL::Mesh _voxelMesh{NoCreate};
    Containers::Optional<ArcBall> _arcballCamera;
    Matrix4 _projectionMatrix;
    PluginManager::Manager <Trade::AbstractImporter> manager;
    ImGuiIntegration::Context _imgui{NoCreate};
    std::string file;

    /** Thread management, perhaps sometime a threadpool needs to be implemented **/
    mutable std::mutex _mtx;
    mutable std::mutex _svo_mtx;
    std::atomic_bool _running = false;
    std::vector<Voxel> _changed;

    /** Grid data structure for D*Lite **/
    DynamicGrid dynamicGrid = DynamicGrid(voxelGrid.max_x, voxelGrid.max_y, voxelGrid.max_z);

    /** UI variables **/
    bool _incremental = false;
    const float _voxelsize = voxelGrid.mVoxelsize;
    const Vec3f _bbox = {6.392, 3.691, 10.04};
    int nThreads = 1;
    int nFires = 1;
    float speed = 1.42f;//1.42 m/s is the standard walking speed
    int time = 500;//milliseconds
    int firesize = 20;
    /** Profiling setup **/
    DebugTools::FrameProfilerGL _profiler{
            DebugTools::FrameProfilerGL::Value::GpuDuration|
            DebugTools::FrameProfilerGL::Value::CpuDuration, 180};

};

Simulator::Simulator(const Arguments& arguments, VoxelGrid &voxels, SparseVoxelOctree &octree):
        Platform::Application{
                arguments,
                Configuration{}.setTitle("Simulator").setWindowFlags(Configuration::WindowFlag::Resizable).setSize({2880,1800}),
                GLConfiguration{}.addFlags(GLConfiguration::Flag::QuietLog)
        },
        voxelGrid{voxels},
        voxelOctree{octree}

{
    _imgui = ImGuiIntegration::Context(Vector2{windowSize()}/dpiScaling(),
                                       windowSize(), framebufferSize());

    /** OpenGL settings **/
    GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha,
                                   GL::Renderer::BlendFunction::OneMinusSourceAlpha);

    /** Import the mesh model of the dataset and generate normals for the mesh **/
    Containers::Pointer<Trade::AbstractImporter> importer =
            manager.loadAndInstantiate("ObjImporter");
    if(!importer) Fatal{} << "Cannot load the AssimpImporter plugin";


    if(voxelGrid.tilted){
        importer->openFile("/Users/michieldejong/Documents/Graduation/Simulator_lpa/dataset_tilted/dataset_1_tilted_228x85x256_fixed.obj");
    }else{
        importer->openFile("/Users/michieldejong/Documents/Graduation/Simulator/dataset/dataset_163_101_256_fixed_2.obj");
    }

    Containers::Optional<Trade::MeshData> _meshData;
    _meshData = importer->mesh(0);
    MeshTools::CompileFlags flags;
    flags |= MeshTools::CompileFlag::GenerateFlatNormals;
    _mesh = MeshTools::compile(*_meshData, flags);
    _mesh.setCount(Containers::arraySize(_meshData->vertexData()));


    /** Instantiate the Uniform Voxel Grid
     * The most basic voxel grid is instantiated by manually setting the rows with the dimensian sizes from the filename.
     * Then a simple .txt file containing indices of all filled voxels is read.
     * These indices are stored as a Voxel.h struct, enabling some simple arithmetic, and potentially even
     * more complicated linear algebra, but this is not used.
     * All points are then filled accordingly on the voxelgrid, with the following values:
     * 0 - free/empty
     * 1 - filled/wall/floor
     * 2 - walkable
     * 3 - blocked path
     * 4 - blocked air
     * 5- goals
     * **/


    /* Setup the instancing */

    _voxelMesh = MeshTools::compile(Primitives::cubeSolid());
    _voxelShader = Shaders::PhongGL{Shaders::PhongGL::Flag::VertexColor|Shaders::PhongGL::Flag::InstancedTransformation};
    _instanceBuffer = GL::Buffer{};

    _voxelMesh.addVertexBufferInstanced(_instanceBuffer, 1, 0, Shaders::PhongGL::TransformationMatrix{}, Shaders::PhongGL::NormalMatrix{}, Shaders::PhongGL::Color3{}, Shaders::PhongGL::ObjectId{});
    _voxelMesh.setInstanceCount(_instanceData.size());
    _voxelMesh.setPrimitive(GL::MeshPrimitive::Triangles);


    setSwapInterval(1);
    setMinimalLoopPeriod(16);
    GL::Renderer::setClearColor(_colorClear);
    dynamicGrid =generateDGrid(voxelGrid, dynamicGrid);



    Debug{} << "This application is running on"<< GL::Context::current().version() << "using" << GL::Context::current().rendererString();
    {
        /* Setup the arcball after the camera objects */
        const Vector3 eye = Vector3(-5.5f, 7.5f,-5.5f);
        const Vector3 viewCenter;
        const Vector3 up = Vector3::yAxis();
        const Deg fov = 100.0_degf;
        _arcballCamera.emplace(eye, viewCenter, up, fov, windowSize());
        _arcballCamera->setLagging(0.9f);
        _projectionMatrix = Matrix4::perspectiveProjection(fov,
                                                           Vector2{framebufferSize()}.aspectRatio(), 0.01f, 100.0f);
    }

}

void Simulator::drawEvent() {
    GL::defaultFramebuffer.clear(
            GL::FramebufferClear::Color|GL::FramebufferClear::Depth);
    _profiler.beginFrame();
    static float alpha = 0.05f;
    _imgui.newFrame();
    /* Enable text input, if needed */
    if(ImGui::GetIO().WantTextInput && !isTextInputActive())
        startTextInput();
    else if(!ImGui::GetIO().WantTextInput && isTextInputActive())
        stopTextInput();

    const bool moving = _arcballCamera->updateTransformation();

    /* ImGui */
    {
        ImGui::Begin("Settings & controls");
        ImGui::Text("Rotate view with mouse drag");
        ImGui::Text("Move with shift+mouse drag");

        if (ImGui::ColorEdit3("Model Color", _colorModel.data())) {
            _shader.setDiffuseColor(Color4 {_colorModel, alpha});
        }
        ImGui::InputInt("No. Evacuees", &nThreads);
        ImGui::InputInt("No. Fires", &nFires);
        ImGui::InputFloat("Walking Speed", &speed);
        ImGui::InputInt("Fire (ms)", &time,100);
        ImGui::InputInt("Fire size)", &firesize);
        const char* DS_items[] = {"Voxel Grid", "Sparse Voxel Octree"};
        const char* PF_items[] = {"A*", "Phi*", "LPA*"};
        static int DS_item_current_idx = 0;
        static int PF_item_current_idx = 0;
        const char * DS_combo_preview_value = DS_items[DS_item_current_idx];
        const char * PF_combo_preview_value = PF_items[PF_item_current_idx];

        if(ImGui::BeginCombo("Data Structure", DS_combo_preview_value, 0)){
            for (int n =0; n < IM_ARRAYSIZE(DS_items); n++){
                const bool is_selected = (DS_item_current_idx == n);
                if(ImGui::Selectable(DS_items[n], is_selected))
                    DS_item_current_idx = n;
                if( is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        if(ImGui::BeginCombo("Algorithm", PF_combo_preview_value, 0)){
            for (int n =0; n < IM_ARRAYSIZE(PF_items); n++){
                const bool is_selected = (PF_item_current_idx == n);
                if(ImGui::Selectable(PF_items[n], is_selected))
                    PF_item_current_idx = n;
                if( is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        if (ImGui::Button("Start")) {
            cleanVoxels();
            cleanGrid(voxelGrid);
            _running = true;
            if(nThreads > simPoints().rooms.size()){
                nThreads = simPoints().rooms.size();
            }
            for (int j = 0; j != nFires; j++) {
                if(DS_item_current_idx == 0) {
                    std::thread fth = std::thread(
                            [this, j] {
                                Simulator::fireAlgoGrid(voxelGrid, simPoints().fires[j], time, firesize, _mtx,
                                                        _running);
                            });
                    fth.detach();
                }
                if(DS_item_current_idx == 1) {
                    std::thread fth = std::thread(
                            [this, j] {
                                Simulator::fireAlgoSVO(voxelOctree, simPoints().fires[j], time, firesize,
                                                       _svo_mtx, _running);
                            });
                    fth.detach();
                }
                if(DS_item_current_idx == 2) {
                    //TODO
                }

            }
            for (int i = 0; i < nThreads; i++) {
                if (DS_item_current_idx == 0) {
                    if (PF_item_current_idx == 0) {
                        std::thread th = std::thread{[this, i] {
                            a_star_search_grid(voxelGrid, _mtx, _instanceData, _voxelsize,
                                               i, speed, _running, _incremental, nThreads);
                        }};
                        th.detach();

                    }

                    if (PF_item_current_idx == 1) {
                        std::thread th = std::thread{[this, i] {
                            phi_star_search_grid(voxelGrid, _mtx, _instanceData, _voxelsize, i, speed, _running,
                                                 _incremental, nThreads);
                        }};
                        th.detach();
                    }
                    if (PF_item_current_idx == 2) {
                        std::thread th = std::thread{[this, i] {
                            lpa_star_search_svo(voxelOctree, _svo_mtx, _instanceData, _voxelsize,
                                                i, speed, _running, _incremental, nThreads);

                        }};
                        th.detach();
                    }
                }
                if (DS_item_current_idx == 1) {
                    if (PF_item_current_idx == 0) {
                        std::thread th = std::thread{[this, i] {
                            a_star_search_svo(voxelOctree, _svo_mtx, _instanceData, _voxelsize,
                                              i, speed, _running, _incremental, nThreads);
                        }};
                        th.detach();
                    }
                    if (PF_item_current_idx == 1) {
                        std::thread th = std::thread{[this, i] {
                            phi_star_search_svo(voxelOctree, _mtx, _instanceData, _voxelsize, i, speed, _running,
                                                _incremental, nThreads);
                        }};
                        th.detach();
                    }
                    if (PF_item_current_idx == 2) {
                        std::thread th = std::thread{[this, i] {
                            lpa_star_search_svo(voxelOctree, _svo_mtx, _instanceData, _voxelsize,
                                                i, speed, _running, _incremental, nThreads);

                        }};
                        th.detach();
                    }
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Show Walkable")) {
            if(DS_item_current_idx == 0){
                showWalkableGrid(voxelGrid);
            }
            else if(DS_item_current_idx == 1){
                showWalkableOctree(voxelOctree);
            }
            else if(DS_item_current_idx == 2){
                //TODO
            }
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("Incremental", &_incremental)){
        }




        ImGui::Text("Application average %.3f ms/frame (%.1f FPS) ",
                    1000.0 / Double(ImGui::GetIO().Framerate), Double(ImGui::GetIO().Framerate));
        ImGui::Text("Application using %d threads",
                    Int(std::thread::hardware_concurrency()));
        if (ImGui::Button("Reset camera")) {
            _arcballCamera->reset();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset simluator")) {
            cleanVoxels();
            cleanGrid(voxelGrid);
        }
        ImGui::End();

    }
    /*
    int voxelID = 10;
    _instanceData.erase(
            remove_if(_instanceData.begin(), _instanceData.end(),
                      [&voxelID](InstanceData instanceData) { return instanceData.id > voxelID; }
            ), _instanceData.end());
    */

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::disable(GL::Renderer::Feature::Blending);
    _voxelMesh.setInstanceCount(_instanceData.size());
    _instanceBuffer.setData(_instanceData,GL::BufferUsage::DynamicDraw);
    _voxelShader.setProjectionMatrix(_projectionMatrix)
            .setTransformationMatrix(_arcballCamera->viewMatrix())
            .setLightPositions({{10.0f, 10.0f, 10.0f,0.0f}})
            .setNormalMatrix(_arcballCamera->viewMatrix().normalMatrix())
            .draw(_voxelMesh);

    GL::Renderer::disable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::Blending);


    GL::Renderer::setFaceCullingMode(GL::Renderer::PolygonFacing::Front);
    _shader.setLightPositions({{10.0f, 10.0f, 10.0f,0.0f}})
            .setDiffuseColor(Color4{_colorModel,alpha})
            .setTransformationMatrix(_arcballCamera->viewMatrix())
            .setShininess(200.0f)
            .setNormalMatrix(_arcballCamera->viewMatrix().normalMatrix())
            .setProjectionMatrix(_projectionMatrix)
            .draw(_mesh);
    GL::Renderer::setFaceCullingMode(GL::Renderer::PolygonFacing::Back);
    _shader.setLightPositions({{10.0f, 10.0f, 10.0f,0.0f}})
            .setDiffuseColor(Color4{_colorModel,alpha})
            .setTransformationMatrix(_arcballCamera->viewMatrix())
            .setShininess(200.0f)
            .setNormalMatrix(_arcballCamera->viewMatrix().normalMatrix())
            .setProjectionMatrix(_projectionMatrix)
            .draw(_mesh);
    GL::Renderer::disable(GL::Renderer::Feature::Blending);
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);




    /* Set appropriate states. If you only draw ImGui, it is sufficient to
   just enable blending and scissor test in the constructor. */
    GL::Renderer::enable(GL::Renderer::Feature::Blending);
    GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);
    GL::Renderer::disable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::disable(GL::Renderer::Feature::DepthTest);

    _imgui.drawFrame();

    /* Reset state. Only needed if you want to draw something else with
       different state after. */
    GL::Renderer::disable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);
    GL::Renderer::enable(GL::Renderer::Feature::Blending);
    _profiler.endFrame();
    _profiler.printStatistics(2000);
    swapBuffers();
    redraw();
}
void Simulator::viewportEvent(ViewportEvent& event) {
    GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});

    _imgui.relayout(Vector2{event.windowSize()}/event.dpiScaling(),
                    event.windowSize(), event.framebufferSize());

    _arcballCamera->reshape(event.windowSize());
    _projectionMatrix = Matrix4::perspectiveProjection(_arcballCamera->fov(),
                                                       Vector2{event.framebufferSize()}.aspectRatio(), 0.01f, 500.0f);
}


void Simulator::keyPressEvent(KeyEvent& event) {
    switch(event.key()) {
        case KeyEvent::Key::L:
            if(_arcballCamera->lagging() > 0.0f) {
                Debug{} << "Lagging disabled";
                _arcballCamera->setLagging(0.0f);
            } else {
                Debug{} << "Lagging enabled";
                _arcballCamera->setLagging(0.85f);
            }
            break;
        case KeyEvent::Key::R:
            _arcballCamera->reset();
            break;
        case KeyEvent::Key::Esc:
            _mtx.try_lock();
            _svo_mtx.try_lock();
            exit();
            std::cout<<"Exiting application...\n";
            std::cout<<"Goodbye!\n";
            _mtx.unlock();
            _svo_mtx.unlock();
        default: return;
    }

    event.setAccepted();
    redraw(); /* camera has changed, redraw! */
}

void Simulator::mousePressEvent(MouseEvent& event) {
    if(_imgui.handleMousePressEvent(event)) return;

    _arcballCamera->initTransformation(event.position());

    event.setAccepted();
    redraw();
}

void Simulator::mouseReleaseEvent(MouseEvent& event) {
    if(_imgui.handleMouseReleaseEvent(event)) return;
}

void Simulator::mouseMoveEvent(MouseMoveEvent& event) {
    if(_imgui.handleMouseMoveEvent(event)) return;

    if(!event.buttons()) return;

    if(event.modifiers() & MouseMoveEvent::Modifier::Shift)
        _arcballCamera->translate(event.position());
    else _arcballCamera->rotate(event.position());
    //std::cout<<event.position()*_arcballCamera->viewMatrix()<<std::endl;
    event.setAccepted();
    redraw();
}

void Simulator::mouseScrollEvent(MouseScrollEvent& event) {
    if(_imgui.handleMouseScrollEvent(event)) {
        /* Prevent scrolling the page */
        event.setAccepted();
        return;
    }

    const Float delta = event.offset().y();
    if(Math::abs(delta) < 1.0e-2f) return;

    _arcballCamera->zoom(delta);

    event.setAccepted();
    redraw();
}
/**
 * Function to simulate a "fire", i.e. a cylinder with voxels that are inacessible. (Code 3)
 * This cylinder grows horizontally first, then vertically.
 *
 * @param voxels VoxelGrid with semantics.
 * @param start Voxel from which the fire is to be started.
 * @param fTime Time (in ms) between generations of the fire.
 * @param limit Maximum size (in voxels) the fire can grow.
 */
void Simulator::fireAlgoGrid(VoxelGrid &voxels, Voxel start, int fTime, int limit, std::mutex &mutex,
                             std::atomic_bool &isRunning) {
    int vGen = 0;
    int vLim = limit*2;
    if(vLim > voxels.max_y){
        vLim = voxels.max_y -2;
    }
    int hLim = limit;
    int generation = 1;
    while(isRunning) {
        while (vGen <= vLim) {
            while (hLim != 0) {
                mutex.lock();
                if (voxels(start.x, start.y + vGen, start.z) == 2) {
                    drawCircle({start.x, start.y + vGen, start.z}, generation, 3);
                } else if (voxels(start.x, start.y + vGen, start.z) == 0) {
                    drawCircle({start.x, start.y + vGen, start.z}, generation, 4);
                } else if (voxels(start.x, start.y + vGen, start.z) == 1) {
                    drawCircle({start.x, start.y + vGen, start.z}, generation, 5);
                } else if (voxels(start.x, start.y + vGen, start.z) == 4) {
                    drawCircle({start.x, start.y + vGen, start.z}, generation, 4);
                } else if (voxels(start.x, start.y + vGen, start.z) == 5) {
                    drawCircle({start.x, start.y + vGen, start.z}, generation, 5);
                }
                if (voxels(start.x, start.y + 1 + vGen, start.z) == 2) {
                    drawCircle({start.x, start.y + 1 + vGen, start.z}, generation, 3);
                } else if (voxels(start.x, start.y + 1 + vGen, start.z) == 0) {
                    drawCircle({start.x, start.y + 1 + vGen, start.z}, generation, 4);
                } else if (voxels(start.x, start.y + 1 + vGen, start.z) == 1) {
                    drawCircle({start.x, start.y + 1 + vGen, start.z}, generation, 5);
                } else if (voxels(start.x, start.y + 1 + vGen, start.z) == 4) {
                    drawCircle({start.x, start.y + 1 + vGen, start.z}, generation, 4);
                } else if (voxels(start.x, start.y + 1 + vGen, start.z) == 5) {
                    drawCircle({start.x, start.y + 1 + vGen, start.z}, generation, 5);
                }
                if (voxels(start.x, start.y + 2 + vGen, start.z) == 2) {
                    drawCircle({start.x, start.y + 2 + vGen, start.z}, generation, 3);
                } else if (voxels(start.x, start.y + 2 + vGen, start.z) == 0) {
                    drawCircle({start.x, start.y + 2 + vGen, start.z}, generation, 4);
                } else if (voxels(start.x, start.y + 2 + vGen, start.z) == 1) {
                    drawCircle({start.x, start.y + 2 + vGen, start.z}, generation, 5);
                } else if (voxels(start.x, start.y + 2 + vGen, start.z) == 4) {
                    drawCircle({start.x, start.y + 2 + vGen, start.z}, generation, 4);
                } else if (voxels(start.x, start.y + 2 + vGen, start.z) == 5) {
                    drawCircle({start.x, start.y + 2 + vGen, start.z}, generation, 5);
                }
                _voxelMesh.setInstanceCount(_instanceData.size());
                //std::cout<< "Instance array has size of "<<_instanceData.size()<<"\n";
                mutex.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(fTime));
                generation++;
                hLim--;
            }
            fTime = fTime / 1.3;
            std::this_thread::sleep_for(std::chrono::milliseconds(fTime));
            generation = 0;
            hLim = limit;
            vGen += 3;
        }
        isRunning = false;
    }
}
/**
 * Function to simulate a "fire" in the octree. It does this by removing voxels from the SVO.
 * This fire grows horizontally first, then vertically.
 * @param octree Sparse Voxel Octree (SparseVoxelOctree()).
 * @param start Voxel from which the fire is to be started.
 * @param fTime Time (in ms) between generations of the fire.
 * @param limit Maximum size (in voxels) the fire can grow.
 */
void Simulator::fireAlgoSVO(SparseVoxelOctree &octree, Voxel start, int fTime, int limit, std::mutex &mutex,
                            std::atomic_bool &isRunning) {
    int vLim;
    uint_fast32_t x, y, z;
    x = start.x;
    y = start.y;
    z = start.z;
    std::queue<OctreeNode*> stack;
    auto index = libmorton::morton3D_64_encode(x, y, z);
    auto seed = octree.mQTree[0][index];
    OctreeNode* current;
    stack.push(seed);
    int generation = 0;
    while(y < _bbox.y/_voxelsize) {
        while (generation < (limit*150)) {
            current = stack.front();
            if(current == nullptr) break;
            stack.pop();
            for (auto &all: getNeighbours18_morton(current->x, current->y, current->z, current->index)) {
                if (octree.mQTree[0].contains(all)) {
                    auto child = octree.mQTree[0][all];
                    createInstanceOctree(child, {0.0f, 0.0f, 0.0f}, 3);
                    stack.push(child);
                    mutex.lock();
                    octree.mQTree[0].erase(all);
                    child->isChanged = true;
                    child->attribute = 3;
                    mutex.unlock();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(fTime/100));
            generation++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(fTime/10));
        y+=3;
        index = libmorton::morton3D_64_encode(x, y, z);
        if(octree.mQTree[0].contains(index)){
            auto tmp = octree.mQTree[0][index];
            stack = {};
            generation = 0;
            stack.push(tmp);
        } else {
            continue;
        }

    }
    isRunning = false;

}

/**
 * A function to draw a circle on the XZ plane in voxelspace.
 *
 * @param start Center of the circle.
 * @param radius Radius of the circle.
 * @param id integer value to set the affected voxels to.
 */
void Simulator::drawCircle(Voxel start, int radius, int id){
    int f = 1 - radius;
    int ddF_x = 0;
    int ddF_z = -2 * radius;
    int x = 0;
    int z = radius;
    Voxel v1 = {start.x, start.y, start.z + radius};
    createInstance(v1, {0.0f, 0.0f, 0.0f}, id);
    Voxel v2 = {start.x, start.y, start.z - radius};
    createInstance(v2, {0.0f, 0.0f, 0.0f}, id);
    Voxel v3 = {start.x + radius, start.y, start.z};
    createInstance(v3, {0.0f, 0.0f, 0.0f}, id);
    Voxel v4 = {start.x - radius, start.y, start.z};
    createInstance(v4, {0.0f, 0.0f, 0.0f}, id);
    while (x < z){
        if ( f >=0){
            z--;
            ddF_z +=2;
            f += ddF_z;
        }
        x++;
        ddF_x += 2;
        f += ddF_x +1;
        createInstance({start.x + x, start.y, start.z + z}, {0.0f, 0.0f, 0.0f}, id);
        createInstance({start.x - x, start.y, start.z + z}, {0.0f, 0.0f, 0.0f}, id);
        createInstance({start.x + x, start.y, start.z - z}, {0.0f, 0.0f, 0.0f}, id);
        createInstance({start.x - x, start.y, start.z - z}, {0.0f, 0.0f, 0.0f}, id);
        createInstance({start.x + z, start.y, start.z + x}, {0.0f, 0.0f, 0.0f}, id);
        createInstance({start.x - z, start.y, start.z + x}, {0.0f, 0.0f, 0.0f}, id);
        createInstance({start.x + z, start.y, start.z - x}, {0.0f, 0.0f, 0.0f}, id);
        createInstance({start.x - z, start.y, start.z - x}, {0.0f, 0.0f, 0.0f}, id);
    }

}
/**
 * Resets the vertex buffer and stops all the running threads.
 */
void Simulator::cleanVoxels(){
    _mtx.lock();
    _running = false;
    _instanceData.erase(_instanceData.begin(), _instanceData.end());
    _voxelMesh.setInstanceCount(_instanceData.size());
    _mtx.unlock();
}
/**
 * Cleans the VoxelGrid (resets the fire and cleans the paths).
 *
 * @param voxels VoxelGrid
 */
void Simulator::cleanGrid(VoxelGrid &voxels){
    _mtx.lock();
    _svo_mtx.lock();
    std::cout<<"Rebuilding and cleaning grid...\n";

    for(auto &e: voxels.voxels){
        if(e == 3){
            e = 2;
        } else if (e == 4){
            e = 0;
        } else if (e > 10){
            e = 2;
        } else if (e == 5) {
            e = 1;
        }
    }
    for(const auto all: voxelOctree.mTree[0]){
        all.second->isChanged = false;
        all.second->attribute = 2;
    }
    voxelOctree.mQTree.clear();
    int n = 0;
    std::cout<<"Rebuilding and cleaning octree...\n";
    while(n < voxelOctree.getDepth()) {
        std::unordered_map<uint_fast64_t, OctreeNode*> level;
        for (auto all: voxelOctree.mTree[n]) {
            level.emplace(all);
        }
        voxelOctree.mQTree.push_back(level);
        n++;
    }
    _mtx.unlock();
    _svo_mtx.unlock();
    std::cout<<"Cleaning done, ready to start new run!\n";
}
/**
 * Showing all the voxels in the grid that are walkable (attribute code 2).
 * Prints the runtime of the algorithm to the console.
 *
 * @param voxels VoxelGrid
 */
void Simulator::showWalkableGrid(VoxelGrid &voxels) {
    auto start = std::chrono::high_resolution_clock::now();
    for (int i=0; i< voxels.max_x-1; i++) {
        for (int j = 0; j < voxels.max_y-1; j++) {
            for (int k = 0; k < voxels.max_z-1; k++) {
                //if(attribute == 3){std::cout<<voxelGrid(i, j, k)<<std::endl;}
                if (voxelGrid(i, j, k) == 2) {
                    Voxel p = {i, j, k};
                    InstanceData _instance;
                    Matrix4 _m = Matrix4::translation({static_cast<float>(p.x * _voxelsize+0.5*_voxelsize), static_cast<float>(p.y * _voxelsize+0.5*_voxelsize), static_cast<float>(p.z * _voxelsize+0.5*_voxelsize)}) * Matrix4::scaling({_voxelsize,_voxelsize, _voxelsize});
                    _instance.transformation = _m;
                    _instance.normalMatrix = _m.normalMatrix();
                    _instance.color = Color3(0.5f,0.64f,0.93f);
                    _instance.id = 2;
                    _instanceData.push_back(_instance);
                }
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;
    std::cout<<"Traversing grid took "<<diff.count()<<" seconds\n";
}

/**
 * Send an instance to the instance buffer. This function is currently not in use.
 * @test
 * @param voxel Voxel to be rendered.
 * @param color Color3 the voxel should have.
 * @param id ID (attribute) the voxel has.
 */
void Simulator::createInstanceOctree(OctreeNode *voxel, Color3 color, int id){
    InstanceData instance;
    Matrix4 _m = Matrix4::translation({static_cast<float>(voxel->x * voxel->level * _voxelsize+0.5*_voxelsize), static_cast<float>(voxel->y * voxel->level * _voxelsize+0.5*_voxelsize), static_cast<float>(voxel->z * voxel->level * _voxelsize+0.5*_voxelsize)}) * Matrix4::scaling({_voxelsize,_voxelsize, _voxelsize});
    instance.transformation = _m;
    instance.normalMatrix = _m.normalMatrix();
    instance.color = color;
    instance.id = id;
    _instanceData.push_back(instance);
}

/**
 * Send an instance to the instance buffer.
 *
 * @param voxel Voxel to be rendered.
 * @param color Color3 the voxel should have.
 * @param id ID (attribute) the voxel has.
 */
void Simulator::createInstance(Voxel voxel, Color3 color, int id){
    voxelGrid(voxel.x, voxel.y, voxel.z) = id;
    InstanceData instance;
    Matrix4 _m = Matrix4::translation({static_cast<float>(voxel.x * _voxelsize+0.5*_voxelsize), static_cast<float>(voxel.y * _voxelsize+0.5*_voxelsize), static_cast<float>(voxel.z * _voxelsize+0.5*_voxelsize)}) * Matrix4::scaling({_voxelsize,_voxelsize, _voxelsize});
    instance.transformation = _m;
    instance.normalMatrix = _m.normalMatrix();
    instance.color = color;
    instance.id = id;
    _instanceData.push_back(instance);
    _changed.push_back(voxel);
}

DynamicGrid Simulator::generateDGrid(VoxelGrid &voxels, DynamicGrid &dynamicGrid){
    for ( int x = 0; x < voxels.max_x-1; ++x) {
        for ( int y = 0; y < voxels.max_y-1; ++y) {
            for (int z = 0; z < voxels.max_z - 1; ++z) {
                    dynamicGrid(x, y, z)->x = x;
                    dynamicGrid(x, y, z)->y = y;
                    dynamicGrid(x, y, z)->z = z;
                    if(voxelGrid(x,y,z) == 2) dynamicGrid(x, y, z)->id = 2;
                    if(voxelGrid(x,y,z) == 1) dynamicGrid(x, y, z)->id = 1;
                }
            }
        }

    return dynamicGrid;
}

/**
 * Sending full octree to the instance buffer.
 * Nodes that are full are rendered at their appropriate level of detail.
 * Prints the runtime of the algorithm to the console.
 *
 * @param octree Sparse Voxel Octree
 */
void Simulator::showWalkableOctree(SparseVoxelOctree &octree) {
    int maxDepth;
    auto start = std::chrono::high_resolution_clock::now();
    int level = 0;
    while(level < 3) {
        float r = randomFloat(1.0f, 0.0f);
        float g = randomFloat(1.0f, 0.0f);
        float b = randomFloat(1.0f, 0.0f);
        float dFactor = pow(2, (level));
        float dFactor_1 = pow(2, (level - 1));
        float vSize;
        float vSize_1 = _voxelsize * dFactor_1;
        float sSize = _voxelsize / 2;
        float sSize_1;
        for (auto all: octree.mTree[level]) {
            if ((all.second->isFull && !all.second->Parent->isFull) ||
                (all.second->isLeaf && !all.second->Parent->isFull)) {
                InstanceData mInstance;
                Matrix4 _m = Matrix4::translation(
                        {static_cast<float>(all.second->x * all.second->level * _voxelsize + sSize),
                         static_cast<float>(all.second->y * all.second->level * _voxelsize + sSize),
                         static_cast<float>(all.second->z * all.second->level * _voxelsize + sSize)}) *
                             Matrix4::scaling({_voxelsize*all.second->level, _voxelsize*all.second->level, _voxelsize*all.second->level});
                mInstance.transformation = _m;
                mInstance.normalMatrix = _m.normalMatrix();
                mInstance.color = Color3(r, g, b);
                mInstance.id = 2;
                _instanceData.push_back(mInstance);
            }
        }
        level++;

    }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end - start;
        std::cout << "Traversing octree took " << diff.count() << " seconds\n";
}
/**
 * Run Simulator Class from main.cpp.
 *
 * @param voxels VoxelGrid read from point cloud/voxel list
 * @param octree SparseVoxelOctree created from VoxelGrid
 * @param argc
 * @param argv
 * @return Simulator application
 */
int runSimulator(VoxelGrid &voxels, SparseVoxelOctree &octree, int argc, char** argv){
    Simulator app({argc, argv}, voxels, octree);
    return app.exec();
}






//MAGNUM_APPLICATION_MAIN(Simulator)
