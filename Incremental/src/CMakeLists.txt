find_package(Corrade REQUIRED Main)
find_package(Magnum REQUIRED GL Sdl2Application SceneGraph Shaders Trade Primitives DebugTools )
set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS OFF)
find_package(Threads REQUIRED)

add_executable(Simulator
        main.cpp
        include/astar.cpp
        include/lpastar.cpp
        include/phistar.cpp
        simulator.cpp
        ArcBall.cpp
)
add_dependencies(Simulator Magnum::AnyImageImporter MagnumPlugins::StbImageImporter Magnum::ObjImporter )

target_link_libraries(Simulator PRIVATE
  Corrade::Main
  Magnum::Application
  Magnum::GL
  Magnum::Shaders
  Magnum::Magnum Magnum::SceneGraph Magnum::Trade Magnum::Primitives Magnum::DebugTools
  MagnumIntegration::ImGui
        Threads::Threads


)

set_target_properties(Simulator PROPERTIES
  CXX_STANDARD 20
)

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Simulator)
