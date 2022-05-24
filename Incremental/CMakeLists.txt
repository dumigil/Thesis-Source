cmake_minimum_required(VERSION 3.17)
project(Simulator)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/modules/ ${CMAKE_MODULE_PATH})


# Add Corrade as a subproject
add_subdirectory(external/corrade EXCLUDE_FROM_ALL)


# Add Magnum as a subproject, enable Sdl2Application
set(WITH_SDL2APPLICATION ON CACHE BOOL "" FORCE)
set(WITH_ANYIMAGEIMPORTER ON CACHE BOOL "" FORCE)
set(WITH_STBIMAGEIMPORTER ON CACHE BOOL "" FORCE)
set(WITH_OBJIMPORTER ON CACHE BOOL "" FORCE)

add_subdirectory(external/magnum EXCLUDE_FROM_ALL)
add_subdirectory(external/magnum-plugins EXCLUDE_FROM_ALL)

# ImGui integration
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
set(WITH_IMGUI ON CACHE BOOL "" FORCE)
add_subdirectory(external/magnum-integration EXCLUDE_FROM_ALL)
find_package(MagnumIntegration REQUIRED ImGui)

# linalg.h, nlohmann-jso
include_directories(${PROJECT_SOURCE_DIR}/external)

message(${CMAKE_CXX_COMPILER_ID})
message(${CMAKE_CXX_COMPILER_VERSION})

add_subdirectory(src)