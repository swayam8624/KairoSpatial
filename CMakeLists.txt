cmake_minimum_required(VERSION 3.28)
project(KairoSpatial LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(KAIRO_SPATIAL_BUILD_TESTS "Build KairoSpatial tests" ${PROJECT_IS_TOP_LEVEL})
option(KAIRO_SPATIAL_BUILD_VISUALIZER "Build the standalone KairoSpatial visualizer" ${PROJECT_IS_TOP_LEVEL})

# Direct consumers fetch the revision verified by the umbrella rather than the
# moving default branch. Workspace and explicit source paths remain preferred.
set(KAIRO_SPATIAL_GEOMETRY_REVISION 87c32babcf71f5bbc375398e4a15af48610569ab)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
elseif(MSVC)
    add_compile_options(/W4)
endif()

if(NOT TARGET KairoGeometry)
    set(KAIRO_GEOMETRY_SOURCE_DIR "" CACHE PATH "Path to a local KairoGeometry checkout")

    if(KAIRO_GEOMETRY_SOURCE_DIR AND EXISTS "${KAIRO_GEOMETRY_SOURCE_DIR}/CMakeLists.txt")
        add_subdirectory(
            "${KAIRO_GEOMETRY_SOURCE_DIR}"
            "${CMAKE_CURRENT_BINARY_DIR}/_deps/KairoGeometry"
        )
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../KairoGeometry/CMakeLists.txt")
        add_subdirectory(
            "${CMAKE_CURRENT_SOURCE_DIR}/../KairoGeometry"
            "${CMAKE_CURRENT_BINARY_DIR}/_deps/KairoGeometry"
        )
    else()
        include(FetchContent)
        FetchContent_Declare(
            KairoGeometry
            GIT_REPOSITORY https://github.com/swayam8624/KairoGeometry.git
            GIT_TAG ${KAIRO_SPATIAL_GEOMETRY_REVISION}
            GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(KairoGeometry)
    endif()
endif()

add_library(KairoSpatial)

target_sources(KairoSpatial
    PUBLIC
        FILE_SET CXX_MODULES
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
            SpatialTypes.cppm
            BVH.cppm
            BVHBuilder.cppm
            BVHTraversal.cppm
            SpatialQuery.cppm
            DynamicAABBTree.cppm
            SweepAndPrune.cppm
            SpatialHash.cppm
            Octree.cppm
            Quadtree.cppm
            KDTree.cppm
            NavigationGraph.cppm
            AStar.cppm
            SpatialDebug.cppm
            KairoSpatial.cppm
)

target_link_libraries(KairoSpatial PUBLIC KairoGeometry)

if(KAIRO_SPATIAL_BUILD_VISUALIZER)
    add_executable(KairoSpatialVisual visual/SpatialVisualizer.cpp)
    target_link_libraries(KairoSpatialVisual PRIVATE KairoSpatial)
endif()

if(KAIRO_SPATIAL_BUILD_TESTS)
    # ExternalProject uses Git's version to decide whether recursive checkout
    # flags are supported. Declare the tool explicitly so standalone builds do
    # not depend on another dependency having discovered it first.
    find_package(Git REQUIRED)
    include(FetchContent)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.5.4
        # Catch2 has no required submodules. Disabling recursive checkout also
        # keeps FetchContent's isolated populate step independent of CMake's
        # transient Git version variables.
        GIT_SUBMODULES ""
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(Catch2)

    add_executable(SpatialTests tests/SpatialTests.cpp)
    target_link_libraries(SpatialTests PRIVATE KairoSpatial Catch2::Catch2WithMain)

    enable_testing()
    add_test(NAME SpatialTests COMMAND SpatialTests)
endif()
