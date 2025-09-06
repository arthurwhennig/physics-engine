cmake_minimum_required(VERSION 3.16)
project(PhysicsEngine VERSION 1.0.0 LANGUAGES CXX)

# set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# compiler-specific options
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# include directories
include_directories(include)

# find OpenGL
find_package(OpenGL REQUIRED)

# try to find SFML (for graphics and windowing)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SFML sfml-all)

if(NOT SFML_FOUND)
    # If SFML is not found via pkg-config, try CMake find_package
    find_package(sfml-graphics)
    find_package(sfml-window)
    find_package(sfml-system)

    if(sfml-graphics_FOUND AND sfml-window_FOUND AND sfml-system_FOUND)
        set(SFML_LIBRARIES sfml-graphics sfml-window sfml-system)
    else()
        message(WARNING "SFML not found. Graphics demo will not be available.")
        set(SFML_LIBRARIES "")
    endif()
endif()

# physics engine library source files
set(PHYSICS_ENGINE_SOURCES
        src/Vector2D.cpp
        src/RigidBody.cpp
        src/PhysicsWorld.cpp
        src/CollisionDetector.cpp
        src/ForceGenerator.cpp
        src/Matrix2D.cpp
        src/Utility.cpp
        src/Polygon.cpp
        src/Quadtree.cpp
)

# physics engine library headers
set(PHYSICS_ENGINE_HEADERS
        include/Vector2D.h
        include/RigidBody.h
        include/PhysicsWorld.h
        include/CollisionDetector.h
        include/ForceGenerator.h
        include/Matrix2D.h
        include/Utility.h
        include/Polygon.h
        include/Quadtree.h
        include/AABB.h
        examples/collision_demo.cpp
)

# create physics engine static library
add_library(physics_engine STATIC ${PHYSICS_ENGINE_SOURCES} ${PHYSICS_ENGINE_HEADERS})
target_include_directories(physics_engine PUBLIC include)

# console demo (no graphics dependencies)
add_executable(console_demo examples/console_demo.cpp)
target_link_libraries(console_demo physics_engine)

# collision demo (no graphics dependencies)
add_executable(collision_demo examples/collision_demo.cpp)
target_link_libraries(collision_demo physics_engine)

# graphics demo (requires dependency SFML)
if(SFML_FOUND OR (sfml-graphics_FOUND AND sfml-window_FOUND AND sfml-system_FOUND))
    add_executable(graphics_demo examples/graphics_demo.cpp)
    target_link_libraries(graphics_demo physics_engine)

    if(SFML_FOUND)
        # sse pkg-config for SFML
        target_compile_options(graphics_demo PRIVATE ${SFML_CFLAGS})
        # Fix the library directory path
        target_link_directories(graphics_demo PRIVATE "/opt/homebrew/Cellar/sfml/3.0.1/lib")
        target_link_libraries(graphics_demo ${SFML_LIBRARIES})
    else()
        # use CMake targets for SFML
        target_link_libraries(graphics_demo sfml-graphics sfml-window sfml-system)
    endif()

    # also link OpenGL if found
    if(OPENGL_FOUND)
        target_link_libraries(graphics_demo ${OPENGL_LIBRARIES})
    endif()

    # copy any required SFML DLLs on Windows
    if(WIN32 AND SFML_FOUND)
        # this would copy SFML DLLs if needed
    endif()
endif()

# installation
install(TARGETS physics_engine
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(FILES ${PHYSICS_ENGINE_HEADERS} DESTINATION ../include/physics_engine)

# print configuration summary
message(STATUS "")
message(STATUS "Configuration Summary:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  SFML found: ${SFML_FOUND}")
if(SFML_LIBRARIES)
    message(STATUS "  SFML libraries: ${SFML_LIBRARIES}")
endif()
message(STATUS "")
