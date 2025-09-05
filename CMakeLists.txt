cmake_minimum_required(VERSION 3.16)
project(PhysicsEngine VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler-specific options
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Include directories
include_directories(include)

# Find packages
find_package(OpenGL REQUIRED)

# Physics engine library source files
set(PHYSICS_ENGINE_SOURCES
        src/Vector2D.cpp
        src/RigidBody.cpp
        src/PhysicsWorld.cpp
        src/CollisionDetector.cpp
        src/ForceGenerator.cpp
)

# Physics engine library headers
set(PHYSICS_ENGINE_HEADERS
        include/Vector2D.hpp
        include/RigidBody.hpp
        include/PhysicsWorld.hpp
        include/CollisionDetector.hpp
        include/ForceGenerator.hpp
)

# Create physics engine static library
add_library(physics_engine STATIC ${PHYSICS_ENGINE_SOURCES} ${PHYSICS_ENGINE_HEADERS})
target_include_directories(physics_engine PUBLIC include)

# Console demo (no graphics dependencies)
add_executable(console_demo examples/console_demo.cpp
)
target_link_libraries(console_demo physics_engine)

# Installation
install(TARGETS physics_engine
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(FILES ${PHYSICS_ENGINE_HEADERS} DESTINATION ../include/physics_engine)

# Print configuration summary
message(STATUS "")
message(STATUS "Configuration Summary:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  SFML found: ${SFML_FOUND}")
if(SFML_LIBRARIES)
    message(STATUS "  SFML libraries: ${SFML_LIBRARIES}")
endif()
message(STATUS "")
