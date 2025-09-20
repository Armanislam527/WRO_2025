# CMakeLists.txt
# Build configuration for WRO 2025 Pi-side application

cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(WRO2025_Vehicle_Controller VERSION 1.0)

# Specify the C++ standard (C++14 or C++17 is common for Pi projects)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- Find Required Libraries ---
# Find OpenCV package (adjust COMPONENTS as needed, e.g., imgproc, videoio)
# This assumes OpenCV is installed on the Pi (e.g., via apt install libopencv-dev)
find_package(OpenCV REQUIRED)

# --- Include Directories ---
# Add the 'src' directory and 'include' directory (if you use it) to the include path
# This allows #include "module/header.h" to work
target_include_directories(${PROJECT_NAME} PRIVATE src)

# --- Source Files ---
# Recursively find all .cpp files in the src directory and its subdirectories
# This is convenient but can sometimes be less explicit than listing files.
# For better control, you can list source files explicitly.
file(GLOB_RECURSE SOURCES "src/*.cpp")

# --- Executable ---
# Create the executable named after the project
add_executable(${PROJECT_NAME} ${SOURCES})

# --- Link Libraries ---
# Link the required libraries to the executable
# Link OpenCV libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# --- Compiler Options (Optional but useful) ---
# Add warnings, optimizations, etc.
# -Wall: Enable most common warnings
# -Wextra: Enable extra warnings
# -O2: Optimization level 2 (balance between speed and compilation time)
# -g: Include debugging information
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -O2 -g)

# --- Installation (Optional) ---
# Define where the executable should be installed (e.g., 'sudo make install')
# install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# --- Custom Targets (Optional) ---
# You could add a custom target to run your vehicle script, if you create one
# add_custom_target(run
#     COMMAND ${PROJECT_NAME}
#     DEPENDS ${PROJECT_NAME}
#     WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
# )