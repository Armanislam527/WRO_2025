# CMakeLists.txt
# Build configuration for WRO 2025 Pi-side application (C++ version)

cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(WRO2025_Vehicle_Controller VERSION 1.0)

# Specify the C++ standard (C++17 is common for modern Pi projects)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- Find Required Libraries ---
# Find OpenCV package (adjust COMPONENTS as needed, e.g., imgproc, videoio, video)
# This assumes OpenCV is installed on the Pi (e.g., via apt install libopencv-dev)
find_package(OpenCV REQUIRED)

# --- Source Files ---
# Recursively find all .cpp files in the src directory and its subdirectories
# This is convenient but can sometimes be less explicit than listing files.
# For better control, especially in larger projects, you might list source files explicitly.
# Ensure the GLOB_RECURSE pattern matches your actual source structure.
file(GLOB_RECURSE SOURCES "src/*.cpp")

# --- Executable ---
# Create the executable named after the project
# **** THIS MUST COME BEFORE target_* commands ****
add_executable(${PROJECT_NAME} ${SOURCES})

# --- Include Directories ---
# Add the 'src' directory to the include path
# This allows #include "module/submodule/header.h" to work relative to src/
# **** THIS MUST COME AFTER add_executable ****
target_include_directories(${PROJECT_NAME} PRIVATE src)

# --- Link Libraries ---
# Link the required libraries to the executable
# Link OpenCV libraries found by find_package
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# --- Compiler Options (Optional but useful for development) ---
# Add warnings, optimizations, etc.
# -Wall: Enable most common warnings
# -Wextra: Enable extra warnings
# -O2: Optimization level 2 (balance between speed and compilation time)
# -g: Include debugging information (useful for debugging with gdb)
# -DNDEBUG: Define NDEBUG to disable asserts in release builds (optional)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -O2 -g)
# Uncomment the line below for release builds to disable asserts
# target_compile_definitions(${PROJECT_NAME} PRIVATE -DNDEBUG)

# --- Optional: Enable Logging for Debug Builds ---
# To enable the LOG_DEBUG, LOG_INFO, etc. macros from src/utils/logger.h,
# uncomment the line below. Comment it out for final competition builds.
 target_compile_definitions(${PROJECT_NAME} PRIVATE LOG_LEVEL=5)

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