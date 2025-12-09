cmake_minimum_required(VERSION 3.16)
project(flock LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Automatically prefer precompiled binaries found in lib/ folder which is sibling to the main 
# executable.
set(CMAKE_INSTALL_RPATH, "$ORIGIN/../lib")


# Only thing users need to change: Set this to your SFML 3.0 installation path
# Example paths:
# - Windows: C:/SFML-3.0.0
# - Linux: /usr/local
# - macOS: /usr/local or /opt/homebrew (if using Homebrew)
# Users can also set this via command line: -DCMAKE_PREFIX_PATH=/path/to/sfml
# set(CMAKE_PREFIX_PATH "/path/to/sfml")

# Set your required components here...
set(SFML_COMPONENTS Graphics Window System Audio)

# First attempt to find system installation
find_package(SFML 3 COMPONENTS ${SFML_COMPONENTS})

if(SFML_FOUND)
    message(STATUS "Found system installation of SFML in ${SFML_DIR}")
else()
    message(STATUS "System installation of SFML not found, looking for install in extern 
                    by appending ${CMAKE_SOURCE_DIR}/extern/SFML-3.0.0 to CMAKE_PREFIX_PATH...")
    list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/extern/SFML-3.0.0")
    find_package(SFML 3 REQUIRED COMPONENTS ${SFML_COMPONENTS})
endif()

# Create executable
add_executable(flock main.cpp)

# Link SFML libraries
target_link_libraries(flock PRIVATE
    SFML::Graphics
    SFML::Window
    SFML::System
)

if(UNIX AND NOT APPLE)
    set_target_properties(flock PROPERTIES
        INSTALL_RPATH_USE_LINK_PATH TRUE
        BUILD_RPATH_USE_ORIGIN TRUE
    )
endif()

# On Windows, copy SFML DLLs to build directory for convenience
if(WIN32)
    add_custom_command(TARGET flock POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:flock>
            $<TARGET_FILE_DIR:flock>
        COMMAND_EXPAND_LISTS
    )
endif()


# Installation rules (optional)
install(TARGETS flock DESTINATION bin)
