# =============================================================================
# tcxOrbbec - Orbbec backend for tcxDepthCamera
# =============================================================================
#
# Implements the tcxDepthCamera interface on top of the Orbbec SDK v2
# (libobsensor). Unlike k4a, the Orbbec SDK v2 is cross-platform: Windows,
# Linux, and macOS (arm64/x86_64) are all supported, so this addon can be built
# and run on a Mac with a connected device.
#
# Finding the SDK:
#   - The Orbbec SDK v2 ships a CMake package config (OrbbecSDKConfig.cmake)
#     that defines the imported target `ob::OrbbecSDK`. Point CMake at it with
#     -DOrbbecSDK_DIR="<sdk>/lib/cmake/OrbbecSDK" (or install it system-wide).
#   - The SDK's runtime shared library (+ its `extensions/` folder) must sit
#     next to the app binary at runtime; the shared lib is bundled below, but
#     the extensions directory currently needs to be copied by hand (see TODO).
# =============================================================================

cmake_minimum_required(VERSION 3.16)

set(ADDON_NAME tcxOrbbec)

file(GLOB ADDON_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB ADDON_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)

add_library(${ADDON_NAME} STATIC ${ADDON_SOURCES} ${ADDON_HEADERS})

# tcxDepthCamera is a header-only interface addon sitting next to us. Anchor its
# path at TRUSSC_DIR (like use_addon.cmake does) rather than a path relative to
# THIS dir: in CI the addon is symlinked into TrussC/addons/, and CMake resolves
# the symlink to its real location, so "../tcxDepthCamera" would miss (Linux).
if(DEFINED TRUSSC_DIR)
    set(_TC_DEPTHCAMERA_SRC "${TRUSSC_DIR}/../addons/tcxDepthCamera/src")
else()
    set(_TC_DEPTHCAMERA_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../tcxDepthCamera/src")
endif()

target_include_directories(${ADDON_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${_TC_DEPTHCAMERA_SRC}
)

target_link_libraries(${ADDON_NAME} PUBLIC TrussC)

# -----------------------------------------------------------------------------
# Locate the Orbbec SDK v2 (libobsensor).
# -----------------------------------------------------------------------------
# Windows convenience: the v2 installer drops the SDK (with its CMake package
# config) under "C:/Program Files/OrbbecSDK <version>", so Windows users don't
# need to pass -DOrbbecSDK_DIR by hand. Auto-locate the config dir if it wasn't
# given. (macOS/Linux are left as-is — set OrbbecSDK_DIR yourself there.)
if(WIN32 AND NOT OrbbecSDK_DIR)
    file(GLOB _tcob_cfg_dirs
        "C:/Program Files/OrbbecSDK*/lib"
        "C:/Program Files/OrbbecSDK*/lib/cmake/OrbbecSDK"
        "C:/Program Files (x86)/OrbbecSDK*/lib"
        "C:/Program Files (x86)/OrbbecSDK*/lib/cmake/OrbbecSDK")
    foreach(_p IN LISTS _tcob_cfg_dirs)
        if(EXISTS "${_p}/OrbbecSDKConfig.cmake")
            set(OrbbecSDK_DIR "${_p}")
            message(STATUS "tcxOrbbec: auto-found Orbbec SDK config at ${OrbbecSDK_DIR}")
            break()
        endif()
    endforeach()
endif()

find_package(OrbbecSDK QUIET CONFIG)

# Resolve the imported target under whichever name the config exposes.
set(_TCOB_TARGET "")
if(TARGET ob::OrbbecSDK)
    set(_TCOB_TARGET ob::OrbbecSDK)
elseif(TARGET OrbbecSDK::OrbbecSDK)
    set(_TCOB_TARGET OrbbecSDK::OrbbecSDK)
elseif(TARGET OrbbecSDK)
    set(_TCOB_TARGET OrbbecSDK)
endif()

if(NOT _TCOB_TARGET AND DEFINED OrbbecSDK_LIBRARIES AND DEFINED OrbbecSDK_INCLUDE_DIRS)
    # Older/variant config that exports plain variables instead of a target.
    add_library(tcxOrbbec_OrbbecSDK INTERFACE IMPORTED)
    set_target_properties(tcxOrbbec_OrbbecSDK PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${OrbbecSDK_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${OrbbecSDK_LIBRARIES}")
    set(_TCOB_TARGET tcxOrbbec_OrbbecSDK)
endif()

if(NOT _TCOB_TARGET)
    message(FATAL_ERROR
        "tcxOrbbec: the Orbbec SDK v2 was not found.\n"
        "  Download it from https://github.com/orbbec/OrbbecSDK_v2/releases and\n"
        "  point CMake at its CMake config:\n"
        "    -DOrbbecSDK_DIR=\"<sdk>/lib/cmake/OrbbecSDK\"\n"
        "  (the config defines the imported target ob::OrbbecSDK).")
endif()

target_link_libraries(${ADDON_NAME} PUBLIC ${_TCOB_TARGET})
message(STATUS "tcxOrbbec: using Orbbec SDK target ${_TCOB_TARGET}")

# -----------------------------------------------------------------------------
# Co-locate the Orbbec SDK runtime shared library with the app binary. The SDK
# also needs its `extensions/` folder next to the binary at runtime; copying
# that automatically is a TODO (it is a directory, not a single file).
# tc_addon_bundle_file() is provided by use_addon.cmake.
# -----------------------------------------------------------------------------
if(COMMAND tc_addon_bundle_file)
    set(_TCOB_RUNTIME "")
    # IMPORTED_LOCATION points at the .dll/.so/.dylib for a SHARED imported target.
    get_target_property(_TCOB_RUNTIME ${_TCOB_TARGET} IMPORTED_LOCATION)
    if(NOT _TCOB_RUNTIME)
        get_target_property(_TCOB_RUNTIME ${_TCOB_TARGET} IMPORTED_LOCATION_RELEASE)
    endif()
    if(_TCOB_RUNTIME AND EXISTS "${_TCOB_RUNTIME}")
        tc_addon_bundle_file("${_TCOB_RUNTIME}")
        message(STATUS "tcxOrbbec: will bundle runtime ${_TCOB_RUNTIME}")
    endif()
endif()
