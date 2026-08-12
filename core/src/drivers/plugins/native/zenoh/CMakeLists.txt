# CMakeLists.txt for the Zenoh Plugin
# Builds a self-contained Zenoh pub/sub bridge plugin with the vendored
# zenoh-pico sources compiled in (same pattern as ethercat/SOEM and
# s7comm/Snap7). install.sh's build_native_plugins() auto-discovers this
# file and builds libzenoh_plugin.so into build/plugins/.
#
# The plugin consumes the editor-generated conf/zenoh.json (registered in
# plugins.conf under the name "zenoh" so the runtime matches config files
# to plugins by name) and bridges configured topics to PLC I/O.

cmake_minimum_required(VERSION 3.10)
project(zenoh_plugin C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Determine OpenPLC root directory for finding common headers
# When building standalone: calculate from plugin location
# When building from main project: pass -DOPENPLC_ROOT=<path>
if(NOT DEFINED OPENPLC_ROOT)
    get_filename_component(OPENPLC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../" ABSOLUTE)
endif()

message(STATUS "Zenoh Plugin - OpenPLC root: ${OPENPLC_ROOT}")

# =============================================================================
# Zenoh-Pico library (vendored, built as static so the plugin .so is
# self-contained like the other native plugins)
# =============================================================================
set(ZENOH_PICO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/zenoh-pico)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build zenoh-pico as static lib" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable zenoh-pico examples" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Disable zenoh-pico tests" FORCE)
add_subdirectory(${ZENOH_PICO_DIR} ${CMAKE_CURRENT_BINARY_DIR}/zenoh-pico)

# =============================================================================
# cJSON Library (shared utility for all native plugins)
# =============================================================================
set(CJSON_SOURCES
    ${OPENPLC_ROOT}/core/src/drivers/plugins/native/cjson/cJSON.c
)

# =============================================================================
# Plugin Source Files
# =============================================================================
set(PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/zenoh_plugin.c
    ${OPENPLC_ROOT}/core/src/drivers/plugins/native/plugin_logger.c
)

# =============================================================================
# Create Shared Library
# =============================================================================
add_library(zenoh_plugin SHARED
    ${CJSON_SOURCES}
    ${PLUGIN_SOURCES}
)

# =============================================================================
# Include Directories
# =============================================================================
target_include_directories(zenoh_plugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OPENPLC_ROOT}/core/src/drivers/plugins/native/cjson
    ${OPENPLC_ROOT}/core/src/drivers
    ${OPENPLC_ROOT}/core/src/drivers/plugins/native
    ${OPENPLC_ROOT}/core/src/lib
)

# =============================================================================
# Compiler Options
# =============================================================================
target_compile_options(zenoh_plugin PRIVATE
    -fPIC
    -Wno-unused-parameter
    -Wno-sign-compare
)

# =============================================================================
# Link Libraries
# =============================================================================
target_link_libraries(zenoh_plugin PRIVATE
    zenohpico_static
    pthread
    ${CMAKE_DL_LIBS}
)

# On MSYS2/Cygwin, ensure plugin API symbols (init, start_loop, etc.) are
# exported from the shared library so that dlsym() can resolve them.
if(CMAKE_SYSTEM_NAME STREQUAL "MSYS" OR CMAKE_SYSTEM_NAME STREQUAL "CYGWIN")
    set_target_properties(zenoh_plugin PROPERTIES
        LINK_FLAGS "-Wl,--export-all-symbols"
    )
endif()

# =============================================================================
# Output Settings
# =============================================================================
set_target_properties(zenoh_plugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
    PREFIX "lib"
    OUTPUT_NAME "zenoh_plugin"
)

# On Linux, use .so extension
if(UNIX)
    set_target_properties(zenoh_plugin PROPERTIES SUFFIX ".so")
endif()

# =============================================================================
# Install Target
# =============================================================================
install(TARGETS zenoh_plugin
    LIBRARY DESTINATION lib/openplc/plugins
    RUNTIME DESTINATION lib/openplc/plugins
)
